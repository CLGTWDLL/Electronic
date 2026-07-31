# -*- coding: utf-8 -*-
"""
K230/CanMV real-time detection for steel_ball_full_v2_180_nncase_2.9.0.kmodel.

Copy this file, k230_deploy_config.json, labels.txt and the kmodel to:
    /sdcard/mp_deployment_yolov8/

The camera image is displayed on the board LCD. Detection boxes, class names,
confidence, target count and FPS are drawn on the OSD layer.
"""

import gc
import os
import time
import ujson

import nncase_runtime as nn
import ulab.numpy as np

from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.PipeLine import PipeLine, ScopedTiming
from libs.Utils import *


DEPLOY_ROOT = "/sdcard/mp_deployment_yolov8/"
CONFIG_PATH = DEPLOY_ROOT + "k230_deploy_config.json"
DISPLAY_MODE = "lcd"
CAMERA_SIZE = [640, 640]
MAX_DETECTIONS = 20
GC_INTERVAL_FRAMES = 15
PERFORMANCE_LOG_INTERVAL = 60
DROP_OLD_FRAMES = 0
DEBUG_MODE = 0


def align_up(value, alignment):
    """Round value up without depending on firmware-specific ALIGN_UP."""
    return (value + alignment - 1) // alignment * alignment


def load_labels(path):
    result = []
    with open(path, "r") as label_file:
        for line in label_file:
            value = line.strip()
            if value:
                result.append(value)
    return result


def letterbox_parameters(source_size, target_size):
    scale = min(
        float(target_size[0]) / source_size[0],
        float(target_size[1]) / source_size[1],
    )
    resized_width = int(source_size[0] * scale)
    resized_height = int(source_size[1] * scale)
    width_padding = target_size[0] - resized_width
    height_padding = target_size[1] - resized_height
    left = width_padding // 2
    right = width_padding - left
    top = height_padding // 2
    bottom = height_padding - top
    return top, bottom, left, right, scale


def intersection_over_union(first, second):
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    width = max(0.0, right - left)
    height = max(0.0, bottom - top)
    intersection = width * height
    first_area = max(0.0, first[2] - first[0]) * max(
        0.0, first[3] - first[1]
    )
    second_area = max(0.0, second[2] - second[0]) * max(
        0.0, second[3] - second[1]
    )
    union = first_area + second_area - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union


def class_aware_nms(detections, iou_threshold, limit):
    # The candidate list is already sorted from high to low confidence.
    selected = []
    for candidate in detections:
        keep = True
        for previous in selected:
            if (
                candidate[0] == previous[0]
                and intersection_over_union(candidate[2:], previous[2:])
                > iou_threshold
            ):
                keep = False
                break
        if keep:
            selected.append(candidate)
            if len(selected) >= limit:
                break
    return selected


class YoloV5RawDetection(AIBase):
    def __init__(
        self,
        kmodel_path,
        labels,
        model_input_size,
        confidence_threshold,
        nms_threshold,
        camera_size,
        display_size,
        debug_mode=0,
    ):
        super().__init__(
            kmodel_path, model_input_size, camera_size, debug_mode
        )
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.camera_size = [
            align_up(camera_size[0], 16),
            camera_size[1],
        ]
        self.display_size = [
            align_up(display_size[0], 16),
            display_size[1],
        ]
        self.debug_mode = debug_mode
        self.colors = get_colors(len(labels))
        self._first_model_input_checked = False
        self._vector_filter_available = True
        self._vector_filter_reported = False
        self._postprocess_count = 0
        self._postprocess_time_total_us = 0
        self.ai2d = Ai2d(debug_mode)
        # CanMV v1.4.3 ulab ndarray has no astype(). Let AI2D produce float
        # data, then normalize it directly in preprocess().
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.float,
        )

    def config_preprocess(self, input_image_size=None):
        source_size = input_image_size or self.camera_size
        (
            top,
            bottom,
            left,
            right,
            self.letterbox_scale,
        ) = letterbox_parameters(source_size, self.model_input_size)
        self.pad_left = left
        self.pad_top = top
        self.ai2d.pad(
            [0, 0, 0, 0, top, bottom, left, right],
            0,
            [114, 114, 114],
        )
        self.ai2d.resize(
            nn.interp_method.tf_bilinear,
            nn.interp_mode.half_pixel,
        )
        self.ai2d.build(
            [1, 3, source_size[1], source_size[0]],
            [
                1,
                3,
                self.model_input_size[1],
                self.model_input_size[0],
            ],
        )

    def preprocess(self, input_np):
        """Normalize AI2D's letterboxed RGB NCHW output for the kmodel."""
        # PipeLine(rgb888p_size=...) supplies RGB planar frames.  AI2D keeps
        # that channel order while it letterboxes to NCHW, so do not swap R/B.
        model_input = input_np / 255.0

        if not self._first_model_input_checked:
            input_min = float(np.min(input_np))
            input_max = float(np.max(input_np))
            normalized_min = float(np.min(model_input))
            normalized_max = float(np.max(model_input))
            print("First preprocess frame")
            print("Input shape:", input_np.shape)
            print("Input min/max:", input_min, input_max)
            print("Normalized shape:", model_input.shape)
            print(
                "Normalized min/max:",
                normalized_min,
                normalized_max,
            )
            self._first_model_input_checked = True

        return [nn.from_numpy(model_input)]

    def postprocess(self, results):
        """
        Decode [1, 25200, 5 + class_count].

        The exported model already supplies cx/cy/w/h and probabilities. KPU
        quantization/dequantization is handled by nncase_runtime.
        """
        postprocess_start_us = time.ticks_us()
        output = results[0]
        field_count = 5 + len(self.labels)
        if len(output.shape) == 3:
            output = output[0]
        if len(output.shape) != 2:
            output = output.reshape((-1, field_count))

        # Some exporters describe [1, 25200, 6] but store [1, 6, 25200].
        if output.shape[1] != field_count:
            if output.shape[0] == field_count:
                output = output.transpose()
            else:
                raise ValueError(
                    "Unexpected runtime output shape: " + str(output.shape)
                )

        candidates = []
        filtered_output = None

        # This is a one-class model. Calculate objectness * class probability
        # for all 25200 rows, then use ulab.compress to retain matching rows.
        # Unlike ulab.where, compress can select rows without building indices.
        # This does not truncate candidates or alter NMS.
        if len(self.labels) == 1 and self._vector_filter_available:
            try:
                combined_scores = output[:, 4] * output[:, 5]
                filtered_output = np.compress(
                    combined_scores >= self.confidence_threshold,
                    output,
                    axis=0,
                )
                del combined_scores
                if not self._vector_filter_reported:
                    print("Vector candidate filter enabled: np.compress")
                    self._vector_filter_reported = True
            except BaseException as filter_error:
                self._vector_filter_available = False
                filtered_output = None
                print(
                    "Vector filter unavailable; using compatible loop:",
                    filter_error,
                )

        rows_to_process = (
            filtered_output if filtered_output is not None else output
        )

        for row_index in range(rows_to_process.shape[0]):
            row = rows_to_process[row_index]
            objectness = float(row[4])
            if objectness < self.confidence_threshold:
                continue

            best_class = 0
            best_class_probability = float(row[5])
            for class_index in range(1, len(self.labels)):
                probability = float(row[5 + class_index])
                if probability > best_class_probability:
                    best_class = class_index
                    best_class_probability = probability

            score = objectness * best_class_probability
            if score < self.confidence_threshold:
                continue

            center_x = (float(row[0]) - self.pad_left) / self.letterbox_scale
            center_y = (float(row[1]) - self.pad_top) / self.letterbox_scale
            width = float(row[2]) / self.letterbox_scale
            height = float(row[3]) / self.letterbox_scale
            x1 = max(0.0, center_x - width / 2.0)
            y1 = max(0.0, center_y - height / 2.0)
            x2 = min(float(self.camera_size[0] - 1), center_x + width / 2.0)
            y2 = min(float(self.camera_size[1] - 1), center_y + height / 2.0)
            if x2 > x1 and y2 > y1:
                candidates.append(
                    [best_class, score, x1, y1, x2, y2]
                )

        candidates.sort(key=lambda item: item[1], reverse=True)
        detections = class_aware_nms(
            candidates, self.nms_threshold, MAX_DETECTIONS
        )
        postprocess_elapsed_us = time.ticks_diff(
            time.ticks_us(), postprocess_start_us
        )
        self._postprocess_count += 1
        self._postprocess_time_total_us += postprocess_elapsed_us
        if self._postprocess_count % PERFORMANCE_LOG_INTERVAL == 0:
            average_ms = (
                self._postprocess_time_total_us
                / self._postprocess_count
                / 1000.0
            )
            print(
                "Postprocess avg ms:",
                round(average_ms, 2),
                "candidates:",
                len(candidates),
                "detections:",
                len(detections),
            )
        return detections

    def draw_result(self, osd_image, detections, fps):
        osd_image.clear()
        for detection in detections:
            class_id, score, x1, y1, x2, y2 = detection
            x = int(x1 * self.display_size[0] / self.camera_size[0])
            y = int(y1 * self.display_size[1] / self.camera_size[1])
            width = int(
                (x2 - x1) * self.display_size[0] / self.camera_size[0]
            )
            height = int(
                (y2 - y1) * self.display_size[1] / self.camera_size[1]
            )
            color = self.colors[class_id]
            # get_colors() may return ARGB; drawing on PipeLine OSD accepts it.
            osd_image.draw_rectangle(
                x, y, width, height, color=color, thickness=3
            )
            label_y = max(0, y - 30)
            osd_image.draw_string_advanced(
                x,
                label_y,
                24,
                self.labels[class_id] + " " + str(round(score, 2)),
                color=color,
            )

        osd_image.draw_string_advanced(
            8,
            8,
            22,
            "Objects: "
            + str(len(detections))
            + "  FPS: "
            + str(round(fps, 1)),
            color=(0, 255, 0),
        )


def validate_config(config, labels):
    if config.get("model_type") != "YOLOv5RawOutput":
        raise ValueError("model_type must be YOLOv5RawOutput")
    if config.get("runtime", {}).get("version") != "2.9.0":
        raise ValueError("This script expects an nncase 2.9.0 kmodel")
    if config.get("num_classes") != len(labels):
        raise ValueError("num_classes does not match labels.txt")
    output_shape = config.get("output", {}).get("shape")
    if output_shape != [1, 25200, 5 + len(labels)]:
        raise ValueError("Unexpected model output shape: " + str(output_shape))


pipeline = None
detector = None
os.exitpoint(os.EXITPOINT_ENABLE)

try:
    print("Steel-ball detector starting")
    print("Config:", CONFIG_PATH)
    with open(CONFIG_PATH, "r") as config_file:
        config = ujson.load(config_file)
    print("Config loaded")
    labels = load_labels(DEPLOY_ROOT + config["labels_path"])
    validate_config(config, labels)
    print("Labels:", labels)

    input_size = [
        config["inference_width"],
        config["inference_height"],
    ]
    postprocess_config = config["output"]["postprocess"]

    pipeline = PipeLine(
        rgb888p_size=CAMERA_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pipeline.create()
    display_size = pipeline.get_display_size()
    print("Camera/display ready:", display_size)

    detector = YoloV5RawDetection(
        DEPLOY_ROOT + config["kmodel_path"],
        labels,
        input_size,
        postprocess_config["confidence_threshold"],
        postprocess_config["nms_iou_threshold"],
        CAMERA_SIZE,
        display_size,
        DEBUG_MODE,
    )
    detector.config_preprocess()
    print("Kmodel and preprocess ready")
    print("Old frames dropped before inference:", DROP_OLD_FRAMES)

    last_tick = time.ticks_ms()
    fps = 0.0
    frame_count = 0
    while True:
        os.exitpoint()
        with ScopedTiming("total", DEBUG_MODE > 0):
            for _ in range(DROP_OLD_FRAMES):
                pipeline.get_frame()
            frame = pipeline.get_frame()
            detections = detector.run(frame)

            current_tick = time.ticks_ms()
            elapsed = time.ticks_diff(current_tick, last_tick)
            last_tick = current_tick
            if elapsed > 0:
                instant_fps = 1000.0 / elapsed
                fps = instant_fps if fps == 0.0 else fps * 0.8 + instant_fps * 0.2

            detector.draw_result(
                pipeline.osd_img, detections, fps
            )
            pipeline.show_image()
            frame_count += 1
            if frame_count % PERFORMANCE_LOG_INTERVAL == 0:
                print("Display FPS:", round(fps, 2))
            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()

        time.sleep_ms(1)

except KeyboardInterrupt:
    print("User stopped steel-ball detection.")
except BaseException as error:
    print("Steel-ball detection exception:", error)
    try:
        with open("/sdcard/k230_error.txt", "w") as error_file:
            error_file.write("Steel-ball detection exception:\n")
            error_file.write(str(error))
            error_file.write("\n")
        print("Error saved to /sdcard/k230_error.txt")
    except BaseException as log_error:
        print("Could not save error log:", log_error)
finally:
    if detector is not None:
        try:
            detector.deinit()
        except BaseException as error:
            print("Detector deinit warning:", error)
    if pipeline is not None:
        try:
            pipeline.destroy()
        except BaseException as error:
            print("Pipeline destroy warning:", error)
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    gc.collect()
    print("Resources released.")

# -*- coding: utf-8 -*-
"""
K230 CanMV v1.4.3 real-time steel-ball detection for:
    steel_ball_416_nncase_2.9.0.kmodel

Required SD-card layout:
    /sdcard/mp_deployment_416/
        k230_deploy_config.json
        labels.txt
        steel_ball_416_nncase_2.9.0.kmodel

Copy this script to /sdcard/main.py for automatic startup.
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

# Keep a square inference source. It has the same aspect ratio as the 416
# model, so the documented resize-then-pad letterbox has zero padding and
# cannot be distorted by firmware-specific AI2D operation ordering.
CAMERA_SIZE = [640, 640]
EXPECTED_INPUT_SHAPE = [1, 3, 416, 416]
EXPECTED_OUTPUT_SHAPE = [1, 10647, 6]
EXPECTED_KMODEL = "steel_ball_416_nncase_2.9.0.kmodel"

MAX_DETECTIONS = 20
GC_INTERVAL_FRAMES = 15
PERFORMANCE_LOG_INTERVAL = 60
DEBUG_MODE = 0


def align_up(value, alignment):
    return (value + alignment - 1) // alignment * alignment


def load_labels(path):
    labels = []
    with open(path, "r") as label_file:
        for line in label_file:
            label = line.strip()
            if label:
                labels.append(label)
    return labels


def letterbox_parameters(source_size, target_size):
    """
    Reproduce DEPLOY_NOTES.md rounding in model-coordinate space.

    With the fixed 640x640 camera source this returns:
        resized=416x416, padding=(0, 0, 0, 0), sx=sy=0.65
    """
    source_width, source_height = source_size
    target_width, target_height = target_size
    ratio = min(
        float(target_width) / source_width,
        float(target_height) / source_height,
    )
    resized_width = int(round(source_width * ratio))
    resized_height = int(round(source_height * ratio))
    pad_left = (target_width - resized_width) // 2
    pad_top = (target_height - resized_height) // 2
    pad_right = target_width - resized_width - pad_left
    pad_bottom = target_height - resized_height - pad_top
    scale_x = float(resized_width) / source_width
    scale_y = float(resized_height) / source_height
    return (
        resized_width,
        resized_height,
        pad_left,
        pad_top,
        pad_right,
        pad_bottom,
        scale_x,
        scale_y,
    )


def intersection_over_union(first, second):
    left = max(first[2], second[2])
    top = max(first[3], second[3])
    right = min(first[4], second[4])
    bottom = min(first[5], second[5])
    intersection_width = max(0.0, right - left)
    intersection_height = max(0.0, bottom - top)
    intersection = intersection_width * intersection_height
    first_area = max(0.0, first[4] - first[2]) * max(
        0.0, first[5] - first[3]
    )
    second_area = max(0.0, second[4] - second[2]) * max(
        0.0, second[5] - second[3]
    )
    union = first_area + second_area - intersection
    return 0.0 if union <= 0.0 else intersection / union


def class_aware_nms(candidates, iou_threshold, limit):
    selected = []
    for candidate in candidates:
        keep = True
        for previous in selected:
            if (
                candidate[0] == previous[0]
                and intersection_over_union(candidate, previous)
                > iou_threshold
            ):
                keep = False
                break
        if keep:
            selected.append(candidate)
            if len(selected) >= limit:
                break
    return selected


class SteelBall416Detector(AIBase):
    def __init__(
        self,
        kmodel_path,
        labels,
        confidence_threshold,
        nms_threshold,
        camera_size,
        display_size,
        debug_mode=0,
    ):
        model_input_size = [
            EXPECTED_INPUT_SHAPE[3],
            EXPECTED_INPUT_SHAPE[2],
        ]
        super().__init__(
            kmodel_path,
            model_input_size,
            camera_size,
            debug_mode,
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
        self.first_input_check = True
        self.first_output_check = True
        self.vector_filter_available = True
        self.vector_filter_reported = False
        self.postprocess_count = 0
        self.postprocess_total_us = 0

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.float,
        )

    def config_preprocess(self):
        (
            resized_width,
            resized_height,
            self.pad_left,
            self.pad_top,
            pad_right,
            pad_bottom,
            self.scale_x,
            self.scale_y,
        ) = letterbox_parameters(
            self.camera_size,
            self.model_input_size,
        )

        # CAMERA_SIZE and model input are both square, so padding must be zero.
        # This assertion prevents a silent departure from DEPLOY_NOTES.md.
        if (
            self.pad_left != 0
            or self.pad_top != 0
            or pad_right != 0
            or pad_bottom != 0
        ):
            raise ValueError(
                "This script requires a square camera source; letterbox pad="
                + str(
                    (
                        self.pad_left,
                        self.pad_top,
                        pad_right,
                        pad_bottom,
                    )
                )
            )

        self.ai2d.resize(
            nn.interp_method.tf_bilinear,
            nn.interp_mode.half_pixel,
        )
        self.ai2d.build(
            [
                1,
                3,
                self.camera_size[1],
                self.camera_size[0],
            ],
            [
                1,
                3,
                self.model_input_size[1],
                self.model_input_size[0],
            ],
        )
        print(
            "Preprocess:",
            self.camera_size,
            "->",
            (resized_width, resized_height),
            "RGB NCHW float /255",
        )

    def preprocess(self, input_np):
        """
        Run AI2D explicitly, then normalize its 416x416 float output.

        Overriding AIBase.preprocess means AIBase will not run AI2D for us.
        CanMV's Ai2d wrapper returns an nncase tensor on this firmware.
        """
        ai2d_output = self.ai2d.run(input_np)

        # Keep compatibility with wrappers that return a one-item list.
        if isinstance(ai2d_output, (list, tuple)):
            ai2d_output = ai2d_output[0]

        if hasattr(ai2d_output, "to_numpy"):
            resized_input = ai2d_output.to_numpy()
        else:
            resized_input = ai2d_output

        # AI2D produces planar RGB float values in the 0..255 range.
        # Do not call astype(): CanMV v1.4.3 ulab ndarray lacks that method.
        model_input = resized_input / 255.0

        if self.first_input_check:
            print("First model input shape:", model_input.shape)
            print(
                "First model input min/max:",
                float(np.min(model_input)),
                float(np.max(model_input)),
            )
            if (
                model_input.shape != (1, 3, 416, 416)
                and model_input.shape != (3, 416, 416)
            ):
                raise ValueError(
                    "Runtime model input must end in (3, 416, 416), got "
                    + str(model_input.shape)
                )
            self.first_input_check = False

        return [nn.from_numpy(model_input)]

    def postprocess(self, results):
        start_us = time.ticks_us()
        output = results[0]

        if self.first_output_check:
            print("First model output shape:", output.shape)
            if output.shape != (1, 10647, 6):
                raise ValueError(
                    "Runtime output must be "
                    + str(EXPECTED_OUTPUT_SHAPE)
                    + ", got "
                    + str(output.shape)
                )
            self.first_output_check = False

        output = output[0]
        filtered_output = None

        # Exact one-class score filtering in ulab native code.
        if self.vector_filter_available:
            try:
                combined_scores = output[:, 4] * output[:, 5]
                filtered_output = np.compress(
                    combined_scores >= self.confidence_threshold,
                    output,
                    axis=0,
                )
                del combined_scores
                if not self.vector_filter_reported:
                    print("Vector candidate filter: np.compress")
                    self.vector_filter_reported = True
            except BaseException as filter_error:
                self.vector_filter_available = False
                print(
                    "Vector filter unavailable; using compatible loop:",
                    filter_error,
                )

        rows = filtered_output if filtered_output is not None else output
        candidates = []

        for row_index in range(rows.shape[0]):
            row = rows[row_index]
            objectness = float(row[4])
            class_probability = float(row[5])
            score = objectness * class_probability
            if score < self.confidence_threshold:
                continue

            # Output xywh belongs to the 416x416 letterbox coordinate system.
            center_x = (
                float(row[0]) - self.pad_left
            ) / self.scale_x
            center_y = (
                float(row[1]) - self.pad_top
            ) / self.scale_y
            width = float(row[2]) / self.scale_x
            height = float(row[3]) / self.scale_y

            x1 = max(0.0, center_x - width / 2.0)
            y1 = max(0.0, center_y - height / 2.0)
            x2 = min(
                float(self.camera_size[0] - 1),
                center_x + width / 2.0,
            )
            y2 = min(
                float(self.camera_size[1] - 1),
                center_y + height / 2.0,
            )
            if x2 > x1 and y2 > y1:
                candidates.append(
                    [0, score, x1, y1, x2, y2]
                )

        candidates.sort(key=lambda item: item[1], reverse=True)
        detections = class_aware_nms(
            candidates,
            self.nms_threshold,
            MAX_DETECTIONS,
        )

        elapsed_us = time.ticks_diff(time.ticks_us(), start_us)
        self.postprocess_count += 1
        self.postprocess_total_us += elapsed_us
        if self.postprocess_count % PERFORMANCE_LOG_INTERVAL == 0:
            print(
                "Postprocess avg ms:",
                round(
                    self.postprocess_total_us
                    / self.postprocess_count
                    / 1000.0,
                    2,
                ),
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
            display_x = int(
                x1 * self.display_size[0] / self.camera_size[0]
            )
            display_y = int(
                y1 * self.display_size[1] / self.camera_size[1]
            )
            display_width = int(
                (x2 - x1)
                * self.display_size[0]
                / self.camera_size[0]
            )
            display_height = int(
                (y2 - y1)
                * self.display_size[1]
                / self.camera_size[1]
            )
            color = self.colors[class_id]
            osd_image.draw_rectangle(
                display_x,
                display_y,
                display_width,
                display_height,
                color=color,
                thickness=3,
            )
            osd_image.draw_string_advanced(
                display_x,
                max(0, display_y - 30),
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
    if config.get("kmodel_path") != EXPECTED_KMODEL:
        raise ValueError("Unexpected KModel filename")
    if config.get("runtime", {}).get("version") != "2.9.0":
        raise ValueError("Expected nncase runtime 2.9.0")
    if config.get("input", {}).get("shape") != EXPECTED_INPUT_SHAPE:
        raise ValueError("Unexpected config input shape")
    if config.get("input", {}).get("dtype") != "float32":
        raise ValueError("KModel external input must be float32")
    if config.get("input", {}).get("layout") != "NCHW":
        raise ValueError("KModel input layout must be NCHW")
    if config.get("input", {}).get("color_order") != "RGB":
        raise ValueError("KModel input color order must be RGB")
    if config.get("output", {}).get("shape") != EXPECTED_OUTPUT_SHAPE:
        raise ValueError("Unexpected config output shape")
    if config.get("output", {}).get("dtype") != "float32":
        raise ValueError("KModel output must be float32")
    if config.get("output", {}).get("nms_required") is not True:
        raise ValueError("Config must require NMS")
    if labels != ["Steel_Ball"]:
        raise ValueError("labels.txt must contain Steel_Ball")


pipeline = None
detector = None
os.exitpoint(os.EXITPOINT_ENABLE)

try:
    print("Steel-ball 416 detector starting")
    print("Config:", CONFIG_PATH)
    with open(CONFIG_PATH, "r") as config_file:
        config = ujson.load(config_file)
    labels = load_labels(DEPLOY_ROOT + config["labels_path"])
    validate_config(config, labels)
    print("Config and labels validated:", labels)

    pipeline = PipeLine(
        rgb888p_size=CAMERA_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pipeline.create()
    display_size = pipeline.get_display_size()
    print("Camera/display ready:", display_size)

    detector = SteelBall416Detector(
        DEPLOY_ROOT + config["kmodel_path"],
        labels,
        config["postprocess"]["confidence_threshold"],
        config["postprocess"]["nms_iou_threshold"],
        CAMERA_SIZE,
        display_size,
        DEBUG_MODE,
    )
    detector.config_preprocess()
    print("Kmodel and preprocess ready")

    frame_count = 0
    last_tick = time.ticks_ms()
    fps = 0.0

    while True:
        os.exitpoint()
        with ScopedTiming("total", DEBUG_MODE > 0):
            frame = pipeline.get_frame()
            detections = detector.run(frame)

            current_tick = time.ticks_ms()
            elapsed_ms = time.ticks_diff(current_tick, last_tick)
            last_tick = current_tick
            if elapsed_ms > 0:
                instant_fps = 1000.0 / elapsed_ms
                fps = (
                    instant_fps
                    if fps == 0.0
                    else fps * 0.8 + instant_fps * 0.2
                )

            detector.draw_result(
                pipeline.osd_img,
                detections,
                fps,
            )
            pipeline.show_image()

            frame_count += 1
            if frame_count % PERFORMANCE_LOG_INTERVAL == 0:
                print("Display FPS:", round(fps, 2))
            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()

        time.sleep_ms(1)

except KeyboardInterrupt:
    print("User stopped steel-ball 416 detection.")
except BaseException as error:
    error_text = str(error)
    if error_text == "IDE interrupt":
        print("IDE stopped steel-ball 416 detection.")
    else:
        print("Steel-ball 416 detection exception:", error)
        try:
            with open("/sdcard/k230_error.txt", "w") as error_file:
                error_file.write("Steel-ball 416 detection exception:\n")
                error_file.write(error_text)
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

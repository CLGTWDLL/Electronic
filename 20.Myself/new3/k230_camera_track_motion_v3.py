# -*- coding: utf-8 -*-
"""
Low-latency K230/CanMV v1.4.3 tracker for the motion-v3 steel-ball model.

SD-card layout:
    /sdcard/mp_deployment_yolov8/
        k230_deploy_config.json
        labels.txt
        steel_ball_416_motion_v3_nncase_2.9.0.kmodel

Copy this script to /sdcard/main.py.
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
MODEL_SIZE = [416, 416]

EXPECTED_KMODEL = "steel_ball_416_motion_v3_nncase_2.9.0.kmodel"
EXPECTED_INPUT_SHAPE = [1, 3, 416, 416]
EXPECTED_OUTPUT_SHAPE = [1, 10647, 6]

MAX_DETECTIONS = 10
GC_INTERVAL_FRAMES = 30
PERFORMANCE_LOG_INTERVAL = 120
DEBUG_MODE = 0


def align_up(value, alignment):
    return (value + alignment - 1) // alignment * alignment


def load_labels(path):
    labels = []
    with open(path, "r") as label_file:
        for line in label_file:
            value = line.strip()
            if value:
                labels.append(value)
    return labels


def intersection_over_union(first, second):
    left = max(first[2], second[2])
    top = max(first[3], second[3])
    right = min(first[4], second[4])
    bottom = min(first[5], second[5])
    width = max(0.0, right - left)
    height = max(0.0, bottom - top)
    intersection = width * height
    first_area = max(0.0, first[4] - first[2]) * max(
        0.0, first[5] - first[3]
    )
    second_area = max(0.0, second[4] - second[2]) * max(
        0.0, second[5] - second[3]
    )
    union = first_area + second_area - intersection
    return 0.0 if union <= 0.0 else intersection / union


def nms(candidates, iou_threshold, limit):
    selected = []
    for candidate in candidates:
        keep = True
        for previous in selected:
            if (
                intersection_over_union(candidate, previous)
                > iou_threshold
            ):
                keep = False
                break
        if keep:
            selected.append(candidate)
            if len(selected) >= limit:
                break
    return selected


class MotionV3Detector(AIBase):
    def __init__(
        self,
        kmodel_path,
        labels,
        confidence_threshold,
        nms_threshold,
        display_size,
        debug_mode=0,
    ):
        super().__init__(
            kmodel_path,
            MODEL_SIZE,
            CAMERA_SIZE,
            debug_mode,
        )
        self.labels = labels
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.camera_size = [
            align_up(CAMERA_SIZE[0], 16),
            CAMERA_SIZE[1],
        ]
        self.display_size = [
            align_up(display_size[0], 16),
            display_size[1],
        ]
        self.colors = get_colors(len(labels))
        self.debug_mode = debug_mode

        # 640x640 -> 416x416 has no letterbox padding.
        self.scale_x = 416.0 / 640.0
        self.scale_y = 416.0 / 640.0
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
            np.uint8,
        )

    def config_preprocess(self):
        self.ai2d.resize(
            nn.interp_method.tf_bilinear,
            nn.interp_mode.half_pixel,
        )
        self.ai2d.build(
            [1, 3, 640, 640],
            [1, 3, 416, 416],
        )
        print(
            "Preprocess: 640x640 -> 416x416, RGB NCHW float /255"
        )

    def preprocess(self, input_np):
        # AIBase does not run AI2D when preprocess() is overridden.
        ai2d_output = self.ai2d.run(input_np)
        if isinstance(ai2d_output, (list, tuple)):
            ai2d_output = ai2d_output[0]

        if hasattr(ai2d_output, "to_numpy"):
            resized_input = ai2d_output.to_numpy()
        else:
            resized_input = ai2d_output

        # AI2D returns uint8; division creates the float model input without
        # calling astype(), which is unavailable in CanMV v1.4.3 ulab.
        model_input = resized_input / 255.0

        if self.first_input_check:
            print("First input shape:", model_input.shape)
            print("First input dtype:", model_input.dtype)
            print(
                "First input min/max:",
                float(np.min(model_input)),
                float(np.max(model_input)),
            )
            if (
                model_input.shape != (1, 3, 416, 416)
                and model_input.shape != (3, 416, 416)
            ):
                raise ValueError(
                    "Expected model input ending in (3,416,416), got "
                    + str(model_input.shape)
                )
            self.first_input_check = False

        return [nn.from_numpy(model_input)]

    def postprocess(self, results):
        start_us = time.ticks_us()
        output = results[0]

        if self.first_output_check:
            print("First output shape:", output.shape)
            if output.shape != (1, 10647, 6):
                raise ValueError(
                    "Expected output (1,10647,6), got "
                    + str(output.shape)
                )
            self.first_output_check = False

        output = output[0]
        filtered_output = None

        # Native ulab filtering avoids a Python loop over all 10647 rows.
        if self.vector_filter_available:
            try:
                scores = output[:, 4] * output[:, 5]
                filtered_output = np.compress(
                    scores >= self.confidence_threshold,
                    output,
                    axis=0,
                )
                del scores
                if not self.vector_filter_reported:
                    print("Fast candidate filter: np.compress")
                    self.vector_filter_reported = True
            except BaseException as filter_error:
                self.vector_filter_available = False
                print(
                    "Fast filter unavailable; using compatible loop:",
                    filter_error,
                )

        rows = filtered_output if filtered_output is not None else output
        candidates = []

        for row_index in range(rows.shape[0]):
            row = rows[row_index]
            score = float(row[4]) * float(row[5])
            if score < self.confidence_threshold:
                continue

            center_x = float(row[0]) / self.scale_x
            center_y = float(row[1]) / self.scale_y
            width = float(row[2]) / self.scale_x
            height = float(row[3]) / self.scale_y
            x1 = max(0.0, center_x - width / 2.0)
            y1 = max(0.0, center_y - height / 2.0)
            x2 = min(639.0, center_x + width / 2.0)
            y2 = min(639.0, center_y + height / 2.0)
            if x2 > x1 and y2 > y1:
                candidates.append([0, score, x1, y1, x2, y2])

        candidates.sort(key=lambda item: item[1], reverse=True)
        detections = nms(
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
            display_x = int(x1 * self.display_size[0] / 640)
            display_y = int(y1 * self.display_size[1] / 640)
            display_width = int(
                (x2 - x1) * self.display_size[0] / 640
            )
            display_height = int(
                (y2 - y1) * self.display_size[1] / 640
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

        # A single short status string is cheaper than a label for every box.
        if detections:
            status = (
                "Ball "
                + str(round(detections[0][1], 2))
                + "  FPS "
                + str(round(fps, 1))
            )
        else:
            status = "No ball  FPS " + str(round(fps, 1))
        osd_image.draw_string_advanced(
            8,
            8,
            22,
            status,
            color=(0, 255, 0),
        )


def validate_config(config, labels):
    if config.get("kmodel_path") != EXPECTED_KMODEL:
        raise ValueError("Unexpected KModel filename")
    if config.get("runtime", {}).get("version") != "2.9.0":
        raise ValueError("Expected nncase runtime 2.9.0")
    input_config = config.get("input", {})
    output_config = config.get("output", {})
    if input_config.get("shape") != EXPECTED_INPUT_SHAPE:
        raise ValueError("Unexpected input shape")
    if input_config.get("layout") != "NCHW":
        raise ValueError("Input layout must be NCHW")
    if input_config.get("dtype") != "float32":
        raise ValueError("External input must be float32")
    if input_config.get("color_order") != "RGB":
        raise ValueError("Input color order must be RGB")
    if output_config.get("shape") != EXPECTED_OUTPUT_SHAPE:
        raise ValueError("Unexpected output shape")
    if output_config.get("dtype") != "float32":
        raise ValueError("Output must be float32")
    if output_config.get("nms_required") is not True:
        raise ValueError("NMS must be enabled")
    if labels != ["Steel_Ball"]:
        raise ValueError("labels.txt must contain Steel_Ball")


pipeline = None
detector = None
os.exitpoint(os.EXITPOINT_ENABLE)

try:
    print("Steel-ball motion-v3 tracker starting")
    print("Config:", CONFIG_PATH)
    with open(CONFIG_PATH, "r") as config_file:
        config = ujson.load(config_file)
    labels = load_labels(DEPLOY_ROOT + config["labels_path"])
    validate_config(config, labels)
    print("Config validated:", labels)

    pipeline = PipeLine(
        rgb888p_size=CAMERA_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pipeline.create()
    display_size = pipeline.get_display_size()
    print("Camera/display ready:", display_size)

    detector = MotionV3Detector(
        DEPLOY_ROOT + config["kmodel_path"],
        labels,
        config["postprocess"]["confidence_threshold"],
        config["postprocess"]["nms_iou_threshold"],
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

except KeyboardInterrupt:
    print("User stopped motion-v3 tracker.")
except BaseException as error:
    error_text = str(error)
    if error_text == "IDE interrupt":
        print("IDE stopped motion-v3 tracker.")
    else:
        print("Motion-v3 tracker exception:", error)
        try:
            with open("/sdcard/k230_error.txt", "w") as error_file:
                error_file.write("Motion-v3 tracker exception:\n")
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

# -*- coding: utf-8 -*-
"""
K230 YOLOv8 steel-ball detection with UART output.

Upload this script as /sdcard/main.py and upload deploy_config.json plus
best_320x256.kmodel to /sdcard/mp_deployment_yolov8/.

UART frame:
    $length,14,x,y,w,h,Steel_ball#

The bounding box sent to the STM32 always uses 1280 x 720 coordinates,
independent of the CanMV IDE preview resolution.
"""

import gc
import os
import time

import aidemo
import nncase_runtime as nn
import ulab.numpy as np

from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.PipeLine import PipeLine, ScopedTiming
from libs.Utils import *
from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart


DISPLAY_MODE = "lcd"
AI_FRAME_SIZE = [1280, 720]
DEPLOY_ROOT = "/sdcard/mp_deployment_yolov8/"
EXPECTED_MODEL_TYPE = "YOLOv8Det"


def align_up(value, alignment):
    """Return value rounded up to the requested hardware alignment."""
    return (value + alignment - 1) // alignment * alignment


def calculate_letterbox_padding(input_size, output_size):
    """
    Match the YOLOv8 example's top-left letterbox preprocessing.

    Padding is placed on the bottom and right before AI2D resizes the padded
    image to the model input. Keeping this local avoids firmware-dependent
    helpers that are absent from some CanMV v1.4.3 images.
    """
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio = min(ratio_w, ratio_h)
    new_width = int(ratio * input_size[0])
    new_height = int(ratio * input_size[1])
    width_difference = (output_size[0] - new_width) / 2
    height_difference = (output_size[1] - new_height) / 2

    top = 0
    bottom = int(round(height_difference * 2 + 0.1))
    left = 0
    right = int(round(width_difference * 2 - 0.1))
    return top, bottom, left, right, ratio


class YoloV8DetectionApp(AIBase):
    """YOLOv8 inference with boxes returned in AI_FRAME_SIZE coordinates."""

    def __init__(
        self,
        kmodel_path,
        labels,
        model_input_size,
        confidence_threshold,
        nms_threshold,
        max_boxes_num,
        rgb888p_size,
        debug_mode=0,
    ):
        super().__init__(
            kmodel_path, model_input_size, rgb888p_size, debug_mode
        )
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        self.rgb888p_size = [
            align_up(rgb888p_size[0], 16),
            rgb888p_size[1],
        ]
        self.debug_mode = debug_mode

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            source_size = (
                input_image_size
                if input_image_size is not None
                else self.rgb888p_size
            )
            (
                top,
                bottom,
                left,
                right,
                self.scale,
            ) = calculate_letterbox_padding(
                source_size,
                self.model_input_size,
            )
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [128, 128, 128],
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

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            predictions = results[0][0].transpose()
            return aidemo.yolov8_det_postprocess(
                predictions.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [AI_FRAME_SIZE[1], AI_FRAME_SIZE[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num,
            )


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def valid_detections(detections):
    return (
        detections is not None
        and len(detections) >= 3
        and detections[0] is not None
        and len(detections[0]) > 0
    )


def normalize_box(box):
    """Clamp an x/y/w/h box to the 1280 x 720 coordinate frame."""
    x = clamp(int(round(box[0])), 0, AI_FRAME_SIZE[0] - 1)
    y = clamp(int(round(box[1])), 0, AI_FRAME_SIZE[1] - 1)
    width = max(0, int(round(box[2])))
    height = max(0, int(round(box[3])))
    width = min(width, AI_FRAME_SIZE[0] - x)
    height = min(height, AI_FRAME_SIZE[1] - y)
    return x, y, width, height


def draw_detections(osd_image, detections, labels, display_size):
    osd_image.clear()
    if not valid_detections(detections):
        return

    for index in range(len(detections[0])):
        x, y, width, height = normalize_box(detections[0][index])
        class_id = int(detections[1][index])
        score = float(detections[2][index])

        display_x = x * display_size[0] // AI_FRAME_SIZE[0]
        display_y = y * display_size[1] // AI_FRAME_SIZE[1]
        display_width = width * display_size[0] // AI_FRAME_SIZE[0]
        display_height = height * display_size[1] // AI_FRAME_SIZE[1]

        osd_image.draw_rectangle(
            display_x,
            display_y,
            display_width,
            display_height,
            color=(0, 255, 0),
            thickness=3,
        )
        osd_image.draw_string_advanced(
            display_x,
            max(0, display_y - 32),
            24,
            labels[class_id] + " " + str(round(score, 2)),
            color=(0, 255, 0),
        )


def send_best_detection(uart, protocol, detections, labels):
    """Send only the highest-confidence detection from the current frame."""
    if not valid_detections(detections):
        return None

    best_index = 0
    best_score = float(detections[2][0])
    for index in range(1, len(detections[2])):
        score = float(detections[2][index])
        if score > best_score:
            best_index = index
            best_score = score

    x, y, width, height = normalize_box(detections[0][best_index])
    if width <= 0 or height <= 0:
        return None

    class_id = int(detections[1][best_index])
    class_name = labels[class_id]
    packet = protocol.get_object_detect_data(
        x, y, width, height, class_name
    )
    uart.send(packet)

    return (
        x + width // 2,
        y + height // 2,
        best_score,
        class_name,
    )


def validate_config(config):
    required_keys = (
        "model_type",
        "img_size",
        "categories",
        "kmodel_path",
        "confidence_threshold",
        "nms_threshold",
        "max_boxes_num",
        "uart_baudrate",
        "coordinate_width",
        "coordinate_height",
    )
    for key in required_keys:
        if key not in config:
            raise ValueError("Missing deploy_config key: " + key)

    if config["model_type"] != EXPECTED_MODEL_TYPE:
        raise ValueError("Unsupported model_type: " + config["model_type"])
    if config["img_size"] != [320, 256]:
        raise ValueError("This model requires img_size [320, 256]")
    if config["categories"] != ["Steel_ball"]:
        raise ValueError("categories must be ['Steel_ball']")
    if (
        config["coordinate_width"] != AI_FRAME_SIZE[0]
        or config["coordinate_height"] != AI_FRAME_SIZE[1]
    ):
        raise ValueError("STM32 coordinates must remain 1280 x 720")


uart = None
pipeline = None
detector = None

os.exitpoint(os.EXITPOINT_ENABLE)

try:
    deploy_config = read_json(DEPLOY_ROOT + "deploy_config.json")
    validate_config(deploy_config)

    model_path = DEPLOY_ROOT + deploy_config["kmodel_path"]
    labels = deploy_config["categories"]

    uart = YbUart(baudrate=deploy_config["uart_baudrate"])
    protocol = YbProtocol()

    pipeline = PipeLine(
        rgb888p_size=AI_FRAME_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pipeline.create()
    display_size = pipeline.get_display_size()

    detector = YoloV8DetectionApp(
        model_path,
        labels,
        deploy_config["img_size"],
        deploy_config["confidence_threshold"],
        deploy_config["nms_threshold"],
        deploy_config["max_boxes_num"],
        AI_FRAME_SIZE,
        debug_mode=0,
    )
    detector.config_preprocess()

    frame_count = 0
    while True:
        os.exitpoint()

        with ScopedTiming("total", 0):
            frame = pipeline.get_frame()
            detections = detector.run(frame)
            draw_detections(
                pipeline.osd_img,
                detections,
                labels,
                display_size,
            )
            target = send_best_detection(
                uart,
                protocol,
                detections,
                labels,
            )
            pipeline.show_image()

            frame_count += 1
            if target is not None and frame_count % 20 == 0:
                center_x, center_y, score, class_name = target
                print(
                    class_name,
                    "center:",
                    center_x,
                    center_y,
                    "error:",
                    center_x - 642,
                    center_y - 369,
                    "score:",
                    round(score, 2),
                )

            gc.collect()

        time.sleep_ms(1)

except KeyboardInterrupt:
    print("User stopped YOLOv8 steel-ball detection.")

except BaseException as error:
    print("YOLOv8 steel-ball detection exception:", error)

finally:
    print("Releasing YOLOv8 detection resources...")

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

    if uart is not None:
        try:
            uart.deinit()
        except BaseException as error:
            print("UART deinit warning:", error)

    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    gc.collect()
    print("Resources released.")

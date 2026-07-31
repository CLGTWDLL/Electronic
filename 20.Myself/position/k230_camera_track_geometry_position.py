# -*- coding: utf-8 -*-
"""
K230 motion-v3 tracker using a physical 3D pipe-axis geometry model.

Required:
  /sdcard/mp_deployment_yolov8/k230_deploy_config.json
  /sdcard/mp_deployment_yolov8/labels.txt
  /sdcard/mp_deployment_yolov8/steel_ball_416_motion_v3_nncase_2.9.0.kmodel
  /sdcard/position/geometry_position_model.json

The detected raw 640x640 box centre is undistorted without OpenCV, converted
to a camera ray, and paired with P(s)=T+s*r by the closest-point solution.
Position is displayed in cm. No legacy PCA/segments map is used.
"""

import gc
import math
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
GEOMETRY_MODEL_PATH = "/sdcard/position/geometry_position_model.json"
DISPLAY_MODE = "lcd"
CAMERA_SIZE = [640, 640]
MODEL_SIZE = [416, 416]

EXPECTED_KMODEL = "steel_ball_416_motion_v3_nncase_2.9.0.kmodel"
EXPECTED_INPUT_SHAPE = [1, 3, 416, 416]
EXPECTED_OUTPUT_SHAPE = [1, 10647, 6]

MAX_DETECTIONS = 10
GC_INTERVAL_FRAMES = 30
PERFORMANCE_LOG_INTERVAL = 120
UNDISTORT_ITERATIONS = 7
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


def load_geometry_model(path):
    with open(path, "r") as model_file:
        data = ujson.load(model_file)
    if data.get("schema_version") != 1:
        raise ValueError("unsupported geometry-position schema")
    if data.get("model_type") != (
        "undistorted_ray_to_3d_pipe_axis_closest_point"
    ):
        raise ValueError("unexpected geometry-position model type")

    camera = data.get("camera", {})
    if camera.get("image_size") != [640, 640]:
        raise ValueError("geometry model must use 640x640 camera data")
    matrix = camera.get("camera_matrix")
    distortion = camera.get("distortion_coefficients")
    if (
        not isinstance(matrix, list)
        or len(matrix) != 3
        or not isinstance(distortion, list)
        or len(distortion) < 5
    ):
        raise ValueError("invalid camera parameters in geometry model")

    fx = float(matrix[0][0])
    fy = float(matrix[1][1])
    cx = float(matrix[0][2])
    cy = float(matrix[1][2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    axis = data.get("pipe_axis_camera_coordinates", {})
    translation = axis.get("T_cm")
    direction = axis.get("r")
    if (
        not isinstance(translation, list)
        or len(translation) != 3
        or not isinstance(direction, list)
        or len(direction) != 3
    ):
        raise ValueError("invalid T/r pipe-axis parameters")
    translation = [float(value) for value in translation]
    direction = [float(value) for value in direction]
    norm = math.sqrt(
        direction[0] * direction[0]
        + direction[1] * direction[1]
        + direction[2] * direction[2]
    )
    if norm < 0.999 or norm > 1.001:
        raise ValueError("pipe direction r must be unit length")

    overall = data.get("overall", {})
    anomalous_positions = overall.get("anomalous_positions_cm", [])
    return {
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "k1": float(distortion[0]),
        "k2": float(distortion[1]),
        "p1": float(distortion[2]),
        "p2": float(distortion[3]),
        "k3": float(distortion[4]),
        "T": translation,
        "r": direction,
        "r_dot_T": (
            direction[0] * translation[0]
            + direction[1] * translation[1]
            + direction[2] * translation[2]
        ),
        "model_accepted": bool(
            overall.get("fixed_line_model_accepted", False)
        ),
        "anomalous_positions": anomalous_positions,
        "calibration_rmse_cm": float(
            overall.get("position_rmse_cm", 0.0)
        ),
        "calibration_max_error_cm": float(
            overall.get("position_max_abs_error_cm", 0.0)
        ),
    }


def undistort_pixel_to_unit_ray(u, v, geometry):
    distorted_x = (u - geometry["cx"]) / geometry["fx"]
    distorted_y = (v - geometry["cy"]) / geometry["fy"]
    x = distorted_x
    y = distorted_y

    # Iterative inverse Brown-Conrady distortion, matching
    # cv2.undistortPoints used by the desktop model builder.
    for _ in range(UNDISTORT_ITERATIONS):
        radius2 = x * x + y * y
        radius4 = radius2 * radius2
        radius6 = radius4 * radius2
        radial = (
            1.0
            + geometry["k1"] * radius2
            + geometry["k2"] * radius4
            + geometry["k3"] * radius6
        )
        if abs(radial) < 1e-9:
            raise ValueError("invalid radial distortion inversion")
        delta_x = (
            2.0 * geometry["p1"] * x * y
            + geometry["p2"] * (radius2 + 2.0 * x * x)
        )
        delta_y = (
            geometry["p1"] * (radius2 + 2.0 * y * y)
            + 2.0 * geometry["p2"] * x * y
        )
        x = (distorted_x - delta_x) / radial
        y = (distorted_y - delta_y) / radial

    norm = math.sqrt(x * x + y * y + 1.0)
    return [x / norm, y / norm, 1.0 / norm]


def ray_to_pipe_position(ray, geometry):
    direction = geometry["r"]
    translation = geometry["T"]
    ray_dot_direction = (
        ray[0] * direction[0]
        + ray[1] * direction[1]
        + ray[2] * direction[2]
    )
    ray_dot_translation = (
        ray[0] * translation[0]
        + ray[1] * translation[1]
        + ray[2] * translation[2]
    )
    denominator = 1.0 - ray_dot_direction * ray_dot_direction
    if denominator <= 1e-8:
        return None

    depth_cm = (
        ray_dot_translation
        - ray_dot_direction * geometry["r_dot_T"]
    ) / denominator
    position_cm = (
        ray_dot_direction * depth_cm - geometry["r_dot_T"]
    )
    if depth_cm <= 0.0:
        return None

    ray_x = depth_cm * ray[0]
    ray_y = depth_cm * ray[1]
    ray_z = depth_cm * ray[2]
    axis_x = translation[0] + position_cm * direction[0]
    axis_y = translation[1] + position_cm * direction[1]
    axis_z = translation[2] + position_cm * direction[2]
    closest_distance_cm = math.sqrt(
        (ray_x - axis_x) * (ray_x - axis_x)
        + (ray_y - axis_y) * (ray_y - axis_y)
        + (ray_z - axis_z) * (ray_z - axis_z)
    )
    return position_cm, closest_distance_cm, depth_cm


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
            if intersection_over_union(candidate, previous) > iou_threshold:
                keep = False
                break
        if keep:
            selected.append(candidate)
            if len(selected) >= limit:
                break
    return selected


class GeometryPositionDetector(AIBase):
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
        print("Preprocess: 640x640 -> 416x416, RGB NCHW float /255")

    def preprocess(self, input_np):
        ai2d_output = self.ai2d.run(input_np)
        if isinstance(ai2d_output, (list, tuple)):
            ai2d_output = ai2d_output[0]
        if hasattr(ai2d_output, "to_numpy"):
            resized_input = ai2d_output.to_numpy()
        else:
            resized_input = ai2d_output
        model_input = resized_input / 255.0

        if self.first_input_check:
            print("First input shape:", model_input.shape)
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
                    "Expected input ending in (3,416,416), got "
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
                print("Fast filter unavailable:", filter_error)

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
                "detections:",
                len(detections),
            )
        return detections

    def draw_result(self, osd_image, detections, fps, geometry):
        osd_image.clear()
        if not detections:
            osd_image.draw_string_advanced(
                8, 8, 28, "NO BALL", color=(255, 0, 0)
            )
            osd_image.draw_string_advanced(
                8, 42, 20, "FPS %.1f" % fps, color=(0, 255, 0)
            )
            return

        best = detections[0]
        class_id, score, x1, y1, x2, y2 = best
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        ray = undistort_pixel_to_unit_ray(
            center_x,
            center_y,
            geometry,
        )
        geometry_result = ray_to_pipe_position(ray, geometry)

        display_x = int(x1 * self.display_size[0] / 640)
        display_y = int(y1 * self.display_size[1] / 640)
        display_w = int((x2 - x1) * self.display_size[0] / 640)
        display_h = int((y2 - y1) * self.display_size[1] / 640)
        display_cx = int(center_x * self.display_size[0] / 640)
        display_cy = int(center_y * self.display_size[1] / 640)
        osd_image.draw_rectangle(
            display_x,
            display_y,
            display_w,
            display_h,
            color=self.colors[class_id],
            thickness=3,
        )
        osd_image.draw_line(
            display_cx - 8,
            display_cy,
            display_cx + 8,
            display_cy,
            color=(255, 255, 0),
            thickness=2,
        )
        osd_image.draw_line(
            display_cx,
            display_cy - 8,
            display_cx,
            display_cy + 8,
            color=(255, 255, 0),
            thickness=2,
        )
        if geometry_result is None:
            osd_image.draw_string_advanced(
                8,
                8,
                25,
                "GEOMETRY INVALID",
                color=(255, 0, 0),
            )
        else:
            position_cm, _, _ = geometry_result
            osd_image.draw_string_advanced(
                8,
                8,
                27,
                "%+.3f cm" % position_cm,
                color=(0, 255, 0),
            )

        osd_image.draw_string_advanced(
            8,
            42,
            18,
            "FPS %.1f" % fps,
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
    print("Geometry position tracker starting")
    with open(CONFIG_PATH, "r") as config_file:
        config = ujson.load(config_file)
    labels = load_labels(DEPLOY_ROOT + config["labels_path"])
    validate_config(config, labels)
    geometry = load_geometry_model(GEOMETRY_MODEL_PATH)
    print("Geometry model:", GEOMETRY_MODEL_PATH)
    print("T_cm:", geometry["T"])
    print("r:", geometry["r"])
    print(
        "Calibration position RMSE/max cm:",
        geometry["calibration_rmse_cm"],
        geometry["calibration_max_error_cm"],
    )
    print("Anomalous positions:", geometry["anomalous_positions"])

    pipeline = PipeLine(
        rgb888p_size=CAMERA_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pipeline.create()
    display_size = pipeline.get_display_size()
    detector = GeometryPositionDetector(
        DEPLOY_ROOT + config["kmodel_path"],
        labels,
        config["postprocess"]["confidence_threshold"],
        config["postprocess"]["nms_iou_threshold"],
        display_size,
        DEBUG_MODE,
    )
    detector.config_preprocess()
    print("Camera, detector, and geometry model ready")

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
                geometry,
            )
            pipeline.show_image()
            frame_count += 1
            if frame_count % PERFORMANCE_LOG_INTERVAL == 0:
                print("Display FPS:", round(fps, 2))
            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()
        time.sleep_ms(1)

except KeyboardInterrupt:
    print("User stopped geometry position tracker.")
except BaseException as error:
    error_text = str(error)
    if error_text == "IDE interrupt":
        print("IDE stopped geometry position tracker.")
    else:
        print("Geometry position tracker exception:", error)
        try:
            with open(
                "/sdcard/geometry_position_error.txt",
                "w",
            ) as error_file:
                error_file.write(
                    "Geometry position tracker exception:\n"
                )
                error_file.write(error_text)
                error_file.write("\n")
        except BaseException:
            pass
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
    print("Geometry position tracker resources released.")

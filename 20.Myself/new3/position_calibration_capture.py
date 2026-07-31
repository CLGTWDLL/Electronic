# -*- coding: utf-8 -*-
"""
K230 motion-v3 position-calibration data capture.

Place the ball at -10 cm and press USER to collect 100 valid detections. After
each completed capture, move the ball through -8, -6, ..., 10 cm and press
USER again. No images are saved. Each CSV row contains the restored ball
center in the detector's original 640x640 camera coordinates, confidence,
and the automatically selected real_cm value.
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
from ybUtils.YbKey import YbKey


DEPLOY_ROOT = "/sdcard/mp_deployment_yolov8/"
CONFIG_PATH = DEPLOY_ROOT + "k230_deploy_config.json"
DISPLAY_MODE = "lcd"
CAMERA_SIZE = [640, 640]
MODEL_SIZE = [416, 416]

EXPECTED_KMODEL = "steel_ball_416_motion_v3_nncase_2.9.0.kmodel"
EXPECTED_INPUT_SHAPE = [1, 3, 416, 416]
EXPECTED_OUTPUT_SHAPE = [1, 10647, 6]

# One USER-key capture per position, in this exact order.
POSITION_SEQUENCE_CM = (-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10)

SAMPLES_PER_CAPTURE = 100
SAVE_ROOT = "/sdcard/position_calibration"
KEY_DEBOUNCE_MS = 250
MAX_DETECTIONS = 10
GC_INTERVAL_FRAMES = 30
PERFORMANCE_LOG_INTERVAL = 120
DEBUG_MODE = 0


def align_up(value, alignment):
    return (value + alignment - 1) // alignment * alignment


def path_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def ensure_directory(path):
    path = path.rstrip("/")
    if not path or path_exists(path):
        return
    parent_end = path.rfind("/")
    if parent_end > 0:
        ensure_directory(path[:parent_end])
    os.mkdir(path)


def next_csv_path():
    ensure_directory(SAVE_ROOT)
    index = 1
    while True:
        path = SAVE_ROOT + "/position_%03d.csv" % index
        if not path_exists(path):
            return path
        index += 1


def sync_filesystem():
    if hasattr(os, "sync"):
        os.sync()


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
            if intersection_over_union(candidate, previous) > iou_threshold:
                keep = False
                break
        if keep:
            selected.append(candidate)
            if len(selected) >= limit:
                break
    return selected


try:
    from ybUtils.YbBuzzer import YbBuzzer

    buzzer = YbBuzzer()
except BaseException:
    buzzer = None


def beep(frequency=2200, duration=0.08):
    if buzzer is not None:
        try:
            buzzer.on(frequency, 40, duration)
        except BaseException:
            pass


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

        # Identical coordinate restoration to k230_camera_track_motion_v3.py.
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
                "detections:",
                len(detections),
            )

        return detections

    def draw_result(
        self,
        osd_image,
        detections,
        fps,
        capturing,
        sample_count,
        current_real_cm,
        sequence_complete,
    ):
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
            osd_image.draw_rectangle(
                display_x,
                display_y,
                display_width,
                display_height,
                color=self.colors[class_id],
                thickness=3,
            )

        if capturing:
            status = "CAP %gcm %d/%d" % (
                current_real_cm,
                sample_count,
                SAMPLES_PER_CAPTURE,
            )
        elif sequence_complete:
            status = "All positions complete"
        else:
            status = "Next %gcm USER=start" % current_real_cm
        if detections:
            status += " Ball %.2f" % detections[0][1]
        else:
            status += " No ball"
        status += " FPS %.1f" % fps
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
csv_file = None
capturing = False
sample_count = 0
active_csv_path = None
position_index = 0
active_real_cm = None

os.exitpoint(os.EXITPOINT_ENABLE)

try:
    print("Steel-ball position calibration capture starting")
    print("Position sequence (cm):", POSITION_SEQUENCE_CM)
    print("Config:", CONFIG_PATH)

    with open(CONFIG_PATH, "r") as config_file:
        config = ujson.load(config_file)
    labels = load_labels(DEPLOY_ROOT + config["labels_path"])
    validate_config(config, labels)

    ensure_directory(SAVE_ROOT)
    key = YbKey()
    pipeline = PipeLine(
        rgb888p_size=CAMERA_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pipeline.create()
    display_size = pipeline.get_display_size()

    detector = MotionV3Detector(
        DEPLOY_ROOT + config["kmodel_path"],
        labels,
        config["postprocess"]["confidence_threshold"],
        config["postprocess"]["nms_iou_threshold"],
        display_size,
        DEBUG_MODE,
    )
    detector.config_preprocess()
    print(
        "Ready. Place the ball at",
        POSITION_SEQUENCE_CM[0],
        "cm and press USER.",
    )
    print("CSV output root:", SAVE_ROOT)
    beep(2500, 0.15)

    frame_count = 0
    last_tick = time.ticks_ms()
    fps = 0.0
    previous_pressed = key.is_pressed()
    last_key_event_ms = time.ticks_ms() - KEY_DEBOUNCE_MS

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

            pressed = key.is_pressed()
            if (
                pressed
                and not previous_pressed
                and time.ticks_diff(current_tick, last_key_event_ms)
                >= KEY_DEBOUNCE_MS
            ):
                last_key_event_ms = current_tick
                if capturing:
                    print(
                        "Capture already active:",
                        sample_count,
                        "/",
                        SAMPLES_PER_CAPTURE,
                    )
                    beep(1400, 0.10)
                elif position_index >= len(POSITION_SEQUENCE_CM):
                    print("All calibration positions are complete.")
                    beep(1200, 0.18)
                else:
                    active_real_cm = POSITION_SEQUENCE_CM[
                        position_index
                    ]
                    active_csv_path = next_csv_path()
                    csv_file = open(active_csv_path, "w")
                    csv_file.write(
                        "sample,x,y,confidence,real_cm\n"
                    )
                    sample_count = 0
                    capturing = True
                    print(
                        "Capture started:",
                        active_csv_path,
                        "REAL_CM:",
                        active_real_cm,
                    )
                    beep(2800, 0.12)
            previous_pressed = pressed

            # Use the highest-confidence detection from each valid frame.
            if capturing and detections:
                best = detections[0]
                score = best[1]
                center_x = (best[2] + best[4]) / 2.0
                center_y = (best[3] + best[5]) / 2.0
                sample_count += 1
                csv_file.write(
                    "%d,%.6f,%.6f,%.8f,%.6f\n"
                    % (
                        sample_count,
                        center_x,
                        center_y,
                        score,
                        active_real_cm,
                    )
                )

                if sample_count >= SAMPLES_PER_CAPTURE:
                    csv_file.close()
                    csv_file = None
                    sync_filesystem()
                    capturing = False
                    position_index += 1
                    print(
                        "Capture completed:",
                        active_csv_path,
                        SAMPLES_PER_CAPTURE,
                        "samples",
                    )
                    if position_index < len(POSITION_SEQUENCE_CM):
                        print(
                            "Move ball to",
                            POSITION_SEQUENCE_CM[position_index],
                            "cm, then press USER.",
                        )
                    else:
                        print("All calibration positions are complete.")
                    beep(1800, 0.25)

            detector.draw_result(
                pipeline.osd_img,
                detections,
                fps,
                capturing,
                sample_count,
                (
                    active_real_cm
                    if capturing
                    else (
                        POSITION_SEQUENCE_CM[position_index]
                        if position_index
                        < len(POSITION_SEQUENCE_CM)
                        else POSITION_SEQUENCE_CM[-1]
                    )
                ),
                position_index >= len(POSITION_SEQUENCE_CM),
            )
            pipeline.show_image()

            frame_count += 1
            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()

        time.sleep_ms(1)

except KeyboardInterrupt:
    print("User stopped position calibration capture.")
except BaseException as error:
    error_text = str(error)
    if error_text == "IDE interrupt":
        print("IDE stopped position calibration capture.")
    else:
        print("Position calibration capture exception:", error)
        try:
            with open(
                "/sdcard/position_calibration_error.txt",
                "w",
            ) as error_file:
                error_file.write(
                    "Position calibration capture exception:\n"
                )
                error_file.write(error_text)
                error_file.write("\n")
        except BaseException:
            pass
finally:
    if csv_file is not None:
        try:
            csv_file.close()
            print(
                "Partial CSV retained:",
                active_csv_path,
                sample_count,
                "samples",
            )
        except BaseException as error:
            print("CSV close warning:", error)
    sync_filesystem()

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
    print("Position calibration capture resources released.")

# -*- coding: utf-8 -*-
"""
K230 motion dataset capture for the 416x416 steel-ball model.

The saved source images are 640x640, matching the square camera source used by
k230_camera_detect_416.py. Press the programmable user key to start a burst;
press it again to stop. Every burst gets a separate session directory.

Do not use the reset button near the USB connector. The programmable key is
GPIO61 on the Yahboom K230 board.
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import (
    CAM_CHN_ID_0,
    CAM_CHN_ID_1,
    PIXEL_FORMAT_YUV_SEMIPLANAR_420,
    Sensor,
)
from ybUtils.YbKey import YbKey


CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 640
CAPTURE_CHANNEL = CAM_CHN_ID_1

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

# 125 ms is a target of 8 saved frames per second. JPEG writing speed may
# lower the real rate; the script prints the measured rate while recording.
CAPTURE_INTERVAL_MS = 125
MAX_FRAMES_PER_SESSION = 600
STARTUP_SETTLE_MS = 3000
KEY_POLL_MS = 10
KEY_DEBOUNCE_MS = 250
GC_INTERVAL_FRAMES = 30

SAVE_ROOT = "/sdcard/motion_dataset_640"


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


def create_session_directory():
    ensure_directory(SAVE_ROOT)
    session_index = 1
    while True:
        session_path = SAVE_ROOT + "/session_%03d" % session_index
        if not path_exists(session_path):
            os.mkdir(session_path)
            return session_path
        session_index += 1


def write_session_info(session_path):
    info_path = session_path + "/session_info.txt"
    with open(info_path, "w") as info_file:
        info_file.write("capture_width=%d\n" % CAPTURE_WIDTH)
        info_file.write("capture_height=%d\n" % CAPTURE_HEIGHT)
        info_file.write(
            "target_interval_ms=%d\n" % CAPTURE_INTERVAL_MS
        )
        info_file.write("pixel_format=RGB565_JPEG\n")
        info_file.write("camera=gc2093_csi2\n")


def sync_filesystem():
    if hasattr(os, "sync"):
        os.sync()


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


sensor = None
media_initialized = False
display_initialized = False
recording = False
session_directory = None
session_frame_count = 0
total_saved_count = 0

os.exitpoint(os.EXITPOINT_ENABLE)

try:
    key = YbKey()
    sensor = Sensor()
    sensor.reset()

    # LCD preview. The capture channel below remains a separate 640x640 image.
    sensor.set_framesize(
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        chn=CAM_CHN_ID_0,
    )
    sensor.set_pixformat(
        PIXEL_FORMAT_YUV_SEMIPLANAR_420,
        chn=CAM_CHN_ID_0,
    )
    preview_bind_info = sensor.bind_info(
        x=0,
        y=0,
        chn=CAM_CHN_ID_0,
    )
    Display.bind_layer(
        **preview_bind_info,
        layer=Display.LAYER_VIDEO1
    )
    Display.init(
        Display.ST7701,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=True,
    )
    display_initialized = True

    # RGB565 is used because this firmware can encode it directly as JPEG.
    sensor.set_framesize(
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        chn=CAPTURE_CHANNEL,
    )
    sensor.set_pixformat(
        Sensor.RGB565,
        chn=CAPTURE_CHANNEL,
    )

    MediaManager.init()
    media_initialized = True
    sensor.run()

    print("Camera warming up; waiting for auto exposure/white balance...")
    settle_start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), settle_start) < STARTUP_SETTLE_MS:
        os.exitpoint()
        sensor.snapshot(chn=CAPTURE_CHANNEL)
        time.sleep_ms(20)

    previous_pressed = key.is_pressed()
    last_key_event_ms = time.ticks_ms() - KEY_DEBOUNCE_MS
    last_capture_ms = time.ticks_ms()
    session_start_ms = 0

    print("Motion capture ready.")
    print("Press USER key to START; press again to STOP.")
    print("Output root:", SAVE_ROOT)
    beep(2500, 0.15)

    while True:
        os.exitpoint()
        now_ms = time.ticks_ms()
        pressed = key.is_pressed()

        if (
            pressed
            and not previous_pressed
            and time.ticks_diff(now_ms, last_key_event_ms)
            >= KEY_DEBOUNCE_MS
        ):
            last_key_event_ms = now_ms

            if recording:
                recording = False
                sync_filesystem()
                elapsed_ms = max(
                    1,
                    time.ticks_diff(now_ms, session_start_ms),
                )
                average_fps = (
                    session_frame_count * 1000.0 / elapsed_ms
                )
                print(
                    "STOP:",
                    session_directory,
                    "frames:",
                    session_frame_count,
                    "average FPS:",
                    round(average_fps, 2),
                )
                beep(1500, 0.18)
            else:
                session_directory = create_session_directory()
                write_session_info(session_directory)
                session_frame_count = 0
                session_start_ms = now_ms
                last_capture_ms = now_ms - CAPTURE_INTERVAL_MS
                recording = True
                print("START:", session_directory)
                beep(2800, 0.12)

        previous_pressed = pressed

        if recording:
            if (
                time.ticks_diff(now_ms, last_capture_ms)
                >= CAPTURE_INTERVAL_MS
            ):
                image = sensor.snapshot(chn=CAPTURE_CHANNEL)
                image_path = (
                    session_directory
                    + "/frame_%06d.jpg" % (session_frame_count + 1)
                )
                image.save(image_path)
                image = None

                session_frame_count += 1
                total_saved_count += 1
                last_capture_ms = now_ms

                if session_frame_count % 20 == 0:
                    elapsed_ms = max(
                        1,
                        time.ticks_diff(now_ms, session_start_ms),
                    )
                    measured_fps = (
                        session_frame_count * 1000.0 / elapsed_ms
                    )
                    print(
                        "Recording:",
                        session_frame_count,
                        "frames, measured FPS:",
                        round(measured_fps, 2),
                    )

                if session_frame_count % GC_INTERVAL_FRAMES == 0:
                    gc.collect()

                if session_frame_count >= MAX_FRAMES_PER_SESSION:
                    recording = False
                    sync_filesystem()
                    print(
                        "Session limit reached:",
                        session_directory,
                        session_frame_count,
                    )
                    beep(1500, 0.25)
        else:
            time.sleep_ms(KEY_POLL_MS)

except KeyboardInterrupt:
    print("User stopped motion dataset capture.")

except BaseException as error:
    error_text = str(error)
    if error_text == "IDE interrupt":
        print("IDE stopped motion dataset capture.")
    else:
        print("Motion dataset capture exception:", error)
        try:
            with open("/sdcard/capture_error.txt", "w") as error_file:
                error_file.write(
                    "Motion dataset capture exception:\n"
                )
                error_file.write(error_text)
                error_file.write("\n")
        except BaseException:
            pass

finally:
    print(
        "Stopping capture. Total images saved:",
        total_saved_count,
    )
    sync_filesystem()

    if sensor is not None:
        try:
            sensor.stop()
        except BaseException as error:
            print("Sensor stop warning:", error)

    if display_initialized:
        try:
            Display.deinit()
        except BaseException as error:
            print("Display deinit warning:", error)

    if media_initialized:
        try:
            MediaManager.deinit()
        except BaseException as error:
            print("Media deinit warning:", error)

    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    gc.collect()
    print("Motion dataset capture resources released.")

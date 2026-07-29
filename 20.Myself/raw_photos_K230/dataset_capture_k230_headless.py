# -*- coding: utf-8 -*-
"""
Headless dataset capture for CanMV K230.

The script waits for the camera to stabilize, then saves one 1280 x 720 JPEG
to the TF card at a fixed interval. It is intended to run from a USB power
bank without a computer or display.

Saved files:
    /sdcard/dataset_capture/session_001/image_000001.jpg
"""

import gc
import os
import time

from media.media import *
from media.sensor import *


IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
CAPTURE_CHANNEL = CAM_CHN_ID_1
CAPTURE_INTERVAL_MS = 1500
CAMERA_WARMUP_MS = 3000
MAX_IMAGES = 1200
DATASET_ROOT = "/sdcard/dataset_capture"


def path_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def ensure_directory(path):
    if not path_exists(path):
        os.mkdir(path)


def create_session_directory():
    ensure_directory(DATASET_ROOT)

    session_index = 1
    while True:
        session_path = DATASET_ROOT + "/session_%03d" % session_index
        if not path_exists(session_path):
            os.mkdir(session_path)
            return session_path
        session_index += 1


def flush_filesystem():
    if hasattr(os, "sync"):
        os.sync()


sensor = None
session_directory = None
saved_count = 0

os.exitpoint(os.EXITPOINT_ENABLE)

try:
    session_directory = create_session_directory()
    print("Dataset session:", session_directory)

    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
        chn=CAPTURE_CHANNEL,
    )
    sensor.set_pixformat(Sensor.RGB565, chn=CAPTURE_CHANNEL)

    MediaManager.init()
    sensor.run()

    print("Camera warmup...")
    time.sleep_ms(CAMERA_WARMUP_MS)
    print("Automatic capture started.")

    last_capture_ms = time.ticks_ms() - CAPTURE_INTERVAL_MS

    while saved_count < MAX_IMAGES:
        os.exitpoint()

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_capture_ms) >= CAPTURE_INTERVAL_MS:
            image = sensor.snapshot(chn=CAPTURE_CHANNEL)
            saved_count += 1
            image_path = (
                session_directory
                + "/image_%06d.jpg" % saved_count
            )
            image.save(image_path)
            flush_filesystem()
            print("Saved", saved_count, "of", MAX_IMAGES, image_path)

            image = None
            gc.collect()
            last_capture_ms = now_ms

        time.sleep_ms(10)

    print("Capture limit reached:", saved_count)

except KeyboardInterrupt:
    print("User stopped dataset capture.")

except BaseException as error:
    print("Dataset capture exception:", error)

finally:
    print("Stopping dataset capture. Saved:", saved_count)

    if isinstance(sensor, Sensor):
        try:
            sensor.stop()
        except BaseException as error:
            print("Sensor stop warning:", error)

    flush_filesystem()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)

    try:
        MediaManager.deinit()
    except BaseException as error:
        print("Media deinit warning:", error)

    gc.collect()
    print("Dataset capture resources released.")

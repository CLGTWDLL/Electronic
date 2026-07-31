# -*- coding: utf-8 -*-
"""
K230 camera-calibration image capture for the 640x640 detection view.

The programmable USER key saves exactly one JPEG per press. Images are saved
as /sdcard/camera_calibration_640/calib_001.jpg through calib_030.jpg.
This script only captures images; it does not detect corners or calibrate.

The saved RGB565 channel uses the same 640x640 size, sensor orientation, and
effective field of view as the formal detector. RGB565 is used because this
CanMV firmware can encode it directly as JPEG. No mirror, vertical flip, crop,
or windowing is applied.
GPIO61 is the programmable USER key on the Yahboom K230 board.
"""

import gc
import os
import time

import image
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

STARTUP_SETTLE_MS = 3000
KEY_POLL_MS = 10
KEY_DEBOUNCE_MS = 250
MAX_IMAGES = 30

SAVE_ROOT = "/sdcard/camera_calibration_640"


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


def image_path(index):
    return SAVE_ROOT + "/calib_%03d.jpg" % index


def scan_saved_images():
    saved = 0
    next_index = None
    for index in range(1, MAX_IMAGES + 1):
        if path_exists(image_path(index)):
            saved += 1
        elif next_index is None:
            next_index = index
    return saved, next_index


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


def draw_status(osd_image, saved_count, message):
    osd_image.clear()
    osd_image.draw_string_advanced(
        8,
        8,
        24,
        "Calib: %d/%d" % (saved_count, MAX_IMAGES),
        color=(255, 0, 255, 0),
    )
    osd_image.draw_string_advanced(
        8,
        38,
        20,
        message,
        color=(255, 255, 255, 0),
    )
    Display.show_image(osd_image, 0, 0, Display.LAYER_OSD2)


sensor = None
osd_image = None
media_initialized = False
display_initialized = False
saved_count = 0

os.exitpoint(os.EXITPOINT_ENABLE)

try:
    ensure_directory(SAVE_ROOT)
    saved_count, next_index = scan_saved_images()

    key = YbKey()
    sensor = Sensor()
    sensor.reset()

    # Preserve the existing 640x480 LCD/IDE preview.
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

    # Use the firmware's JPEG-compatible RGB565 channel. Its 640x640 size,
    # orientation, and uncropped sensor view match the formal detector.
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
    osd_image = image.Image(
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        image.ARGB8888,
    )
    sensor.run()

    draw_status(osd_image, saved_count, "Warming up...")
    print("Camera warming up; waiting for auto exposure/white balance...")
    settle_start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), settle_start) < STARTUP_SETTLE_MS:
        os.exitpoint()
        sensor.snapshot(chn=CAPTURE_CHANNEL)
        time.sleep_ms(20)

    previous_pressed = key.is_pressed()
    last_key_event_ms = time.ticks_ms() - KEY_DEBOUNCE_MS

    if next_index is None:
        status_message = "Limit reached"
    else:
        status_message = "Press USER to save"
    draw_status(osd_image, saved_count, status_message)

    print("Camera calibration capture ready.")
    print("Press USER once to save one 640x640 JPEG.")
    print("Output:", SAVE_ROOT)
    print("Saved:", saved_count, "/", MAX_IMAGES)
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

            if next_index is None:
                print("Calibration image limit reached:", MAX_IMAGES)
                draw_status(osd_image, saved_count, "Limit reached")
                beep(1200, 0.18)
            else:
                saved_index = next_index
                captured_image = sensor.snapshot(chn=CAPTURE_CHANNEL)
                target_path = image_path(saved_index)
                captured_image.save(target_path)
                captured_image = None
                sync_filesystem()

                saved_count += 1
                print(
                    "Saved:",
                    target_path,
                    "(%d/%d)" % (saved_count, MAX_IMAGES),
                )
                beep(2800, 0.10)
                gc.collect()

                saved_count, next_index = scan_saved_images()
                if next_index is None:
                    draw_status(
                        osd_image,
                        saved_count,
                        "Limit reached",
                    )
                    beep(1600, 0.20)
                else:
                    draw_status(
                        osd_image,
                        saved_count,
                        "Saved calib_%03d.jpg" % saved_index,
                    )

        previous_pressed = pressed
        time.sleep_ms(KEY_POLL_MS)

except KeyboardInterrupt:
    print("User stopped camera calibration capture.")

except BaseException as error:
    error_text = str(error)
    if error_text == "IDE interrupt":
        print("IDE stopped camera calibration capture.")
    else:
        print("Camera calibration capture exception:", error)
        try:
            with open(
                "/sdcard/camera_calibration_capture_error.txt",
                "w",
            ) as error_file:
                error_file.write(
                    "Camera calibration capture exception:\n"
                )
                error_file.write(error_text)
                error_file.write("\n")
        except BaseException:
            pass

finally:
    print(
        "Stopping capture. Calibration images present:",
        saved_count,
    )
    sync_filesystem()

    if sensor is not None:
        try:
            sensor.stop()
        except BaseException as error:
            print("Sensor stop warning:", error)

    osd_image = None

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
    print("Camera calibration capture resources released.")

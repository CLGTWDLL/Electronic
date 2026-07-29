# -*- coding: utf-8 -*-
"""
Offline 1280 x 720 JPEG dataset capture for the Yahboom K230.

Operation:
  1. Save this script to the board as /sdcard/main.py.
  2. Power the K230 from a 5 V power bank.
  3. Wait for the startup beep.
  4. Press the user Key once for each photo.
  5. A short beep confirms that the JPEG was saved.

Photos are stored under:
    /sdcard/snapshot_1280x720/session_xxx/000001.jpg

The Key is the programmable button on GPIO61. The button nearest the USB
connector is RST and must not be used for taking photos.
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


CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
SAVE_ROOT = "/sdcard/snapshot_1280x720"
STARTUP_SETTLE_MS = 3000
KEY_POLL_MS = 20


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


try:
    from ybUtils.YbBuzzer import YbBuzzer
    buzzer = YbBuzzer()
except BaseException:
    buzzer = None


def beep(frequency=2000, duration=0.08):
    if buzzer is not None:
        try:
            buzzer.on(frequency, 40, duration)
        except BaseException:
            pass


sensor = None
media_initialized = False
display_initialized = False

os.exitpoint(os.EXITPOINT_ENABLE)

try:
    session_directory = create_session_directory()
    print("Dataset directory:", session_directory)

    key = YbKey()
    sensor = Sensor()
    sensor.reset()

    # Channel 0 is bound directly to the LCD for a smooth live preview.
    # Keeping preview and capture on separate channels avoids resizing the
    # 1280 x 720 photo buffer or repeatedly copying it to the display.
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

    # Channel 1 remains the full-resolution still-photo channel.
    sensor.set_framesize(
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        chn=CAM_CHN_ID_1,
    )
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

    MediaManager.init()
    media_initialized = True
    sensor.run()

    # Let auto exposure and white balance settle before the first photo.
    settle_start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), settle_start) < STARTUP_SETTLE_MS:
        os.exitpoint()
        sensor.snapshot(chn=CAM_CHN_ID_1)
        time.sleep_ms(20)

    photo_index = 1
    previous_pressed = key.is_pressed()
    print("Ready. Press the user Key to capture 1280x720 JPEG images.")
    beep(2400, 0.15)

    while True:
        os.exitpoint()
        pressed = key.is_pressed()

        if pressed and not previous_pressed:
            image = sensor.snapshot(chn=CAM_CHN_ID_1)
            image_path = (
                session_directory + "/%06d.jpg" % photo_index
            )
            image.save(image_path)
            print(
                "Saved:",
                image_path,
                CAPTURE_WIDTH,
                "x",
                CAPTURE_HEIGHT,
            )
            photo_index += 1
            beep()
            gc.collect()

        previous_pressed = pressed
        time.sleep_ms(KEY_POLL_MS)

except KeyboardInterrupt:
    print("User stopped dataset capture.")

except BaseException as error:
    print("Dataset capture exception:", error)

finally:
    print("Releasing dataset capture resources...")

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
    print("Dataset capture resources released.")

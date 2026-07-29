from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Optional, Sequence, Union


NumberTriplet = Union[Sequence[float], Mapping[str, float]]
FrameProvider = Callable[[], Any]
StreamFactory = Callable[[], Iterable[bytes]]


def _clip(value: Any, lower: float, upper: float, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return max(lower, min(upper, number))


def _bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


@dataclass
class DriveCommand:
    """Normalized car drive command."""

    mode: str = "joystick"
    x: float = 0.0
    y: float = 0.0
    active: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "x": self.x,
            "y": self.y,
            "active": self.active,
        }


@dataclass
class ArmCommand:
    """Current robot arm button command."""

    mode: str = "axis"
    step: float = 1.0
    axis1_negative: bool = False
    axis1_positive: bool = False
    axis2_negative: bool = False
    axis2_positive: bool = False
    axis3_negative: bool = False
    axis3_positive: bool = False
    x_negative: bool = False
    x_positive: bool = False
    y_negative: bool = False
    y_positive: bool = False
    z_negative: bool = False
    z_positive: bool = False
    gripper: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "step": self.step,
            "buttons": {
                "axis": {
                    "axis1": {
                        "negative": self.axis1_negative,
                        "positive": self.axis1_positive,
                    },
                    "axis2": {
                        "negative": self.axis2_negative,
                        "positive": self.axis2_positive,
                    },
                    "axis3": {
                        "negative": self.axis3_negative,
                        "positive": self.axis3_positive,
                    },
                },
                "translation": {
                    "x": {
                        "negative": self.x_negative,
                        "positive": self.x_positive,
                    },
                    "y": {
                        "negative": self.y_negative,
                        "positive": self.y_positive,
                    },
                    "z": {
                        "negative": self.z_negative,
                        "positive": self.z_positive,
                    },
                },
            },
            "gripper": self.gripper,
        }


@dataclass
class RobotDisplay:
    """Physical robot state shown on the page."""

    axis1_angle: float = 0.0
    axis2_angle: float = 0.0
    axis3_angle: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "angles": {
                "axis1": self.axis1_angle,
                "axis2": self.axis2_angle,
                "axis3": self.axis3_angle,
            },
            "position": {
                "x": self.x,
                "y": self.y,
                "z": self.z,
            },
        }


@dataclass
class GyroData:
    """Latest phone motion and orientation values reported by the page."""

    supported: bool = False
    active: bool = False
    permission: str = "unknown"
    alpha: float = 0.0
    beta: float = 0.0
    gamma: float = 0.0
    absolute: bool = False
    acceleration_x: float = 0.0
    acceleration_y: float = 0.0
    acceleration_z: float = 0.0
    gravity_x: float = 0.0
    gravity_y: float = 0.0
    gravity_z: float = 0.0
    rotation_alpha: float = 0.0
    rotation_beta: float = 0.0
    rotation_gamma: float = 0.0
    interval: float = 0.0
    updated_at: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "supported": self.supported,
            "active": self.active,
            "permission": self.permission,
            "orientation": {
                "alpha": self.alpha,
                "beta": self.beta,
                "gamma": self.gamma,
                "absolute": self.absolute,
            },
            "acceleration": {
                "x": self.acceleration_x,
                "y": self.acceleration_y,
                "z": self.acceleration_z,
            },
            "acceleration_including_gravity": {
                "x": self.gravity_x,
                "y": self.gravity_y,
                "z": self.gravity_z,
            },
            "rotation_rate": {
                "alpha": self.rotation_alpha,
                "beta": self.rotation_beta,
                "gamma": self.rotation_gamma,
            },
            "interval": self.interval,
            "updated_at": self.updated_at,
        }


class OpenCVVideoStream:
    """MJPEG frame source backed by OpenCV."""

    def __init__(
        self,
        source: Any = 0,
        size: tuple[int, int] = (640, 480),
        jpeg_quality: int = 82,
        rotate_180: bool = False,
    ) -> None:
        self.source = source
        self.size = size
        self.jpeg_quality = int(_clip(jpeg_quality, 1, 100, 82))
        self.rotate_180 = bool(rotate_180)
        self._capture: Any = None
        self._jpeg_provider: Optional[Callable[[], bytes]] = None
        self._stream_factory: Optional[StreamFactory] = None
        self._preview_frame: Any = None
        self._preview_updated_at = 0.0
        self._lock = threading.RLock()

    def set_source(self, source: Any) -> None:
        with self._lock:
            self.release()
            self.source = source
            self._jpeg_provider = None
            self._stream_factory = None

    def set_jpeg_provider(self, provider: Callable[[], bytes]) -> None:
        """Use a callback returning one already encoded JPEG frame."""

        with self._lock:
            self.release()
            self.source = None
            self._jpeg_provider = provider
            self._stream_factory = None

    def set_stream(self, stream_factory: StreamFactory) -> None:
        """Use a callback returning an iterable of MJPEG chunks or JPEG frames."""

        with self._lock:
            self.release()
            self.source = None
            self._jpeg_provider = None
            self._stream_factory = stream_factory

    def release(self) -> None:
        if self._capture is not None:
            try:
                self._capture.release()
            finally:
                self._capture = None

    def read_jpeg(self) -> bytes:
        cv2 = self._cv2()
        with self._lock:
            jpeg_provider = self._jpeg_provider
            preview = self._preview_frame
            preview_is_fresh = time.time() - self._preview_updated_at <= 0.25
        if jpeg_provider is not None:
            return jpeg_provider()

        frame = preview.copy() if preview_is_fresh and preview is not None else self.read_frame()
        if frame is None:
            frame = self._placeholder_frame(cv2)
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            frame = self._placeholder_frame(cv2, "Frame encode failed")
            ok, encoded = cv2.imencode(".jpg", frame)
        return encoded.tobytes()

    def read_frame(self) -> Any:
        """Read one normalized BGR frame from the shared camera."""
        cv2 = self._cv2()
        frame = self._read_frame(cv2)
        if frame is None:
            return None
        frame = self._normalize_frame(cv2, frame)
        if self.rotate_180:
            frame = cv2.flip(frame, -1)
        return frame

    def set_preview_frame(self, frame: Any) -> None:
        """Publish an annotated frame for the web preview without reopening the camera."""
        with self._lock:
            self._preview_frame = None if frame is None else frame.copy()
            self._preview_updated_at = time.time()

    def frames(self, fps: float = 20.0):
        with self._lock:
            stream_factory = self._stream_factory
        if stream_factory is not None:
            for chunk in stream_factory():
                if chunk.startswith(b"--frame"):
                    yield chunk
                else:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + chunk + b"\r\n"
                    )
            return

        delay = 1.0 / max(1.0, fps)
        while True:
            frame = self.read_jpeg()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            time.sleep(delay)

    def _read_frame(self, cv2: Any) -> Any:
        with self._lock:
            source = self.source
            if source is None:
                return None

            if callable(source):
                frame = source()
                if isinstance(frame, tuple) and len(frame) == 2:
                    ok, frame = frame
                    return frame if ok else None
                return frame

            if hasattr(source, "read"):
                ok, frame = source.read()
                return frame if ok else None

            if self._capture is None:
                self._capture = cv2.VideoCapture(source)
                # Keep control vision close to real time instead of processing a
                # queue of stale frames when camera FPS exceeds processing FPS.
                self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not self._capture.isOpened():
                return None

            ok, frame = self._capture.read()
            if ok:
                return frame

            if isinstance(source, str):
                self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self._capture.read()
                if ok:
                    return frame
            return None

    def _normalize_frame(self, cv2: Any, frame: Any) -> Any:
        if getattr(frame, "ndim", None) == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if getattr(frame, "shape", (0, 0, 0))[2:3] == (4,):
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    def _placeholder_frame(self, cv2: Any, text: str = "No video source") -> Any:
        import numpy as np

        width, height = self.size
        frame = np.full((height, width, 3), (32, 36, 43), dtype=np.uint8)
        cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (72, 82, 96), 2)
        cv2.putText(
            frame,
            text,
            (40, height // 2 - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (230, 235, 240),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "Set video_source=0, a file path, or a frame callback",
            (40, height // 2 + 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (165, 174, 188),
            1,
            cv2.LINE_AA,
        )
        return frame

    @staticmethod
    def _cv2() -> Any:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "OpenCV is required for video streaming. Install opencv-python."
            ) from exc
        return cv2


class CarControlPanel:
    """A web page and API for controlling a small car robot."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8080,
        video_source: Any = 0,
        video_rotate_180: bool = False,
        title: str = "Car Control Panel",
        client_timeout: float = 3.0,
    ) -> None:
        self.host = host
        self.port = port
        self.title = title
        self.client_timeout = float(client_timeout)
        self._drive = DriveCommand()
        self._arm = ArmCommand()
        self._display = RobotDisplay()
        self._gyro = GyroData()
        self._work_mode = "manual"
        self._updated_at = time.time()
        self._last_control_at = 0.0
        self._lock = threading.RLock()
        self.video = OpenCVVideoStream(
            video_source,
            rotate_180=video_rotate_180,
        )
        self.app = self._create_app()
        self._server: Any = None
        self._server_thread: Any = None

    def run(
        self,
        host: Any = None,
        port: Any = None,
        debug: bool = False,
    ) -> None:
        """Run the control page in the current thread."""

        self.app.run(
            host=host or self.host,
            port=port or self.port,
            debug=debug,
            threaded=True,
            use_reloader=False,
        )

    def start(self, host: Any = None, port: Any = None) -> None:
        """Start the control page in a background thread."""

        from werkzeug.serving import make_server

        if self._server_thread and self._server_thread.is_alive():
            return

        self.host = host or self.host
        self.port = int(port or self.port)
        self._server = make_server(self.host, self.port, self.app, threaded=True)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="car-control-web",
            daemon=True,
        )
        self._server_thread.start()

    def stop(self) -> None:
        """Stop the background server and release the video source."""

        if self._server is not None:
            self._server.shutdown()
            self._server = None
        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)
            self._server_thread = None
        self.video.release()

    def url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def get_drive(self) -> DriveCommand:
        """Return the latest car drive command."""

        with self._lock:
            return DriveCommand(**self._drive.to_dict())

    def get_arm(self) -> ArmCommand:
        """Return the latest arm command."""

        with self._lock:
            return ArmCommand(
                mode=self._arm.mode,
                step=self._arm.step,
                axis1_negative=self._arm.axis1_negative,
                axis1_positive=self._arm.axis1_positive,
                axis2_negative=self._arm.axis2_negative,
                axis2_positive=self._arm.axis2_positive,
                axis3_negative=self._arm.axis3_negative,
                axis3_positive=self._arm.axis3_positive,
                x_negative=self._arm.x_negative,
                x_positive=self._arm.x_positive,
                y_negative=self._arm.y_negative,
                y_positive=self._arm.y_positive,
                z_negative=self._arm.z_negative,
                z_positive=self._arm.z_positive,
                gripper=self._arm.gripper,
            )

    def get_work_mode(self) -> str:
        """Return manual/auto mode selected on the page."""

        with self._lock:
            return self._work_mode

    def get_gyro(self) -> GyroData:
        """Return the latest phone gyroscope and motion data."""

        with self._lock:
            return GyroData(
                supported=self._gyro.supported,
                active=self._gyro.active,
                permission=self._gyro.permission,
                alpha=self._gyro.alpha,
                beta=self._gyro.beta,
                gamma=self._gyro.gamma,
                absolute=self._gyro.absolute,
                acceleration_x=self._gyro.acceleration_x,
                acceleration_y=self._gyro.acceleration_y,
                acceleration_z=self._gyro.acceleration_z,
                gravity_x=self._gyro.gravity_x,
                gravity_y=self._gyro.gravity_y,
                gravity_z=self._gyro.gravity_z,
                rotation_alpha=self._gyro.rotation_alpha,
                rotation_beta=self._gyro.rotation_beta,
                rotation_gamma=self._gyro.rotation_gamma,
                interval=self._gyro.interval,
                updated_at=self._gyro.updated_at,
            )

    def get_operation_info(self) -> dict[str, Any]:
        """Return all latest operation values as a plain dictionary."""

        with self._lock:
            return self._snapshot_unlocked()

    def is_connected(self) -> bool:
        """Return whether a web control page recently sent control data."""

        with self._lock:
            return self._is_connected_unlocked()

    def set_robot_state(
        self,
        angles: Any = None,
        position: Any = None,
    ) -> dict[str, Any]:
        """Update displayed physical angles and XYZ position."""

        with self._lock:
            if angles is not None:
                a1, a2, a3 = self._read_triplet(
                    angles,
                    ("axis1", "axis2", "axis3"),
                    (
                        self._display.axis1_angle,
                        self._display.axis2_angle,
                        self._display.axis3_angle,
                    ),
                )
                self._display.axis1_angle = a1
                self._display.axis2_angle = a2
                self._display.axis3_angle = a3

            if position is not None:
                x, y, z = self._read_triplet(
                    position,
                    ("x", "y", "z"),
                    (self._display.x, self._display.y, self._display.z),
                )
                self._display.x = x
                self._display.y = y
                self._display.z = z

            self._updated_at = time.time()
            return self._snapshot_unlocked()

    def set_video_source(
        self,
        source: Any,
    ) -> None:
        """Change the OpenCV video source."""

        self.video.set_source(source)

    def set_video_frame_provider(self, provider: FrameProvider) -> None:
        """Use a callback returning an OpenCV BGR frame."""

        self.video.set_source(provider)

    def set_video_jpeg_provider(self, provider: Callable[[], bytes]) -> None:
        """Use a callback returning one already encoded JPEG frame."""

        self.video.set_jpeg_provider(provider)

    def set_video_stream(self, stream_factory: StreamFactory) -> None:
        """Use a custom MJPEG stream or JPEG-frame iterable."""

        self.video.set_stream(stream_factory)

    def _update_control(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            now = time.time()
            self._last_control_at = now

            work_mode = payload.get("work_mode", self._work_mode)
            if work_mode in {"manual", "auto1", "auto2", "auto3"}:
                self._work_mode = work_mode

            drive = payload.get("drive")
            if isinstance(drive, Mapping):
                mode = drive.get("mode", self._drive.mode)
                if mode in {"joystick", "gyro"}:
                    self._drive.mode = mode
                self._drive.x = _clip(drive.get("x"), -1.0, 1.0, self._drive.x)
                self._drive.y = _clip(drive.get("y"), -1.0, 1.0, self._drive.y)
                self._drive.active = _bool(drive.get("active", self._drive.active))

            arm = payload.get("arm")
            if isinstance(arm, Mapping):
                mode = arm.get("mode", self._arm.mode)
                if mode in {"axis", "translate"}:
                    self._arm.mode = mode

                self._arm.step = _clip(arm.get("step"), 0.1, 10.0, self._arm.step)

                buttons = arm.get("buttons")
                if isinstance(buttons, Mapping):
                    axis_buttons = buttons.get("axis")
                    if isinstance(axis_buttons, Mapping):
                        self._update_arm_button_group_unlocked(
                            axis_buttons,
                            ("axis1", "axis2", "axis3"),
                        )

                    translation_buttons = buttons.get("translation")
                    if isinstance(translation_buttons, Mapping):
                        self._update_arm_button_group_unlocked(
                            translation_buttons,
                            ("x", "y", "z"),
                        )

                self._arm.gripper = _clip(
                    arm.get("gripper"),
                    0.0,
                    1.0,
                    self._arm.gripper,
                )

            gyro = payload.get("gyro")
            if isinstance(gyro, Mapping):
                self._update_gyro_unlocked(gyro)

            self._updated_at = now
            return self._snapshot_unlocked()

    def _snapshot_unlocked(self) -> dict[str, Any]:
        return {
            "connected": self._is_connected_unlocked(),
            "last_control_at": self._last_control_at,
            "work_mode": self._work_mode,
            "drive": self._drive.to_dict(),
            "arm": self._arm.to_dict(),
            "display": self._display.to_dict(),
            "gyro": self._gyro.to_dict(),
            "updated_at": self._updated_at,
        }

    def _is_connected_unlocked(self) -> bool:
        if self._last_control_at <= 0:
            return False
        return time.time() - self._last_control_at <= self.client_timeout

    def _update_gyro_unlocked(self, gyro: Mapping[str, Any]) -> None:
        self._gyro.supported = _bool(gyro.get("supported", self._gyro.supported))
        self._gyro.active = _bool(gyro.get("active", self._gyro.active))
        permission = gyro.get("permission", self._gyro.permission)
        if isinstance(permission, str):
            self._gyro.permission = permission

        orientation = gyro.get("orientation")
        if isinstance(orientation, Mapping):
            self._gyro.alpha = _clip(orientation.get("alpha"), -360.0, 360.0, self._gyro.alpha)
            self._gyro.beta = _clip(orientation.get("beta"), -360.0, 360.0, self._gyro.beta)
            self._gyro.gamma = _clip(orientation.get("gamma"), -360.0, 360.0, self._gyro.gamma)
            self._gyro.absolute = _bool(orientation.get("absolute", self._gyro.absolute))

        acceleration = gyro.get("acceleration")
        if isinstance(acceleration, Mapping):
            self._gyro.acceleration_x = _clip(
                acceleration.get("x"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.acceleration_x,
            )
            self._gyro.acceleration_y = _clip(
                acceleration.get("y"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.acceleration_y,
            )
            self._gyro.acceleration_z = _clip(
                acceleration.get("z"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.acceleration_z,
            )

        gravity = gyro.get("acceleration_including_gravity")
        if isinstance(gravity, Mapping):
            self._gyro.gravity_x = _clip(
                gravity.get("x"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.gravity_x,
            )
            self._gyro.gravity_y = _clip(
                gravity.get("y"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.gravity_y,
            )
            self._gyro.gravity_z = _clip(
                gravity.get("z"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.gravity_z,
            )

        rotation_rate = gyro.get("rotation_rate")
        if isinstance(rotation_rate, Mapping):
            self._gyro.rotation_alpha = _clip(
                rotation_rate.get("alpha"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.rotation_alpha,
            )
            self._gyro.rotation_beta = _clip(
                rotation_rate.get("beta"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.rotation_beta,
            )
            self._gyro.rotation_gamma = _clip(
                rotation_rate.get("gamma"),
                -1_000_000.0,
                1_000_000.0,
                self._gyro.rotation_gamma,
            )

        self._gyro.interval = _clip(gyro.get("interval"), 0.0, 1_000_000.0, self._gyro.interval)
        self._gyro.updated_at = _clip(gyro.get("updated_at"), 0.0, 1_000_000_000_000.0, time.time())

    def _update_arm_button_group_unlocked(
        self,
        payload: Mapping[str, Any],
        names: tuple[str, str, str],
    ) -> None:
        for name in names:
            pair = payload.get(name)
            if not isinstance(pair, Mapping):
                continue
            setattr(
                self._arm,
                f"{name}_negative",
                _bool(pair.get("negative", getattr(self._arm, f"{name}_negative"))),
            )
            setattr(
                self._arm,
                f"{name}_positive",
                _bool(pair.get("positive", getattr(self._arm, f"{name}_positive"))),
            )

    def _create_app(self):
        from flask import Flask, Response, jsonify, request, send_from_directory

        static_dir = Path(__file__).with_name("static")
        app = Flask(
            __name__,
            static_folder=str(static_dir),
            static_url_path="/static",
        )

        @app.route("/", methods=["GET"])
        def index():
            return send_from_directory(static_dir, "index.html")

        @app.route("/api/state", methods=["GET"])
        def api_state():
            return jsonify(self.get_operation_info())

        @app.route("/api/control", methods=["POST"])
        def api_control():
            payload = request.get_json(silent=True) or {}
            if not isinstance(payload, Mapping):
                return jsonify({"error": "JSON object required"}), 400
            return jsonify(self._update_control(payload))

        @app.route("/api/display", methods=["POST"])
        def api_display():
            payload = request.get_json(silent=True) or {}
            if not isinstance(payload, Mapping):
                return jsonify({"error": "JSON object required"}), 400
            return jsonify(
                self.set_robot_state(
                    angles=payload.get("angles"),
                    position=payload.get("position"),
                )
            )

        @app.route("/video_feed", methods=["GET"])
        def video_feed():
            return Response(
                self.video.frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        return app

    @staticmethod
    def _read_triplet(
        value: NumberTriplet,
        names: tuple[str, str, str],
        defaults: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        if isinstance(value, Mapping):
            return (
                _clip(value.get(names[0]), -1_000_000.0, 1_000_000.0, defaults[0]),
                _clip(value.get(names[1]), -1_000_000.0, 1_000_000.0, defaults[1]),
                _clip(value.get(names[2]), -1_000_000.0, 1_000_000.0, defaults[2]),
            )
        if len(value) != 3:
            raise ValueError("A triplet must contain exactly three values.")
        return (
            _clip(value[0], -1_000_000.0, 1_000_000.0, defaults[0]),
            _clip(value[1], -1_000_000.0, 1_000_000.0, defaults[1]),
            _clip(value[2], -1_000_000.0, 1_000_000.0, defaults[2]),
        )

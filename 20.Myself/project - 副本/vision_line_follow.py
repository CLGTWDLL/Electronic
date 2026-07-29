#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Camera-only line following for a forward/downward facing USB camera."""

from dataclasses import dataclass
from pathlib import Path
import math
import time
from typing import Any, Optional


@dataclass
class VisionLineCommand:
    turn: float = 0.0
    forward: float = 0.0
    found: bool = False
    confidence: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    flow_x: float = 0.0
    flow_y: float = 0.0
    message: str = "等待摄像头"


class CameraLineFollower:
    """Find a dark guide line and convert its predicted path to skid-steer input.

    Pixel measurements are intentionally normalized, so camera intrinsics are
    optional.  When ``camera_calibration.npz`` exists, lens distortion is
    removed before measuring the line.
    """

    def __init__(
        self,
        camera_height_cm: float,
        camera_forward_cm: float,
        camera_down_angle_deg: float,
        calibration_path: str = "camera_calibration.npz",
        cruise_speed: float = 0.65,
        max_turn: float = 0.72,
        process_fps: float = 20.0,
        min_line_width_ratio: float = 0.050,
        small_turn_gain: float = 0.35,
        cross_track_gain: float = 0.50,
    ) -> None:
        import cv2
        import numpy as np

        self.cv2 = cv2
        self.np = np
        self.camera_height_cm = float(camera_height_cm)
        self.camera_forward_cm = float(camera_forward_cm)
        self.camera_down_angle_deg = float(camera_down_angle_deg)
        self.calibration_path = Path(calibration_path)
        self.cruise_speed = float(cruise_speed)
        self.max_turn = float(max_turn)
        self.min_line_width_ratio = max(0.005, float(min_line_width_ratio))
        self.small_turn_gain = max(0.10, min(1.0, float(small_turn_gain)))
        self.cross_track_gain = max(0.10, min(1.0, float(cross_track_gain)))
        self.min_interval = 1.0 / max(1.0, float(process_fps))

        self.camera_matrix: Optional[Any] = None
        self.dist_coeffs: Optional[Any] = None
        self.calibration_size: Optional[tuple[int, int]] = None
        self.calibration_loaded = False
        self._load_calibration()

        self.last_processed_at = 0.0
        self.last_seen_at = 0.0
        self.last_error = 0.0
        self.last_turn = 0.0
        self.filtered_lateral = 0.0
        self.filtered_heading = 0.0
        self.expected_line_width_px: Optional[float] = None
        self.last_command = VisionLineCommand()
        self.previous_gray: Optional[Any] = None
        self.debug_frame: Optional[Any] = None

    def _load_calibration(self) -> None:
        if not self.calibration_path.exists():
            return
        try:
            data = self.np.load(str(self.calibration_path))
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            if "image_width" in data and "image_height" in data:
                self.calibration_size = (
                    int(data["image_width"]), int(data["image_height"])
                )
            self.calibration_loaded = True
        except (KeyError, OSError, ValueError):
            self.camera_matrix = None
            self.dist_coeffs = None

    def reset(self) -> None:
        self.last_processed_at = 0.0
        self.last_seen_at = 0.0
        self.last_error = 0.0
        self.last_turn = 0.0
        self.filtered_lateral = 0.0
        self.filtered_heading = 0.0
        self.expected_line_width_px = None
        self.last_command = VisionLineCommand(message="视觉循迹已复位")
        self.previous_gray = None

    def ready(self, now: Optional[float] = None) -> bool:
        """Return whether it is time to acquire and process another frame."""
        now = time.monotonic() if now is None else float(now)
        return now - self.last_processed_at >= self.min_interval

    def update(self, frame: Any, now: Optional[float] = None) -> VisionLineCommand:
        now = time.monotonic() if now is None else float(now)
        if now - self.last_processed_at < self.min_interval:
            return self.last_command
        self.last_processed_at = now

        if frame is None:
            return self._line_lost(now, None, "摄像头无画面")

        if self.calibration_loaded:
            frame = self.cv2.undistort(
                frame, self.camera_matrix, self.dist_coeffs
            )

        height, width = frame.shape[:2]
        roi_top = int(height * 0.32)
        roi_bottom = int(height * 0.96)
        roi = frame[roi_top:roi_bottom]
        gray = self.cv2.cvtColor(roi, self.cv2.COLOR_BGR2GRAY)
        gray = self.cv2.GaussianBlur(gray, (5, 5), 0)

        # Use the complete globally dark region. Intersecting this with an
        # adaptive mask removes the uniform interior of a wide black tape and
        # leaves only two thin edges, which then look exactly like floor seams.
        otsu_level, _ = self.cv2.threshold(
            gray, 0, 255, self.cv2.THRESH_BINARY_INV + self.cv2.THRESH_OTSU
        )
        # Cap an unusually high Otsu level so grey shadows/floor patterns do not
        # become line candidates, while retaining the full black stripe.
        dark_limit = max(45.0, min(140.0, float(otsu_level)))
        _, mask = self.cv2.threshold(
            gray,
            dark_limit,
            255,
            self.cv2.THRESH_BINARY_INV,
        )
        kernel = self.cv2.getStructuringElement(self.cv2.MORPH_ELLIPSE, (5, 5))
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_OPEN, kernel)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_CLOSE, kernel, iterations=2)

        component, area_ratio = self._select_line_component(mask)
        flow_x, flow_y = self._optical_flow(gray)
        centers = self._scan_centers(component)
        if len(centers) < 3:
            return self._line_lost(now, frame, "未识别到连续引导线", flow_x, flow_y)

        centers.sort(key=lambda item: item[0], reverse=True)
        near_points = centers[: min(2, len(centers))]
        far_points = centers[-min(2, len(centers)) :]
        near_x = sum(point[1] for point in near_points) / len(near_points)
        far_x = sum(point[1] for point in far_points) / len(far_points)
        near_y = sum(point[0] for point in near_points) / len(near_points) + roi_top
        far_y = sum(point[0] for point in far_points) / len(far_points) + roi_top
        half_width = max(1.0, width / 2.0)
        raw_lateral_error = (near_x - width / 2.0) / half_width
        raw_heading_error = (far_x - near_x) / half_width

        # Independently smooth centre displacement and path direction. Heading
        # uses stronger filtering because a few far pixels otherwise cause sway.
        if self.last_seen_at == 0.0:
            self.filtered_lateral = raw_lateral_error
            self.filtered_heading = raw_heading_error
        else:
            lateral_delta = abs(raw_lateral_error - self.filtered_lateral)
            heading_delta = abs(raw_heading_error - self.filtered_heading)
            # Fast attack for a newly appearing bend, gentler smoothing for
            # small camera jitter around the current path.
            lateral_alpha = 0.70 if lateral_delta > 0.10 else 0.45
            heading_alpha = 0.55 if heading_delta > 0.08 else 0.32
            self.filtered_lateral = (
                (1.0 - lateral_alpha) * self.filtered_lateral
                + lateral_alpha * raw_lateral_error
            )
            self.filtered_heading = (
                (1.0 - heading_alpha) * self.filtered_heading
                + heading_alpha * raw_heading_error
            )
        lateral_error = self.filtered_lateral
        heading_error = self.filtered_heading

        measured_width = sum(point[2] for point in centers) / len(centers)
        if self.expected_line_width_px is None:
            self.expected_line_width_px = measured_width
        else:
            self.expected_line_width_px = (
                0.88 * self.expected_line_width_px + 0.12 * measured_width
            )

        # Predict where the line should be when the rotation centre reaches it.
        # Far-line heading advances the turn; error derivative and optical flow
        # add damping without requiring metric vehicle velocity.
        error_rate = lateral_error - self.last_error
        filtered_near_x = width / 2.0 + lateral_error * half_width
        filtered_far_x = filtered_near_x + heading_error * half_width
        near_ground = self._pixel_to_ground(
            filtered_near_x, near_y, width, height
        )
        far_ground = self._pixel_to_ground(
            filtered_far_x, far_y, width, height
        )
        if near_ground is not None and far_ground is not None:
            lateral_angle = math.atan2(
                near_ground[0], max(5.0, near_ground[1])
            )
            path_heading = math.atan2(
                far_ground[0] - near_ground[0],
                max(5.0, far_ground[1] - near_ground[1]),
            )
            predicted_error = (
                0.70 * lateral_angle + 0.42 * path_heading
            ) / math.radians(25.0)
            predicted_error += 0.18 * error_rate - 0.02 * flow_x
        else:
            predicted_error = (
                0.65 * lateral_error
                + 0.42 * heading_error
                + 0.18 * error_rate
                - 0.02 * flow_x
            )
        target_turn = max(
            -self.max_turn, min(self.max_turn, predicted_error * 1.05)
        )
        # Cross-track error must not be cancelled by an opposite far-line
        # heading. Use a continuous proportional floor after a small deadband;
        # a hard threshold here creates a left/right limit-cycle on straights.
        cross_track_magnitude = abs(lateral_error)
        cross_track_deadband = 0.05
        if cross_track_magnitude > cross_track_deadband:
            minimum_cross_track_turn = min(
                self.max_turn,
                self.cross_track_gain
                * (cross_track_magnitude - cross_track_deadband),
            )
            if (
                target_turn * lateral_error <= 0.0
                or abs(target_turn) < minimum_cross_track_turn
            ):
                target_turn = math.copysign(
                    minimum_cross_track_turn, lateral_error
                )
        # Four-wheel skid steering is very sensitive near zero. Compress only
        # the small-turn range when the stripe itself is already near centre;
        # never soften a large lateral displacement just because heading terms
        # happen to cancel each other.
        target_magnitude = abs(target_turn)
        if target_magnitude < 0.040:
            target_turn = 0.0
        elif target_magnitude < 0.25 and cross_track_magnitude < 0.18:
            base_soft_gain = self.small_turn_gain + (
                1.0 - self.small_turn_gain
            ) * target_magnitude / 0.25
            # Fade the soft zone out continuously between 10% and 18% lateral
            # error instead of switching it off at one threshold.
            soft_blend = max(
                0.0, min(1.0, (0.18 - cross_track_magnitude) / 0.08)
            )
            soft_gain = 1.0 - soft_blend * (1.0 - base_soft_gain)
            target_turn = math.copysign(
                target_magnitude * soft_gain, target_turn
            )
        same_direction = target_turn * self.last_turn >= 0.0
        growing_correction = abs(target_turn) > abs(self.last_turn)
        raw_path_centered = (
            abs(raw_lateral_error) < 0.04 and abs(raw_heading_error) < 0.06
        )
        if raw_path_centered and abs(self.last_turn) > 0.10:
            # Do not let the low-pass tail keep steering after the measured line
            # has already returned to the centre; that is a common overshoot cause.
            turn_alpha = 0.68
            max_turn_step = 0.24
        elif not same_direction:
            turn_alpha = 0.72
            max_turn_step = 0.24
        elif abs(target_turn) >= 0.25:
            turn_alpha = 0.72 if same_direction else 0.58
            max_turn_step = 0.20
        elif growing_correction:
            # Build small corrections gently so tyre scrub does not turn a tiny
            # centre error into a large yaw response.
            turn_alpha = 0.38
            max_turn_step = 0.06
        else:
            # Release correction faster than it was applied; this prevents the
            # filter tail from carrying the car across the line centre.
            turn_alpha = 0.60
            max_turn_step = 0.12
        smoothed_turn = (
            (1.0 - turn_alpha) * self.last_turn + turn_alpha * target_turn
        )
        turn = max(
            self.last_turn - max_turn_step,
            min(self.last_turn + max_turn_step, smoothed_turn),
        )
        if target_turn == 0.0 and abs(turn) < 0.015:
            turn = 0.0
        confidence = max(
            0.0,
            min(1.0, 0.45 * min(1.0, len(centers) / 4.0) + 0.55 * min(1.0, area_ratio / 0.08)),
        )
        # Steering is deliberately slew-limited, but speed must react to a large
        # newly observed error immediately instead of waiting for turn ramp-up.
        speed_turn = max(
            abs(turn),
            abs(target_turn),
            min(1.0, abs(raw_lateral_error) * 1.2),
        )
        forward = max(
            0.22,
            self.cruise_speed
            * (0.35 + 0.65 * confidence)
            * (1.0 - 0.58 * speed_turn),
        )

        self.last_seen_at = now
        self.last_error = lateral_error
        self.last_turn = turn
        command = VisionLineCommand(
            turn=turn,
            forward=forward,
            found=True,
            confidence=confidence,
            lateral_error=lateral_error,
            heading_error=heading_error,
            flow_x=flow_x,
            flow_y=flow_y,
            message="视觉循迹正常",
        )
        self.last_command = command
        self.debug_frame = self._draw_debug(
            frame, roi_top, component, centers, command
        )
        return command

    def _select_line_component(self, mask: Any) -> tuple[Any, float]:
        contours, _ = self.cv2.findContours(
            mask, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE
        )
        selected = self.np.zeros_like(mask)
        image_area = float(mask.shape[0] * mask.shape[1])
        best_score = 0.0
        best = None
        image_width = mask.shape[1]
        minimum_mean_width = max(7.0, image_width * self.min_line_width_ratio)
        # A close stripe can occupy a large part of the image due to perspective.
        maximum_mean_width = image_width * 0.60
        expected_width = self.expected_line_width_px or image_width * 0.035
        expected_center = image_width / 2.0 + self.last_error * image_width / 2.0
        for contour in contours:
            area = self.cv2.contourArea(contour)
            if area < image_area * 0.001:
                continue
            x, y, w, h = self.cv2.boundingRect(contour)
            mean_width = area / max(1.0, h)
            vertical_reach = h / mask.shape[0]
            # Floor seams are normally long but only a few pixels wide. A real
            # guide stripe has measurable width over a continuous vertical span.
            if (
                mean_width < minimum_mean_width
                or mean_width > maximum_mean_width
                or vertical_reach < 0.18
            ):
                continue
            centre_distance = abs((x + w / 2.0) - expected_center) / image_width
            vertical_reach = h / mask.shape[0]
            bottom_reach = (y + h) / mask.shape[0]
            width_ratio = max(0.05, mean_width / max(1.0, expected_width))
            width_score = math.exp(-abs(math.log(width_ratio)))
            temporal_score = 1.0 - 0.70 * min(1.0, centre_distance)
            score = (
                area
                * (0.7 + vertical_reach + 0.5 * bottom_reach)
                * (0.40 + 0.60 * width_score)
                * temporal_score
            )
            if score > best_score:
                best_score = score
                best = contour
        if best is None:
            return selected, 0.0
        self.cv2.drawContours(selected, [best], -1, 255, thickness=-1)
        return selected, self.cv2.contourArea(best) / image_area

    def _scan_centers(self, component: Any) -> list[tuple[float, float, float]]:
        height, width = component.shape[:2]
        centers = []
        expected_center = width / 2.0 + self.last_error * width / 2.0
        base_minimum_width = width * self.min_line_width_ratio
        for fraction in (0.86, 0.70, 0.54, 0.38, 0.24):
            y = int(height * fraction)
            band = component[max(0, y - 5) : min(height, y + 6)]
            # Far parts of a tape are narrower because of perspective, while a
            # near run must be wider to avoid accepting a floor seam intersection.
            minimum_width = base_minimum_width * (0.55 + 0.55 * fraction)
            row_results = []
            for row in band:
                occupied = row != 0
                padded = self.np.concatenate(
                    (
                        self.np.array([False]),
                        occupied,
                        self.np.array([False]),
                    )
                )
                changes = self.np.diff(padded.astype(self.np.int8))
                starts = self.np.flatnonzero(changes == 1)
                ends = self.np.flatnonzero(changes == -1)
                candidates = []
                for left, right_exclusive in zip(starts, ends):
                    run_width = float(right_exclusive - left)
                    if run_width < minimum_width:
                        continue
                    run_center = (left + right_exclusive - 1) / 2.0
                    distance = abs(run_center - expected_center) / max(1.0, width)
                    score = run_width * (1.0 - 0.45 * min(1.0, distance))
                    candidates.append((score, run_center, run_width))
                if candidates:
                    _, run_center, run_width = max(
                        candidates, key=lambda item: item[0]
                    )
                    row_results.append((run_center, run_width))

            # A valid stripe must be present in several neighbouring rows. The
            # median keeps small holes/reflections from moving the centre.
            if len(row_results) >= max(3, band.shape[0] // 3):
                run_center = float(
                    self.np.median([item[0] for item in row_results])
                )
                run_width = float(
                    self.np.median([item[1] for item in row_results])
                )
                centers.append((float(y), run_center, run_width))
        return centers

    def _pixel_to_ground(
        self, pixel_x: float, pixel_y: float, width: int, height: int
    ) -> Optional[tuple[float, float]]:
        """Project an image pixel to (right, forward) ground coordinates in cm."""
        if self.camera_matrix is not None:
            matrix = self.camera_matrix
            scale_x = 1.0
            scale_y = 1.0
            if self.calibration_size is not None:
                scale_x = width / max(1.0, self.calibration_size[0])
                scale_y = height / max(1.0, self.calibration_size[1])
            fx = float(matrix[0, 0]) * scale_x
            fy = float(matrix[1, 1]) * scale_y
            cx = float(matrix[0, 2]) * scale_x
            cy = float(matrix[1, 2]) * scale_y
        else:
            # Fallback for a typical 60°x46° USB camera. Intrinsic calibration
            # replaces this approximation automatically.
            fx = width / (2.0 * math.tan(math.radians(60.0 / 2.0)))
            fy = height / (2.0 * math.tan(math.radians(46.0 / 2.0)))
            cx = width / 2.0
            cy = height / 2.0

        ray_right = (pixel_x - cx) / max(1.0, fx)
        ray_down = (pixel_y - cy) / max(1.0, fy)
        pitch = math.radians(self.camera_down_angle_deg)
        ground_down = ray_down * math.cos(pitch) + math.sin(pitch)
        ground_forward = math.cos(pitch) - ray_down * math.sin(pitch)
        if ground_down <= 0.02:
            return None
        distance_scale = self.camera_height_cm / ground_down
        right_cm = distance_scale * ray_right
        forward_cm = self.camera_forward_cm + distance_scale * ground_forward
        return right_cm, forward_cm

    def _optical_flow(self, gray: Any) -> tuple[float, float]:
        flow_x = 0.0
        flow_y = 0.0
        if self.previous_gray is not None and self.previous_gray.shape == gray.shape:
            points = self.cv2.goodFeaturesToTrack(
                self.previous_gray,
                maxCorners=60,
                qualityLevel=0.02,
                minDistance=12,
                blockSize=7,
            )
            if points is not None:
                moved, status, _ = self.cv2.calcOpticalFlowPyrLK(
                    self.previous_gray, gray, points, None
                )
                if moved is not None and status is not None:
                    valid = status.reshape(-1) == 1
                    delta = moved.reshape(-1, 2)[valid] - points.reshape(-1, 2)[valid]
                    if len(delta) >= 5:
                        flow_x = float(self.np.median(delta[:, 0]) / max(1, gray.shape[1] / 2))
                        flow_y = float(self.np.median(delta[:, 1]) / max(1, gray.shape[0]))
        self.previous_gray = gray.copy()
        return flow_x, flow_y

    def _line_lost(
        self,
        now: float,
        frame: Any,
        message: str,
        flow_x: float = 0.0,
        flow_y: float = 0.0,
    ) -> VisionLineCommand:
        age = now - self.last_seen_at if self.last_seen_at else math.inf
        if age <= 0.25:
            turn = self.last_turn
            forward = 0.20
            message += "，短暂保持上一控制量"
        elif age <= 0.7 and self.last_turn != 0:
            turn = 0.30 if self.last_turn > 0 else -0.30
            forward = 0.18
            message += "，按上次方向搜索"
        else:
            turn = 0.0
            forward = 0.0
            message += "，停车"
        command = VisionLineCommand(
            turn=turn,
            forward=forward,
            found=False,
            flow_x=flow_x,
            flow_y=flow_y,
            message=message,
        )
        self.last_command = command
        if frame is not None:
            self.debug_frame = frame.copy()
            if forward == 0.0:
                overlay_text = "VISION: LINE LOST - STOP"
            elif turn > 0:
                overlay_text = "VISION: LINE LOST - SEARCH RIGHT"
            elif turn < 0:
                overlay_text = "VISION: LINE LOST - SEARCH LEFT"
            else:
                overlay_text = "VISION: LINE LOST - HOLD"
            self.cv2.putText(
                self.debug_frame,
                overlay_text,
                (18, 32),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                self.cv2.LINE_AA,
            )
        return command

    def _draw_debug(
        self,
        frame: Any,
        roi_top: int,
        component: Any,
        centers: list[tuple[float, float, float]],
        command: VisionLineCommand,
    ) -> Any:
        debug = frame.copy()
        overlay = self.np.zeros_like(debug)
        accepted = self.np.zeros_like(component)
        ordered_centers = sorted(centers, key=lambda item: item[0])
        if len(ordered_centers) >= 2:
            left_edge = [
                (int(x - line_width / 2.0), int(y))
                for y, x, line_width in ordered_centers
            ]
            right_edge = [
                (int(x + line_width / 2.0), int(y))
                for y, x, line_width in reversed(ordered_centers)
            ]
            polygon = self.np.array(left_edge + right_edge, dtype=self.np.int32)
            self.cv2.fillPoly(accepted, [polygon], 255)
        overlay[roi_top : roi_top + component.shape[0], :, 1] = accepted
        debug = self.cv2.addWeighted(debug, 1.0, overlay, 0.25, 0)
        center_points = []
        for y, x, line_width in centers:
            self.cv2.circle(debug, (int(x), int(y + roi_top)), 6, (0, 255, 255), -1)
            half_line = int(line_width / 2.0)
            self.cv2.line(
                debug,
                (int(x) - half_line, int(y + roi_top)),
                (int(x) + half_line, int(y + roi_top)),
                (255, 255, 0),
                2,
            )
            center_points.append((int(x), int(y + roi_top)))
        if len(center_points) >= 2:
            center_points.sort(key=lambda point: point[1])
            self.cv2.polylines(
                debug,
                [self.np.array(center_points, dtype=self.np.int32)],
                False,
                (0, 255, 255),
                3,
            )
        target_x = int(debug.shape[1] / 2 + command.turn * debug.shape[1] * 0.25)
        self.cv2.line(
            debug,
            (debug.shape[1] // 2, debug.shape[0] - 1),
            (target_x, roi_top),
            (255, 80, 30),
            3,
        )
        calibration_text = "calibrated" if self.calibration_loaded else "uncalibrated"
        text = (
            f"VISION {calibration_text} conf={command.confidence:.2f} "
            f"turn={command.turn:+.2f} speed={command.forward:.2f}"
        )
        self.cv2.putText(
            debug,
            text,
            (14, 28),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (40, 255, 40),
            2,
            self.cv2.LINE_AA,
        )
        return debug

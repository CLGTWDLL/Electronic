#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fit the physical 3D pipe axis P(s) = T + s*r from position_001..011.csv.

Coordinates:
  * s / real_cm is in centimetres.
  * pipe centre is 0 cm.
  * left from centre is positive; right is negative.
  * input x,y are raw 640x640 distorted image pixels.

The script uses camera intrinsics and Brown-Conrady distortion, rejects
per-position outliers with MAD, undistorts every retained observation, fits
T and unit direction r, and writes a K230-friendly geometry model. It never
reads the legacy PCA/segments position_map.json.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


MAD_Z_THRESHOLD = 3.5
HUBER_DELTA_PX = 1.5
MAX_LM_ITERATIONS = 80
EXPECTED_POSITIONS_CM = (-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10)

SCRIPT_DIR = Path(__file__).resolve().parent
POSITION_DIR = SCRIPT_DIR.parent / "position_calibration2"
CAMERA_JSON = (
    SCRIPT_DIR.parent
    / "camera_calibration_640"
    / "camera_calibration.json"
)
OUTPUT_JSON = SCRIPT_DIR / "geometry_position_model.json"
OUTPUT_CSV = SCRIPT_DIR / "geometry_position_summary.csv"


def load_camera() -> tuple[np.ndarray, np.ndarray, dict]:
    data = json.loads(CAMERA_JSON.read_text(encoding="utf-8"))
    size = data.get("image_size", {})
    if (size.get("width"), size.get("height")) != (640, 640):
        raise ValueError("camera calibration must be for 640x640 images")
    camera_matrix = np.asarray(
        data["camera_matrix"],
        dtype=np.float64,
    )
    distortion = np.asarray(
        data["distortion_coefficients"],
        dtype=np.float64,
    ).reshape(-1)
    if camera_matrix.shape != (3, 3) or distortion.size < 5:
        raise ValueError("invalid camera calibration parameters")
    return camera_matrix, distortion[:5], data


def load_csv_samples() -> list[dict]:
    samples: list[dict] = []
    for index in range(1, 12):
        path = POSITION_DIR / ("position_%03d.csv" % index)
        if not path.is_file():
            raise FileNotFoundError("missing calibration CSV: " + str(path))
        with path.open("r", encoding="utf-8-sig", newline="") as source:
            reader = csv.DictReader(source)
            required = {"x", "y", "confidence", "real_cm"}
            if reader.fieldnames is None or not required.issubset(
                set(reader.fieldnames)
            ):
                raise ValueError(path.name + " has invalid columns")
            for row_number, row in enumerate(reader, start=2):
                try:
                    sample = {
                        "source": path.name,
                        "row": row_number,
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "confidence": float(row["confidence"]),
                        "real_cm": float(row["real_cm"]),
                    }
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        "%s row %d contains invalid numbers"
                        % (path.name, row_number)
                    ) from error
                if not all(
                    math.isfinite(sample[key])
                    for key in ("x", "y", "confidence", "real_cm")
                ):
                    raise ValueError(
                        "%s row %d contains non-finite values"
                        % (path.name, row_number)
                    )
                samples.append(sample)

    actual_positions = tuple(
        sorted({int(round(sample["real_cm"])) for sample in samples})
    )
    if actual_positions != EXPECTED_POSITIONS_CM:
        raise ValueError(
            "expected real_cm groups %s, got %s"
            % (EXPECTED_POSITIONS_CM, actual_positions)
        )
    return samples


def mad(values: np.ndarray) -> float:
    centre = np.median(values)
    return float(np.median(np.abs(values - centre)))


def robust_z(values: np.ndarray) -> np.ndarray:
    centre = float(np.median(values))
    scale = mad(values)
    deviation = np.abs(values - centre)
    if scale <= 1e-12:
        return np.where(deviation <= 1e-9, 0.0, np.inf)
    return 0.6744897501960817 * deviation / scale


def reject_group_outliers(
    samples: list[dict],
) -> tuple[list[dict], list[dict], dict]:
    x = np.asarray([sample["undistorted_x"] for sample in samples])
    y = np.asarray([sample["undistorted_y"] for sample in samples])
    confidence = np.asarray(
        [sample["confidence"] for sample in samples]
    )
    distance = np.sqrt(
        robust_z(x) ** 2
        + robust_z(y) ** 2
        + 0.25 * robust_z(confidence) ** 2
    )
    keep = distance <= MAD_Z_THRESHOLD
    if int(np.count_nonzero(keep)) < max(10, len(samples) // 2):
        raise RuntimeError(
            "MAD filtering retained too few samples at %.3f cm"
            % samples[0]["real_cm"]
        )
    retained = [
        sample for sample, accepted in zip(samples, keep) if accepted
    ]
    rejected = [
        sample for sample, accepted in zip(samples, keep) if not accepted
    ]
    stats = {
        "raw_count": len(samples),
        "kept_count": len(retained),
        "rejected_count": len(rejected),
        "undistorted_median_x": float(np.median(x)),
        "undistorted_median_y": float(np.median(y)),
    }
    return retained, rejected, stats


def undistort_to_rays(
    pixels: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2),
        camera_matrix,
        distortion,
    ).reshape(-1, 2)
    rays = np.column_stack(
        (normalized, np.ones(normalized.shape[0], dtype=np.float64))
    )
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    return rays


def add_undistorted_coordinates(
    samples: list[dict],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> None:
    pixels = np.asarray(
        [[sample["x"], sample["y"]] for sample in samples],
        dtype=np.float64,
    )
    undistorted = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2),
        camera_matrix,
        distortion,
        P=camera_matrix,
    ).reshape(-1, 2)
    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2),
        camera_matrix,
        distortion,
    ).reshape(-1, 2)
    for sample, pixel, point in zip(samples, undistorted, normalized):
        ray = np.asarray([point[0], point[1], 1.0], dtype=np.float64)
        ray /= np.linalg.norm(ray)
        sample["undistorted_x"] = float(pixel[0])
        sample["undistorted_y"] = float(pixel[1])
        sample["ray"] = ray


def cross_matrix(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        dtype=np.float64,
    )


def linear_axis_initialization(
    rays: np.ndarray,
    real_cm: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    identity = np.eye(3, dtype=np.float64)
    for ray, position in zip(rays, real_cm):
        cross = cross_matrix(ray)
        rows.append(
            np.hstack((cross, position * cross)) @ np.block(
                [
                    [identity, np.zeros((3, 3))],
                    [np.zeros((3, 3)), identity],
                ]
            )
        )
    matrix = np.vstack(rows)
    _, _, right_vectors = np.linalg.svd(matrix, full_matrices=False)
    solution = right_vectors[-1]
    translation_scaled = solution[:3]
    direction_scaled = solution[3:]
    direction_scale = np.linalg.norm(direction_scaled)
    if direction_scale <= 1e-12:
        raise RuntimeError("linear 1D-PnP initialization is degenerate")
    translation = translation_scaled / direction_scale
    direction = direction_scaled / direction_scale

    depths = (translation[None, :] + real_cm[:, None] * direction)[:, 2]
    if float(np.median(depths)) < 0.0:
        translation = -translation
        direction = -direction
    return translation, direction


def direction_to_angles(direction: np.ndarray) -> tuple[float, float]:
    theta = math.atan2(float(direction[1]), float(direction[0]))
    phi = math.asin(float(np.clip(direction[2], -1.0, 1.0)))
    return theta, phi


def angles_to_direction(theta: float, phi: float) -> np.ndarray:
    cos_phi = math.cos(phi)
    return np.asarray(
        [
            cos_phi * math.cos(theta),
            cos_phi * math.sin(theta),
            math.sin(phi),
        ],
        dtype=np.float64,
    )


def unpack(parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    translation = parameters[:3]
    direction = angles_to_direction(parameters[3], parameters[4])
    return translation, direction


def project_axis_points(
    parameters: np.ndarray,
    real_cm: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    translation, direction = unpack(parameters)
    points = translation[None, :] + real_cm[:, None] * direction[None, :]
    if np.any(points[:, 2] <= 1e-6):
        return np.full((real_cm.size, 2), 1e6, dtype=np.float64)
    projected, _ = cv2.projectPoints(
        points.reshape(-1, 1, 3),
        np.zeros(3),
        np.zeros(3),
        camera_matrix,
        distortion,
    )
    return projected.reshape(-1, 2)


def residual_vector(
    parameters: np.ndarray,
    observed_pixels: np.ndarray,
    real_cm: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    return (
        project_axis_points(
            parameters,
            real_cm,
            camera_matrix,
            distortion,
        )
        - observed_pixels
    ).reshape(-1)


def robust_weights(residual: np.ndarray) -> np.ndarray:
    point_error = np.linalg.norm(residual.reshape(-1, 2), axis=1)
    point_weight = np.ones_like(point_error)
    large = point_error > HUBER_DELTA_PX
    point_weight[large] = HUBER_DELTA_PX / point_error[large]
    return np.repeat(np.sqrt(point_weight), 2)


def refine_axis_lm(
    initial_translation: np.ndarray,
    initial_direction: np.ndarray,
    observed_pixels: np.ndarray,
    real_cm: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict]:
    theta, phi = direction_to_angles(initial_direction)
    parameters = np.asarray(
        [
            initial_translation[0],
            initial_translation[1],
            initial_translation[2],
            theta,
            phi,
        ],
        dtype=np.float64,
    )
    damping = 1e-3
    steps = np.asarray([1e-4, 1e-4, 1e-4, 1e-6, 1e-6])
    accepted_iterations = 0

    def cost_and_residual(candidate):
        residual = residual_vector(
            candidate,
            observed_pixels,
            real_cm,
            camera_matrix,
            distortion,
        )
        weight = robust_weights(residual)
        weighted = residual * weight
        return float(weighted @ weighted), residual, weight

    cost, residual, weight = cost_and_residual(parameters)
    for _ in range(MAX_LM_ITERATIONS):
        jacobian = np.empty((residual.size, parameters.size))
        for column in range(parameters.size):
            shifted = parameters.copy()
            shifted[column] += steps[column]
            shifted_residual = residual_vector(
                shifted,
                observed_pixels,
                real_cm,
                camera_matrix,
                distortion,
            )
            jacobian[:, column] = (
                shifted_residual - residual
            ) / steps[column]

        weighted_jacobian = jacobian * weight[:, None]
        weighted_residual = residual * weight
        normal = weighted_jacobian.T @ weighted_jacobian
        gradient = weighted_jacobian.T @ weighted_residual
        diagonal = np.maximum(np.diag(normal), 1e-9)
        try:
            delta = np.linalg.solve(
                normal + damping * np.diag(diagonal),
                -gradient,
            )
        except np.linalg.LinAlgError:
            delta = np.linalg.lstsq(
                normal + damping * np.diag(diagonal),
                -gradient,
                rcond=None,
            )[0]

        candidate = parameters + delta
        candidate_cost, candidate_residual, candidate_weight = (
            cost_and_residual(candidate)
        )
        if candidate_cost < cost:
            parameters = candidate
            cost = candidate_cost
            residual = candidate_residual
            weight = candidate_weight
            damping = max(damping / 3.0, 1e-9)
            accepted_iterations += 1
            if float(np.linalg.norm(delta)) < 1e-9:
                break
        else:
            damping = min(damping * 10.0, 1e12)
            if damping >= 1e12:
                break

    translation, direction = unpack(parameters)
    return translation, direction, {
        "accepted_iterations": accepted_iterations,
        "robust_cost": cost,
        "final_damping": damping,
    }


def closest_axis_coordinate(
    rays: np.ndarray,
    translation: np.ndarray,
    direction: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ray_dot_direction = rays @ direction
    ray_dot_translation = rays @ translation
    direction_dot_translation = float(direction @ translation)
    denominator = 1.0 - ray_dot_direction * ray_dot_direction
    if np.any(denominator <= 1e-10):
        raise RuntimeError("viewing ray is nearly parallel to pipe axis")
    depth = (
        ray_dot_translation
        - ray_dot_direction * direction_dot_translation
    ) / denominator
    position = ray_dot_direction * depth - direction_dot_translation
    ray_points = depth[:, None] * rays
    axis_points = (
        translation[None, :] + position[:, None] * direction[None, :]
    )
    closest_distance = np.linalg.norm(ray_points - axis_points, axis=1)
    return position, closest_distance, depth


def robust_limit(values: np.ndarray, floor: float) -> float:
    centre = float(np.median(values))
    spread = mad(values)
    return max(floor, centre + 3.5 * 1.4826 * spread)


def main() -> None:
    camera_matrix, distortion, camera_data = load_camera()
    raw_samples = load_csv_samples()
    add_undistorted_coordinates(
        raw_samples, camera_matrix, distortion
    )

    grouped: dict[float, list[dict]] = {}
    for sample in raw_samples:
        grouped.setdefault(sample["real_cm"], []).append(sample)

    retained_samples: list[dict] = []
    filter_stats: dict[float, dict] = {}
    for real_cm in sorted(grouped):
        retained, _, stats = reject_group_outliers(grouped[real_cm])
        retained_samples.extend(retained)
        filter_stats[real_cm] = stats

    pixels = np.asarray(
        [
            [sample["undistorted_x"], sample["undistorted_y"]]
            for sample in retained_samples
        ],
        dtype=np.float64,
    )
    real_cm = np.asarray(
        [sample["real_cm"] for sample in retained_samples],
        dtype=np.float64,
    )
    rays = np.asarray(
        [sample["ray"] for sample in retained_samples],
        dtype=np.float64,
    )
    zero_distortion = np.zeros_like(distortion)

    initial_t, initial_r = linear_axis_initialization(rays, real_cm)
    translation, direction, optimizer = refine_axis_lm(
        initial_t,
        initial_r,
        pixels,
        real_cm,
        camera_matrix,
        zero_distortion,
    )

    # Direction must follow the supplied sign convention: left is positive.
    positive_pixel = project_axis_points(
        np.asarray(
            [
                translation[0],
                translation[1],
                translation[2],
                *direction_to_angles(direction),
            ]
        ),
        np.asarray([1.0]),
        camera_matrix,
        zero_distortion,
    )[0]
    zero_pixel = project_axis_points(
        np.asarray(
            [
                translation[0],
                translation[1],
                translation[2],
                *direction_to_angles(direction),
            ]
        ),
        np.asarray([0.0]),
        camera_matrix,
        zero_distortion,
    )[0]
    if positive_pixel[0] >= zero_pixel[0]:
        raise RuntimeError(
            "fitted +s direction is not image-left; check CSV labels"
        )

    predicted_pixels = project_axis_points(
        np.asarray(
            [
                translation[0],
                translation[1],
                translation[2],
                *direction_to_angles(direction),
            ]
        ),
        real_cm,
        camera_matrix,
        zero_distortion,
    )
    reprojection_error = np.linalg.norm(predicted_pixels - pixels, axis=1)
    predicted_cm, ray_axis_distance, depth = closest_axis_coordinate(
        rays,
        translation,
        direction,
    )
    position_error = predicted_cm - real_cm

    summaries = []
    for position in sorted(grouped):
        group_mask = real_cm == position
        group_reprojection = reprojection_error[group_mask]
        group_position_error = position_error[group_mask]
        group_distance = ray_axis_distance[group_mask]
        group_predicted = predicted_cm[group_mask]
        stats = filter_stats[position]
        summaries.append(
            {
                "real_cm": position,
                **stats,
                "geometric_reprojection_rmse_px": float(
                    np.sqrt(np.mean(group_reprojection**2))
                ),
                "geometric_reprojection_median_px": float(
                    np.median(group_reprojection)
                ),
                "geometric_reprojection_max_px": float(
                    np.max(group_reprojection)
                ),
                "predicted_cm_median": float(np.median(group_predicted)),
                "position_bias_cm": float(np.mean(group_position_error)),
                "position_rmse_cm": float(
                    np.sqrt(np.mean(group_position_error**2))
                ),
                "position_max_abs_error_cm": float(
                    np.max(np.abs(group_position_error))
                ),
                "ray_axis_distance_median_cm": float(
                    np.median(group_distance)
                ),
                "anomalous_fixed_line_position": False,
            }
        )

    group_reprojection_rmse = np.asarray(
        [item["geometric_reprojection_rmse_px"] for item in summaries]
    )
    group_position_rmse = np.asarray(
        [item["position_rmse_cm"] for item in summaries]
    )
    reprojection_limit = robust_limit(group_reprojection_rmse, 2.0)
    position_limit = robust_limit(group_position_rmse, 0.5)
    anomalous_positions = []
    for item in summaries:
        anomalous = (
            item["geometric_reprojection_rmse_px"] > reprojection_limit
            or item["position_rmse_cm"] > position_limit
        )
        item["anomalous_fixed_line_position"] = anomalous
        if anomalous:
            anomalous_positions.append(item["real_cm"])

    overall = {
        "retained_sample_count": len(retained_samples),
        "rejected_sample_count": len(raw_samples) - len(retained_samples),
        "geometric_reprojection_rmse_px": float(
            np.sqrt(np.mean(reprojection_error**2))
        ),
        "geometric_reprojection_max_px": float(
            np.max(reprojection_error)
        ),
        "position_rmse_cm": float(
            np.sqrt(np.mean(position_error**2))
        ),
        "position_max_abs_error_cm": float(
            np.max(np.abs(position_error))
        ),
        "position_rmse_mm": float(
            10.0 * np.sqrt(np.mean(position_error**2))
        ),
        "position_max_abs_error_mm": float(
            10.0 * np.max(np.abs(position_error))
        ),
        "minimum_depth_cm": float(np.min(depth)),
        "anomalous_position_thresholds": {
            "geometric_reprojection_rmse_px": reprojection_limit,
            "position_rmse_cm": position_limit,
        },
        "anomalous_positions_cm": anomalous_positions,
        "fixed_line_model_accepted": len(anomalous_positions) == 0,
    }

    output = {
        "schema_version": 1,
        "model_type": "undistorted_ray_to_3d_pipe_axis_closest_point",
        "units": {
            "translation_T": "cm",
            "axis_coordinate": "cm",
            "runtime_secondary_output": "mm",
        },
        "coordinate_definition": {
            "zero": "physical pipe centre",
            "positive": "from centre toward image-left",
            "negative": "from centre toward image-right",
            "input_image_coordinates": "raw distorted 640x640 pixels",
            "fit_image_coordinates": (
                "undistorted 640x640 pixels with P=camera_matrix"
            ),
            "zero_is_image_centre": False,
        },
        "camera": {
            "image_size": [640, 640],
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": distortion.tolist(),
            "calibration_rms_px": camera_data.get("total_rms_pixels"),
            "runtime_undistortion": (
                "iterative inverse Brown-Conrady on detected centre pixel"
            ),
        },
        "pipe_axis_camera_coordinates": {
            "equation": "P_cm(s) = T_cm + s_cm * r",
            "T_cm": translation.tolist(),
            "r": direction.tolist(),
            "r_norm": float(np.linalg.norm(direction)),
            "zero_point_undistorted_pixel": zero_pixel.tolist(),
            "plus_one_cm_undistorted_pixel": positive_pixel.tolist(),
        },
        "fit": {
            "initialization": "homogeneous 1D-PnP from undistorted rays",
            "refinement": (
                "custom Huber robust LM in undistorted pixel space"
            ),
            "processing_order": (
                "undistort every sample, per-position MAD rejection, "
                "global T/r fit"
            ),
            "optimizer": optimizer,
            "mad_z_threshold": MAD_Z_THRESHOLD,
            "huber_delta_px": HUBER_DELTA_PX,
        },
        "per_position": summaries,
        "overall": overall,
    }
    OUTPUT_JSON.write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    fields = [
        "real_cm",
        "raw_count",
        "kept_count",
        "rejected_count",
        "geometric_reprojection_rmse_px",
        "geometric_reprojection_median_px",
        "geometric_reprojection_max_px",
        "predicted_cm_median",
        "position_bias_cm",
        "position_rmse_cm",
        "position_max_abs_error_cm",
        "ray_axis_distance_median_cm",
        "anomalous_fixed_line_position",
    ]
    with OUTPUT_CSV.open("w", encoding="utf-8", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=fields)
        writer.writeheader()
        for item in summaries:
            writer.writerow({field: item[field] for field in fields})

    print("Geometry position model completed.")
    print("T_cm:", translation)
    print("r:", direction)
    for item in summaries:
        print(
            "%+5.1f cm kept=%3d rejected=%2d reproj=%.3f px "
            "position_rmse=%.3f cm max=%.3f cm anomaly=%s"
            % (
                item["real_cm"],
                item["kept_count"],
                item["rejected_count"],
                item["geometric_reprojection_rmse_px"],
                item["position_rmse_cm"],
                item["position_max_abs_error_cm"],
                item["anomalous_fixed_line_position"],
            )
        )
    print("Overall reprojection RMSE px:", overall["geometric_reprojection_rmse_px"])
    print("Overall reprojection max px:", overall["geometric_reprojection_max_px"])
    print("Overall position RMSE cm:", overall["position_rmse_cm"])
    print("Overall position max error cm:", overall["position_max_abs_error_cm"])
    print("Anomalous positions cm:", anomalous_positions)
    print("Saved:", OUTPUT_JSON)
    print("Saved:", OUTPUT_CSV)


if __name__ == "__main__":
    main()

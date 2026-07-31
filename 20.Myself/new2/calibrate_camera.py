#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibrate the K230 camera from 640x640 chessboard images.

Expected input names:
    camera_calibration_640/calib_001.jpg
    camera_calibration_640/calib_002.jpg
    ...

The chessboard has 11x8 inner corners and a 15 mm square size. This is a
desktop OpenCV script; copy or mount the K230 SD-card image directory first.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


BOARD_COLUMNS = 11
BOARD_ROWS = 8
SQUARE_SIZE_MM = 15.0
BOARD_SIZE = (BOARD_COLUMNS, BOARD_ROWS)


def parse_args() -> argparse.Namespace:
    default_directory = Path(__file__).resolve().parent.parent / (
        "camera_calibration_640"
    )
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate a camera from calib_*.jpg chessboard images "
            "(11x8 inner corners, 15 mm squares)."
        )
    )
    parser.add_argument(
        "image_dir",
        nargs="?",
        type=Path,
        default=default_directory,
        help=(
            "directory containing calib_*.jpg "
            "(default: ../camera_calibration_640 relative to this script)"
        ),
    )
    return parser.parse_args()


def make_object_points() -> np.ndarray:
    points = np.zeros(
        (BOARD_COLUMNS * BOARD_ROWS, 3),
        dtype=np.float32,
    )
    points[:, :2] = (
        np.mgrid[0:BOARD_COLUMNS, 0:BOARD_ROWS]
        .T.reshape(-1, 2)
        .astype(np.float32)
    )
    points[:, :2] *= SQUARE_SIZE_MM
    return points


def save_corner_result(
    source: np.ndarray,
    source_path: Path,
    found: bool,
    corners: np.ndarray | None,
) -> Path:
    result = source.copy()
    if found and corners is not None:
        cv2.drawChessboardCorners(
            result,
            BOARD_SIZE,
            corners,
            True,
        )
        label = "FOUND 11x8"
        color = (0, 255, 0)
    else:
        label = "NOT FOUND 11x8"
        color = (0, 0, 255)

    cv2.rectangle(result, (0, 0), (250, 36), (0, 0, 0), -1)
    cv2.putText(
        result,
        label,
        (8, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )
    output_path = source_path.parent / (
        "corners_" + source_path.name
    )
    if not cv2.imwrite(str(output_path), result):
        raise OSError("could not write corner result: " + str(output_path))
    return output_path


def per_image_reprojection_error(
    object_points: np.ndarray,
    detected_points: np.ndarray,
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
) -> float:
    projected_points, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion_coefficients,
    )
    delta = detected_points.reshape(-1, 2) - projected_points.reshape(
        -1,
        2,
    )
    squared_pixel_distance = np.sum(delta * delta, axis=1)
    return float(math.sqrt(float(np.mean(squared_pixel_distance))))


def main() -> None:
    args = parse_args()
    image_directory = args.image_dir.expanduser().resolve()
    if not image_directory.is_dir():
        raise FileNotFoundError(
            "image directory does not exist: " + str(image_directory)
        )

    image_paths = sorted(image_directory.glob("calib_*.jpg"))
    if not image_paths:
        raise FileNotFoundError(
            "no calib_*.jpg images found in " + str(image_directory)
        )

    object_template = make_object_points()
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    accepted_paths: list[Path] = []
    image_results: list[dict[str, object]] = []
    image_size: tuple[int, int] | None = None

    find_flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )

    print("Image directory:", image_directory)
    print(
        "Chessboard:",
        "%dx%d inner corners," % BOARD_SIZE,
        SQUARE_SIZE_MM,
        "mm squares",
    )

    for path in image_paths:
        source = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if source is None:
            image_results.append(
                {
                    "image": path.name,
                    "corners_found": False,
                    "error": "OpenCV could not read the image",
                }
            )
            print("[READ FAILED]", path.name)
            continue

        current_size = (source.shape[1], source.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            result_path = save_corner_result(
                source,
                path,
                False,
                None,
            )
            image_results.append(
                {
                    "image": path.name,
                    "corners_found": False,
                    "corner_result": result_path.name,
                    "error": (
                        "image size %s differs from expected %s"
                        % (current_size, image_size)
                    ),
                }
            )
            print("[SIZE MISMATCH]", path.name, current_size)
            continue

        gray = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray,
            BOARD_SIZE,
            flags=find_flags,
        )
        result_path = save_corner_result(
            source,
            path,
            bool(found),
            corners if found else None,
        )

        entry: dict[str, object] = {
            "image": path.name,
            "corners_found": bool(found),
            "corner_result": result_path.name,
        }
        image_results.append(entry)

        if found:
            object_points.append(object_template.copy())
            image_points.append(
                np.asarray(corners, dtype=np.float32)
            )
            accepted_paths.append(path)
            print("[FOUND]", path.name)
        else:
            print("[NOT FOUND]", path.name)

    if image_size is None:
        raise RuntimeError("none of the input images could be read")
    if len(object_points) < 3:
        raise RuntimeError(
            "only %d usable images; at least 3 are required"
            % len(object_points)
        )

    (
        total_rms,
        camera_matrix,
        distortion_coefficients,
        rotation_vectors,
        translation_vectors,
    ) = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    error_by_name: dict[str, float] = {}
    for index, path in enumerate(accepted_paths):
        error_by_name[path.name] = per_image_reprojection_error(
            object_points[index],
            image_points[index],
            rotation_vectors[index],
            translation_vectors[index],
            camera_matrix,
            distortion_coefficients,
        )

    for entry in image_results:
        image_name = str(entry["image"])
        if image_name in error_by_name:
            entry["reprojection_error_pixels"] = error_by_name[
                image_name
            ]

    successful_errors = list(error_by_name.values())
    result = {
        "schema_version": 1,
        "image_directory": str(image_directory),
        "image_size": {
            "width": image_size[0],
            "height": image_size[1],
        },
        "board": {
            "inner_corners_columns": BOARD_COLUMNS,
            "inner_corners_rows": BOARD_ROWS,
            "square_size_mm": SQUARE_SIZE_MM,
        },
        "input_image_count": len(image_paths),
        "successful_image_count": len(object_points),
        "total_rms_pixels": float(total_rms),
        "mean_per_image_reprojection_error_pixels": float(
            np.mean(successful_errors)
        ),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion_coefficients.reshape(
            -1
        ).tolist(),
        "images": image_results,
    }

    output_path = image_directory / "camera_calibration.json"
    output_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print("\nCalibration completed")
    print("Usable images:", len(object_points), "/", len(image_paths))
    print("Image size:", image_size)
    print("Total RMS (pixels):", float(total_rms))
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(distortion_coefficients.reshape(-1))
    print("Per-image reprojection error (pixels):")
    for path in accepted_paths:
        print(
            "  %s: %.6f"
            % (path.name, error_by_name[path.name])
        )
    print("Saved:", output_path)
    print(
        "Corner result images:",
        image_directory / "corners_calib_*.jpg",
    )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Web-assisted intrinsic calibration for the USB camera.

Stop main.py before running this program because a camera can normally only be
opened by one process at a time. Open http://<raspberry-pi-ip>:8081 afterwards.
"""

import argparse
from io import BytesIO
from pathlib import Path
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify, send_file


BOARD_INNER_CORNERS = (9, 6)
MIN_SAMPLES = 10
OUTPUT_FILE = Path(__file__).resolve().with_name("camera_calibration.npz")


class CalibrationSession:
    def __init__(self, camera_index: int, rotate_180: bool = False) -> None:
        self.capture = cv2.VideoCapture(camera_index)
        self.rotate_180 = rotate_180
        self.lock = threading.RLock()
        self.frame = None
        self.corners = None
        self.image_points = []
        self.image_size = None
        self.active = False
        self.rms = None
        self.message = "点击“开始标定”"
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self) -> None:
        while self.running:
            ok, frame = self.capture.read()
            if not ok:
                with self.lock:
                    self.message = "无法读取摄像头，请确认主程序已关闭"
                time.sleep(0.1)
                continue
            if self.rotate_180:
                frame = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray,
                BOARD_INNER_CORNERS,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK,
            )
            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                )
            with self.lock:
                self.frame = frame
                self.corners = corners if found else None
                self.image_size = (gray.shape[1], gray.shape[0])

    def start(self) -> dict:
        with self.lock:
            self.image_points.clear()
            self.rms = None
            self.active = True
            self.message = "标定已开始：移动棋盘格后逐次点击“采集当前画面”"
            return self.status()

    def capture_sample(self) -> tuple[dict, int]:
        with self.lock:
            if not self.active:
                return {"error": "请先开始标定"}, 400
            if self.corners is None:
                self.message = "未检测到完整棋盘格，请调整距离、角度或光照"
                return self.status(), 422
            current = self.corners.copy()
            if self.image_points:
                # Reject almost identical poses; diverse views produce a useful calibration.
                displacement = np.mean(np.linalg.norm(current - self.image_points[-1], axis=2))
                if displacement < 8.0:
                    self.message = "画面与上一次过于接近，请移动或倾斜棋盘格"
                    return self.status(), 409
            self.image_points.append(current)
            self.message = f"已采集 {len(self.image_points)} 张"
            return self.status(), 200

    def finish(self) -> tuple[dict, int]:
        with self.lock:
            if len(self.image_points) < MIN_SAMPLES:
                self.message = f"至少需要 {MIN_SAMPLES} 张，当前 {len(self.image_points)} 张"
                return self.status(), 422
            image_points = [item.copy() for item in self.image_points]
            image_size = self.image_size

        object_template = np.zeros(
            (BOARD_INNER_CORNERS[0] * BOARD_INNER_CORNERS[1], 3), np.float32
        )
        object_template[:, :2] = np.mgrid[
            0 : BOARD_INNER_CORNERS[0], 0 : BOARD_INNER_CORNERS[1]
        ].T.reshape(-1, 2)
        object_points = [object_template.copy() for _ in image_points]
        rms, matrix, distortion, _, _ = cv2.calibrateCamera(
            object_points, image_points, image_size, None, None
        )
        np.savez(
            str(OUTPUT_FILE),
            camera_matrix=matrix,
            dist_coeffs=distortion,
            image_width=image_size[0],
            image_height=image_size[1],
            rms=rms,
            created_at=time.time(),
        )
        with self.lock:
            self.rms = float(rms)
            self.active = False
            self.message = f"标定完成，重投影 RMS={rms:.3f}，文件已保存"
            return self.status(), 200

    def status(self) -> dict:
        return {
            "active": self.active,
            "board_found": self.corners is not None,
            "samples": len(self.image_points),
            "minimum_samples": MIN_SAMPLES,
            "finished": OUTPUT_FILE.exists() and self.rms is not None,
            "rms": self.rms,
            "message": self.message,
        }

    def jpeg(self) -> bytes:
        with self.lock:
            if self.frame is None:
                frame = np.full((480, 640, 3), 35, np.uint8)
            else:
                frame = self.frame.copy()
                corners = None if self.corners is None else self.corners.copy()
                if corners is not None:
                    cv2.drawChessboardCorners(frame, BOARD_INNER_CORNERS, corners, True)
            text = f"samples {len(self.image_points)}/{MIN_SAMPLES}"
        cv2.putText(frame, text, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        return encoded.tobytes() if ok else b""


def calibration_target_png() -> bytes:
    square = 100
    columns = BOARD_INNER_CORNERS[0] + 1
    rows = BOARD_INNER_CORNERS[1] + 1
    margin = square
    image = np.full(
        (rows * square + 2 * margin, columns * square + 2 * margin), 255, np.uint8
    )
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2 == 0:
                y0 = margin + row * square
                x0 = margin + column * square
                image[y0 : y0 + square, x0 : x0 + square] = 0
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("无法生成棋盘格")
    return encoded.tobytes()


PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>USB 摄像头标定</title><style>
body{font-family:system-ui;margin:0;background:#111827;color:#e5e7eb}main{max-width:960px;margin:auto;padding:24px}
.card{background:#1f2937;border-radius:14px;padding:18px;margin:14px 0}img{max-width:100%;border-radius:10px}
button,a.button{border:0;border-radius:9px;padding:11px 16px;margin:5px;background:#2563eb;color:white;text-decoration:none;display:inline-block;font-size:15px}
button:disabled{opacity:.4}.ok{color:#4ade80}.warn{color:#fbbf24}li{margin:7px 0}code{background:#111827;padding:2px 5px}
</style></head><body><main><h1>USB 摄像头标定</h1>
<section class="card"><h2>操作步骤</h2><ol>
<li>停止 <code>main.py</code>，打印或在另一块平整屏幕上全屏显示下方棋盘格，保持棋盘格平整。</li>
<li>点击“开始标定”，让棋盘格占画面的 40%～80%，看到彩色角点后点击“采集当前画面”。</li>
<li>分别放在画面中央、四角和边缘，并改变远近与倾斜角度，至少采集 10 张；不要连续采集相同姿态。</li>
<li>点击“完成标定”。下载文件并放到 <code>main.py</code> 同一目录（本机运行时也会自动保存到那里）。</li>
</ol><a class="button" href="/target.png" download="camera_calibration_board.png">下载/打开标定棋盘格</a></section>
<section class="card"><img src="/video_feed" alt="标定实时画面"><p id="status">读取状态...</p>
<button onclick="action('/api/start')">开始标定</button><button id="capture" onclick="action('/api/capture')">采集当前画面</button>
<button onclick="action('/api/finish')">完成标定</button><a id="download" class="button" href="/download" style="display:none">下载标定文件</a></section>
<script>async function action(url){const r=await fetch(url,{method:'POST'});const p=await r.json();render(p)}
function render(p){const s=document.getElementById('status');s.textContent=`${p.message}｜棋盘格：${p.board_found?'已识别':'未识别'}｜样本：${p.samples}/${p.minimum_samples}`;s.className=p.board_found?'ok':'warn';document.getElementById('capture').disabled=!p.active||!p.board_found;document.getElementById('download').style.display=p.finished?'inline-block':'none'}
async function poll(){try{render(await(await fetch('/api/status')).json())}catch(e){}setTimeout(poll,500)}poll()</script>
</main></body></html>"""


def create_app(session: CalibrationSession) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        return PAGE

    @app.get("/video_feed")
    def video_feed():
        def frames():
            while True:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + session.jpeg() + b"\r\n"
                time.sleep(0.05)
        return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/target.png")
    def target():
        return send_file(BytesIO(calibration_target_png()), mimetype="image/png")

    @app.get("/api/status")
    def status():
        with session.lock:
            return jsonify(session.status())

    @app.post("/api/start")
    def start():
        return jsonify(session.start())

    @app.post("/api/capture")
    def capture():
        payload, status_code = session.capture_sample()
        return jsonify(payload), status_code

    @app.post("/api/finish")
    def finish():
        payload, status_code = session.finish()
        return jsonify(payload), status_code

    @app.get("/download")
    def download():
        if not OUTPUT_FILE.exists():
            return "尚未生成标定文件", 404
        return send_file(OUTPUT_FILE, as_attachment=True, download_name=OUTPUT_FILE.name)

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="USB camera web calibration")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--rotate-180", action="store_true")
    args = parser.parse_args()
    session = CalibrationSession(args.camera, args.rotate_180)
    print(f"请打开 http://<树莓派IP>:{args.port} 进行标定")
    create_app(session).run(host=args.host, port=args.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()

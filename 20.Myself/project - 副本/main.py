#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
* @par Copyright (C): 2010-2020, hunan CLB Tech
* @file         Basic_movement
* @version      V2.0
* @details
* @par History

@author: zhulin
"""
from LOBOROBOT import LOBOROBOT  # 载入机器人库
import RPi.GPIO as GPIO
import os
import sys
import time
import math
import socket
import threading
from typing import Optional
from car_control_web import CarControlPanel
from vision_line_follow import CameraLineFollower
# Python3 移除了 reload 和 setdefaultencoding，无需设置
# 注：Python3 默认编码为 utf-8，无需手动设置
ARM_KEYS = ("axis1", "axis2", "axis3")
TRANSLATE_KEYS = ("x", "y", "z")
BUTTON_DIRECTIONS = ("negative", "positive")
USB_CAMERA_INDEX = 0  # OpenCV index 0 normally maps to /dev/video0 on Raspberry Pi.
# 视觉循迹相机安装参数。高度可按实车测量后直接修改。
CAMERA_HEIGHT_CM = 2.0
CAMERA_FORWARD_CM = 18.0
CAMERA_DOWN_ANGLE_DEG = 30.0
CAMERA_CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "camera_calibration.npz"
)
CAMERA_LINE_CRUISE_SPEED = 0.5
# 四轮差速底盘的小角度转向灵敏度；越小越不容易在中心附近过冲。
CAMERA_SMALL_TURN_GAIN = 0.35
# 横向回线比例增益；采用连续比例控制，避免直线两侧反复触发。
CAMERA_CROSS_TRACK_GAIN = 0.3
# 根据实拍黑带宽度设置：640 像素画面约 32 像素，更细的暗线按地缝处理。
CAMERA_MIN_LINE_WIDTH_RATIO = 0.050
zone = 0.2

# 四路循迹传感器（BCM 编号），顺序为从车身左侧到右侧。
# 传感器输出低电平表示检测到黑线，高电平表示在线外。
LINE_SENSOR_PINS = (12, 20, 21, 26)
# 以车体中心为 0，四个探头的近似横向位置（cm）。中间两个间距
# 1 cm、最外侧两个间距约 6.5 cm，因此不能按等间距处理。
LINE_OUTER_POSITION_CM = 3.25
LINE_SENSOR_POSITIONS_CM = (-3.25, -0.5, 0.5, 3.25)
# 传感器横排安装在四轮底盘旋转中心前方 18 cm。
LINE_SENSOR_FORWARD_CM = 18.0
LINE_BASE_SPEED = 1.00
LINE_CORNER_SPEED = 0.35
LINE_SEARCH_SPEED = 0.18
LINE_OUTER_TURN = 0.60
# 如果实车转向与黑线方向相反，只需改成 -1.0。
LINE_STEERING_SIGN = 1.0
HYBRID_MAX_SPEED = 1.00
HYBRID_SENSOR_ONLY_SPEED = 1.00
STATUS_PRINT_INTERVAL_SECONDS = 0.2
CONTROL_LOOP_INTERVAL_SECONDS = 0.005


def setup_line_sensors() -> None:
    """将四个循迹传感器配置为 BCM 输入引脚。"""
    for pin in LINE_SENSOR_PINS:
        GPIO.setup(pin, GPIO.IN)


def read_line_sensors() -> tuple:
    """按从左到右的顺序返回四路原始电平。"""
    return tuple(GPIO.input(pin) for pin in LINE_SENSOR_PINS)


def line_follow_command(sensor_values: tuple, last_error: float) -> tuple:
    """根据四路电平计算 (前进量, 转向量, 当前偏差)。"""
    # 输入为 0 时在线上，因此需要取反得到“检测到线”的布尔值。
    detected_positions = [
        position
        for value, position in zip(sensor_values, LINE_SENSOR_POSITIONS_CM)
        if value == 0
    ]

    if not detected_positions:
        # 四路均为 1：沿上一次偏差方向低速寻找黑线；启动时未见线则停车。
        if last_error == 0:
            return 0.0, 0.0, last_error
        turn = LINE_OUTER_TURN if last_error > 0 else -LINE_OUTER_TURN
        return LINE_SEARCH_SPEED, LINE_STEERING_SIGN * turn, last_error

    # 1001 时两个内侧探头的位置互相抵消，误差为 0，车辆直行。
    error = sum(detected_positions) / len(detected_positions)

    # 传感器位于旋转中心前方 18 cm，用几何方向角 atan(横向偏差/18)
    # 控制四轮差速转向。外侧探头对应最大修正量；1011 / 1101 只有
    # 一个内侧探头压线时，修正量自然约为外侧的六分之一。
    heading_error = math.atan2(error, LINE_SENSOR_FORWARD_CM)
    outer_heading_error = math.atan2(
        LINE_OUTER_POSITION_CM, LINE_SENSOR_FORWARD_CM
    )
    turn = LINE_OUTER_TURN * max(
        -1.0, min(1.0, heading_error / outer_heading_error)
    )

    turn *= LINE_STEERING_SIGN
    # 外侧纠偏时降低前进速度；回到内侧探头后恢复速度并小幅微调。
    correction_ratio = min(1.0, abs(turn) / LINE_OUTER_TURN)
    forward = max(
        LINE_SEARCH_SPEED,
        LINE_BASE_SPEED
        - (LINE_BASE_SPEED - LINE_CORNER_SPEED) * correction_ratio,
    )
    return forward, turn, error


def hybrid_follow_command(
    sensor_values: tuple, last_error: float, vision_command
) -> tuple:
    """融合近距离循迹传感器与摄像头前瞻结果。"""
    sensor_forward, sensor_turn, current_error = line_follow_command(
        sensor_values, last_error
    )
    detected_count = sum(value == 0 for value in sensor_values)
    sensor_valid = detected_count > 0
    vision_valid = bool(
        vision_command is not None
        and vision_command.found
        and vision_command.confidence >= 0.35
    )

    if sensor_valid and vision_valid:
        outer_detected = sensor_values[0] == 0 or sensor_values[3] == 0
        perfect_center = sensor_values == (1, 0, 0, 1)
        one_inner_detected = (
            not outer_detected
            and (sensor_values[1] == 0) != (sensor_values[2] == 0)
        )

        if outer_detected:
            sensor_weight = 0.88
            source = "外侧传感器强纠偏"
        elif one_inner_detected:
            sensor_weight = 0.65
            source = "内侧传感器微调"
        elif perfect_center:
            # 1001 表示近场位置已经准确居中。视觉只用于提前感知前方弯道，
            # 避免摄像头被地缝或纹理误导后主导车辆转向。
            sensor_weight = 0.75
            source = "传感器居中，视觉小幅前瞻"
        else:
            sensor_weight = 0.50
            source = "多点融合"

        if sensor_turn * vision_command.turn < 0:
            # 两种来源相反时，车底附近的传感器负责避免冲出黑线。
            sensor_weight = max(sensor_weight, 0.95 if outer_detected else 0.80)
            source += "（方向冲突，近场优先）"

        vision_weight = (1.0 - sensor_weight) * vision_command.confidence
        sensor_weight = 1.0 - vision_weight
        turn = (
            sensor_weight * sensor_turn
            + vision_weight * vision_command.turn
        )
        # 两种来源都可靠且道路笔直时允许满速；弯道根据融合转向量
        # 连续降速，外侧传感器自身的低速限制仍由 sensor_forward 保留。
        turn_speed_factor = max(0.35, 1.0 - 0.75 * abs(turn))
        forward = max(
            LINE_SEARCH_SPEED,
            min(sensor_forward, HYBRID_MAX_SPEED) * turn_speed_factor,
        )
        return forward, max(-1.0, min(1.0, turn)), current_error, source

    if sensor_valid:
        return (
            min(sensor_forward, HYBRID_SENSOR_ONLY_SPEED),
            sensor_turn,
            current_error,
            "视觉丢失，传感器接管",
        )

    if vision_valid:
        return (
            min(vision_command.forward, HYBRID_MAX_SPEED),
            vision_command.turn,
            current_error,
            "传感器丢线，视觉接管",
        )

    # 两种来源都没有可靠地看到黑线时，只允许短时低速搜索。
    if vision_command is not None and vision_command.forward > 0:
        return (
            min(LINE_SEARCH_SPEED, vision_command.forward),
            vision_command.turn,
            current_error,
            "双源丢线，视觉短时搜索",
        )
    if sensor_forward > 0:
        return (
            min(LINE_SEARCH_SPEED, sensor_forward),
            sensor_turn,
            current_error,
            "双源丢线，按传感器历史搜索",
        )
    return 0.0, 0.0, current_error, "双源丢线，停车"


def run_vision_monitor(vision_follower, panel, stop_event) -> None:
    """在后台处理摄像头，避免阻塞传感器和电机控制循环。"""
    while not stop_event.is_set():
        if not vision_follower.ready():
            stop_event.wait(0.005)
            continue
        try:
            frame = panel.video.read_frame()
            vision_follower.update(frame)
            if vision_follower.debug_frame is not None:
                panel.video.set_preview_frame(vision_follower.debug_frame)
        except Exception as exc:
            # 后台视觉异常不能拖死底盘控制，复位命令可让视觉/融合
            # 模式立即停车，同时不影响传感器循迹主循环。
            print(f"视觉后台处理异常：{exc}")
            vision_follower.reset()
            stop_event.wait(0.1)


def button_state(buttons: dict, group: str, name: str, direction: str) -> bool:
    try:
        return bool(buttons[group][name][direction])
    except (KeyError, TypeError):
        return False


def copy_arm_buttons(buttons: dict) -> dict:
    return {
        group: {
            name: {
                direction: button_state(buttons, group, name, direction)
                for direction in BUTTON_DIRECTIONS
            }
            for name in names
        }
        for group, names in (("axis", ARM_KEYS), ("translation", TRANSLATE_KEYS))
    }


def button_released(previous: Optional[dict], current: dict, group: str, name: str, direction: str) -> bool:
    if previous is None:
        return False
    return (
        button_state(previous, group, name, direction)
        and not button_state(current, group, name, direction)
    )


def translation_delta(previous: Optional[dict], current: dict, step: float) -> tuple:
    """Return the XYZ displacement generated by this release event."""
    delta = []
    for axis in TRANSLATE_KEYS:
        positive = button_released(
            previous, current, "translation", axis, "positive"
        )
        negative = button_released(
            previous, current, "translation", axis, "negative"
        )
        delta.append(step * (int(positive) - int(negative)))
    return tuple(delta)


def enable_video_rotation_180(panel) -> bool:
    """Enable 180-degree rotation with both old and new panel versions."""
    video = getattr(panel, "video", None)
    if video is None:
        return False

    if hasattr(video, "rotate_180"):
        video.rotate_180 = True
        return True

    normalize_frame = getattr(video, "_normalize_frame", None)
    if not callable(normalize_frame):
        return False

    def normalize_and_rotate(cv2, frame):
        return cv2.flip(normalize_frame(cv2, frame), -1)

    video._normalize_frame = normalize_and_rotate
    return True


def get_lan_ip() -> Optional[str]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


def pressed_summary(buttons: dict) -> str:
    pressed = []

    for name in ARM_KEYS:
        pair = buttons["axis"][name]
        if pair["negative"]:
            pressed.append(f"{name}-")
        if pair["positive"]:
            pressed.append(f"{name}+")

    for name in TRANSLATE_KEYS:
        pair = buttons["translation"][name]
        if pair["negative"]:
            pressed.append(f"{name}-")
        if pair["positive"]:
            pressed.append(f"{name}+")

    return ",".join(pressed) if pressed else "none"

class InverseKinematicsSolver:
    """
    基于给定方程组的位置逆解求解器：
    z = a*sin(alpha) - b*sin(theta)
    y = (a*cos(alpha) + b*cos(theta) + c) * sin(beta)
    x = (a*cos(alpha) + b*cos(theta) + c) * cos(beta)
    """

    def __init__(self, a: float, b: float, c: float):
        """
        初始化机械参数
        :param a: 连杆长度 a (需大于 0)
        :param b: 连杆长度 b (需大于 0)
        :param c: 偏置长度 c
        """
        if a <= 0 or b <= 0:
            raise ValueError("参数 a 和 b 必须为正数")
        self.a = a
        self.b = b
        self.c = c

    def solve(self, x: float, y: float, z: float, sign: int = 1) -> tuple:
        """
        根据目标位置计算关节角度 (弧度制)
        :param x: 目标 X 坐标
        :param y: 目标 Y 坐标
        :param z: 目标 Z 坐标
        :param sign: 构型选择，+1 或 -1（对应 theta 解的两种分支）
        :return: (alpha, beta, theta) 弧度，范围约 [-pi, pi]
        :raises ValueError: 当目标位置不可达或处于奇异点时
        """
        if sign not in (1, -1):
            raise ValueError("sign 必须为 +1 或 -1")

        # 1. 计算 R 和 beta
        R = math.hypot(x, y)  # sqrt(x^2 + y^2)
        beta = math.atan2(y, x)

        # 如果 R 接近 0，beta 可以任意设定，这里保持 atan2 结果（通常为 0）
        # 但后续 R_c = R - c 可能为负，不影响计算，只要不在奇异点即可。

        R_c = R - self.c

        # 2. 检查奇异点：当 R_c = 0 且 z = 0 时，方程组退化为特殊情形
        # 此时需要额外判断是否 a == b，否则无解
        if abs(R_c) < 1e-12 and abs(z) < 1e-12:
            if abs(self.a - self.b) < 1e-12:
                # a == b 时，有无穷多组解，这里给出一个特解 (alpha=0, theta=pi)
                # 同时警告用户
                print("Warning: Singular configuration detected (a=b). "
                      "Returning special solution: alpha=0, theta=pi")
                alpha = 0.0
                theta = math.pi
                return alpha, beta, theta
            else:
                raise ValueError("Target position is unreachable: singularity "
                                 f"with R={R}, z={z}, but a != b.")

        # 3. 计算 theta
        # M = (R_c^2 + z^2 + b^2 - a^2) / (2 * b)
        M = (R_c * R_c + z * z + self.b * self.b - self.a * self.a) / (2.0 * self.b)

        # S = sqrt(R_c^2 + z^2)
        S = math.hypot(R_c, z)

        # 检查数值有效范围 |M / S| <= 1
        ratio = M / S
        if ratio > 1.0:
            if ratio - 1.0 < 1e-10:
                ratio = 1.0
            else:
                raise ValueError(f"Target unreachable: M/S = {ratio} > 1. "
                                 "Check target coordinates or mechanical parameters.")
        elif ratio < -1.0:
            if -1.0 - ratio < 1e-10:
                ratio = -1.0
            else:
                raise ValueError(f"Target unreachable: M/S = {ratio} < -1. "
                                 "Check target coordinates or mechanical parameters.")

        # theta = atan2(-z, R_c) + sign * arccos(ratio)
        theta = math.atan2(-z, R_c) + sign * math.acos(ratio)

        # 防止浮点误差导致 theta 轻微越界，归一化到 [-pi, pi]
        theta = math.atan2(math.sin(theta), math.cos(theta))

        # 4. 计算 alpha
        # alpha = atan2(z + b*sin(theta), R_c - b*cos(theta))
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        alpha = math.atan2(z + self.b * sin_theta, R_c - self.b * cos_theta)

        # 归一化到 [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        return alpha, beta, theta

    def solve_degrees(self, x: float, y: float, z: float, sign: int = 1) -> tuple:
        """
        与 solve 相同，但返回角度制（度）
        """
        alpha, beta, theta = self.solve(x, y, z, sign)
        return math.degrees(alpha), math.degrees(beta), math.degrees(theta)

solver = InverseKinematicsSolver(a=8.0, b=8.0, c=6.0)


def angle_trans_xyz(alpha_degrees, beta_degrees, theta_degrees):
    """Convert joint angles in degrees to the Cartesian end position."""
    alpha = math.radians(alpha_degrees)
    beta = math.radians(beta_degrees)
    theta = math.radians(theta_degrees)
    R_check = solver.a * math.cos(alpha) + solver.b * math.cos(theta) + solver.c
    x = R_check * math.cos(beta)
    y = R_check * math.sin(beta)
    z = solver.a * math.sin(alpha) - solver.b * math.sin(theta)
    return x, y, z


def solve_target_angles(position) -> Optional[tuple]:
    try:
        return solver.solve_degrees(*position, sign=1)
    except ValueError as e:
        print(
            "平移目标不可达: "
            f"X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}; {e}"
        )
        return None

if __name__ == "__main__":
    clbrobot = LOBOROBOT()  # 实例化机器人对象
    setup_line_sensors()
    panel = CarControlPanel(
        host="0.0.0.0",
        port=8080,
        video_source=USB_CAMERA_INDEX,
    )
    if not enable_video_rotation_180(panel):
        print("警告：当前网页控制模块不支持摄像头旋转")
    panel.start()
    vision_stop_event = threading.Event()
    vision_thread = None
    try:
        vision_follower = CameraLineFollower(
            camera_height_cm=CAMERA_HEIGHT_CM,
            camera_forward_cm=CAMERA_FORWARD_CM,
            camera_down_angle_deg=CAMERA_DOWN_ANGLE_DEG,
            calibration_path=CAMERA_CALIBRATION_FILE,
            cruise_speed=CAMERA_LINE_CRUISE_SPEED,
            min_line_width_ratio=CAMERA_MIN_LINE_WIDTH_RATIO,
            small_turn_gain=CAMERA_SMALL_TURN_GAIN,
            cross_track_gain=CAMERA_CROSS_TRACK_GAIN,
        )
        calibration_state = (
            "已加载相机标定" if vision_follower.calibration_loaded else "未找到标定文件，使用归一化图像坐标"
        )
        print(f"视觉循迹就绪：{calibration_state}")
    except (ImportError, RuntimeError) as exc:
        vision_follower = None
        print(f"视觉循迹不可用：{exc}")
    if vision_follower is not None:
        vision_thread = threading.Thread(
            target=run_vision_monitor,
            args=(vision_follower, panel, vision_stop_event),
            name="vision-line-monitor",
            daemon=True,
        )
        vision_thread.start()
    gripper=0
    current_angles = list(solver.solve_degrees(0,10,0, sign=1))
    last_arm_buttons = None
    last_line_error = 0.0
    last_arm_angles = None
    last_servo_targets = None
    last_gripper_angle = None
    current_pos1 = None
    next_status_print_at = 0.0
    next_control_loop_at = time.monotonic()
    try:
        while True:
            forward=0
            direction=0
            status_message = None
            info = panel.get_operation_info()
            # OpenCV 在独立线程运行；传感器/电机主循环只读取最新结果，
            # 不再被摄像头采集和图像处理阻塞。
            vision_command = (
                vision_follower.last_command
                if vision_follower is not None
                else None
            )
            if info.get("connected") is False:
                last_arm_buttons = None
                last_line_error = 0.0
                status_message = "等待网页/手机连接..."
            elif info.get("connected") is True and all(
                key in info for key in ("work_mode", "drive", "arm", "gyro")
            ):
                work_mode = info["work_mode"]
                drive = info["drive"]
                arm = info["arm"]
                gyro = info["gyro"]
                step=arm["step"]
                gripper=arm["gripper"]
                if work_mode == "manual":
                    last_line_error = 0.0
                    if drive["mode"] == "joystick":
                        x = drive["x"]
                        y = drive["y"]
                        direction=x
                        forward=y

                    elif drive["mode"] == "gyro":
                        x = drive["x"]
                        y = drive["y"]
                        if x>zone:
                            direction=(x-zone)/(1-zone)
                            direction=min(max(direction,0),1)
                        elif x<-zone:
                            direction=(x+zone)/(1-zone)
                            direction=min(max(direction,-1),0)
                        if y>zone:
                            forward=(y-zone)/(1-zone)
                            forward=min(max(forward,0),1)
                        elif y<-zone:
                            forward=(y+zone)/(1-zone)
                            forward=min(max(forward,-1),0)
                    
                    current_arm_buttons = copy_arm_buttons(arm.get("buttons", {}))
                    if arm["mode"] == "axis":
                        if button_released(last_arm_buttons, current_arm_buttons, "axis", "axis1", "positive"):
                            print("轴1 +")
                            current_angles[0]+=step
                        if button_released(last_arm_buttons, current_arm_buttons, "axis", "axis1", "negative"):
                            print("轴1 -")
                            current_angles[0]-=step
                        if button_released(last_arm_buttons, current_arm_buttons, "axis", "axis2", "positive"):
                            print("轴2 +")
                            current_angles[1]+=step
                        if button_released(last_arm_buttons, current_arm_buttons, "axis", "axis2", "negative"):
                            print("轴2 -")
                            current_angles[1]-=step
                        if button_released(last_arm_buttons, current_arm_buttons, "axis", "axis3", "positive"):
                            print("轴3 +")
                            current_angles[2]+=step
                        if button_released(last_arm_buttons, current_arm_buttons, "axis", "axis3", "negative"):
                            print("轴3 -")
                            current_angles[2]-=step

                    elif arm["mode"] == "translate":
                        delta = translation_delta(
                            last_arm_buttons, current_arm_buttons, step
                        )
                        if any(delta):
                            current_pos = angle_trans_xyz(*current_angles)
                            target_pos = tuple(
                                coordinate + offset
                                for coordinate, offset in zip(current_pos, delta)
                            )
                            target_angles = solve_target_angles(target_pos)
                            if target_angles is not None:
                                current_angles = list(target_angles)
                                moved_axes = ", ".join(
                                    f"{axis.upper()} {offset:+g}"
                                    for axis, offset in zip(TRANSLATE_KEYS, delta)
                                    if offset
                                )
                                print(
                                    f"平移 {moved_axes} -> "
                                    f"X={target_pos[0]:.2f}, "
                                    f"Y={target_pos[1]:.2f}, "
                                    f"Z={target_pos[2]:.2f}"
                                )
                    last_arm_buttons = current_arm_buttons

                elif work_mode == "auto1":
                    line_values = read_line_sensors()
                    forward, direction, last_line_error = line_follow_command(
                        line_values, last_line_error
                    )
                    status_message = (
                        f"传感器循迹: {line_values} "
                        f"前进={forward:.2f} 转向={direction:.2f}"
                    )

                elif work_mode == "auto2":
                    last_line_error = 0.0
                    if vision_follower is None:
                        forward = 0.0
                        direction = 0.0
                        status_message = "视觉循迹不可用：请安装 OpenCV 和 NumPy"
                    else:
                        direction = vision_command.turn
                        forward = vision_command.forward
                        status_message = (
                            f"视觉循迹: {vision_command.message} "
                            f"置信度={vision_command.confidence:.2f} "
                            f"前进={forward:.2f} 转向={direction:+.2f}"
                        )

                elif work_mode == "auto3":
                    line_values = read_line_sensors()
                    (
                        forward,
                        direction,
                        last_line_error,
                        fusion_state,
                    ) = hybrid_follow_command(
                        line_values, last_line_error, vision_command
                    )
                    vision_confidence = (
                        vision_command.confidence
                        if vision_command is not None
                        else 0.0
                    )
                    status_message = (
                        f"融合循迹: {fusion_state} 传感器={line_values} "
                        f"视觉置信度={vision_confidence:.2f} "
                        f"前进={forward:.2f} 转向={direction:+.2f}"
                    )

                
            '''
            clbrobot.t_up(50, 3)      # 机器人前进
            clbrobot.t_stop(1)        # 机器人停止
            clbrobot.t_down(50, 3)    # 机器人后退
            clbrobot.t_stop(1)        # 机器人停止
            clbrobot.turnLeft(50, 3)  # 机器人左转
            clbrobot.t_stop(1)        # 机器人停止
            clbrobot.turnRight(50, 3) # 机器人右转
            clbrobot.t_stop(1)        # 机器人停止
            '''
            now = time.monotonic()
            if status_message is not None and now >= next_status_print_at:
                print(status_message, end="\r")
                next_status_print_at = now + STATUS_PRINT_INTERVAL_SECONDS

            arm_angles = tuple(current_angles)
            servo_targets = (
                current_angles[1] - 22,
                180 - current_angles[0] - 78,
                180 - current_angles[2] - 65,
            )
            if servo_targets != last_servo_targets:
                clbrobot.set_dir_angle(servo_targets[0])
                clbrobot.set_a_angle(servo_targets[1])
                clbrobot.set_b_angle(servo_targets[2])
                last_servo_targets = servo_targets

            if arm_angles != last_arm_angles:
                current_pos1 = list(angle_trans_xyz(*arm_angles))
                panel.set_robot_state(angles=arm_angles, position=current_pos1)
                last_arm_angles = arm_angles
            # t_move 的参数顺序为（转向量，前进量）。
            clbrobot.t_move(direction,forward)
            gripper_angle = 90 if gripper == 0 else 45 if gripper == 1 else None
            if gripper_angle is not None and gripper_angle != last_gripper_angle:
                clbrobot.set_clip_angle(gripper_angle)
                last_gripper_angle = gripper_angle

            # Keep the hardware/control loop close to 200 Hz while yielding CPU
            # time to the vision and web threads. Re-align after an overrun so a
            # slow iteration does not create an accumulating timing backlog.
            next_control_loop_at += CONTROL_LOOP_INTERVAL_SECONDS
            loop_wait = next_control_loop_at - time.monotonic()
            if loop_wait > 0.0:
                time.sleep(loop_wait)
            else:
                next_control_loop_at = time.monotonic()
            '''
            for i in range(8):
                move(0,10,i+1)
                time.sleep(0.05)
            time.sleep(0.5)
            '''
    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        # 视觉处理或摄像头异常时也必须立即停止四轮底盘。
        try:
            clbrobot.t_stop(0)
        finally:
            vision_stop_event.set()
            if vision_thread is not None:
                vision_thread.join(timeout=1.0)
            GPIO.cleanup()
            panel.stop()

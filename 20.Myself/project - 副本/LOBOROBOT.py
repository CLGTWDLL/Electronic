#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import math
try:
    # Python3 中 smbus 可能需要安装 smbus2，兼容原 smbus 接口
    import smbus2 as smbus
except ImportError:
    import smbus  # 备用（部分树莓派系统仍保留 smbus）
import RPi.GPIO as GPIO

Dir = [
    'forward',
    'backward',
]

class PCA9685:
    # Registers/etc.
    __SUBADR1            = 0x02
    __SUBADR2            = 0x03
    __SUBADR3            = 0x04
    __MODE1              = 0x00
    __PRESCALE           = 0xFE
    __LED0_ON_L          = 0x06
    __LED0_ON_H          = 0x07
    __LED0_OFF_L         = 0x08
    __LED0_OFF_H         = 0x09
    __ALLLED_ON_L        = 0xFA
    __ALLLED_ON_H        = 0xFB
    __ALLLED_OFF_L       = 0xFC
    __ALLLED_OFF_H       = 0xFD

    def __init__(self, address, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        if self.debug:
            print("Reseting PCA9685")  # Python3 print 改为函数
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        """Writes an 8-bit value to the specified register/address"""
        self.bus.write_byte_data(self.address, reg, value)
        if self.debug:
            # Python3 格式化字符串更规范
            print(f"I2C: Write 0x{value:02X} to register 0x{reg:02X}")

    def read(self, reg):
        """Read an unsigned byte from the I2C device"""
        result = self.bus.read_byte_data(self.address, reg)
        if self.debug:
            print(f"I2C: Device 0x{self.address:02X} returned 0x{result & 0xFF:02X} from reg 0x{reg:02X}")
        return result

    def setPWMFreq(self, freq):
        """Sets the PWM frequency"""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if self.debug:
            print(f"Setting PWM frequency to {freq} Hz")
            print(f"Estimated pre-scale: {prescaleval}")
        prescale = math.floor(prescaleval + 0.5)
        if self.debug:
            print(f"Final pre-scale: {prescale}")

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10        # sleep
        self.write(self.__MODE1, newmode)        # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        """Sets a single PWM channel"""
        self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4*channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4*channel, off >> 8)
        if self.debug:
            print(f"channel: {channel}  LED_ON: {on} LED_OFF: {off}")

    def setDutycycle(self, channel, pulse):
        self.setPWM(channel, 0, int(pulse * (4096 / 100)))

    def setLevel(self, channel, value):
        if value == 1:
            self.setPWM(channel, 0, 4095)
        else:
            self.setPWM(channel, 0, 0)

# 控制机器人库
class LOBOROBOT():
    def __init__(self):
        self.PWMA = 18
        self.AIN1 = 22
        self.AIN2 = 27

        self.PWMB = 23
        self.BIN1 = 25
        self.BIN2 = 24

        self.dir = 0
        self.axisa = 1
        self.axisb = 2
        self.clip = 3
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWMA, GPIO.OUT)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.PWMB, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        self.pwm0 = GPIO.PWM(18, 50)
        self.pwm0.start(0)
        self.pwm1 = GPIO.PWM(23, 50)
        self.pwm1.start(0)


    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if motor == 0:
            self.pwm0.ChangeDutyCycle(speed)
            if index == Dir[0]:
                GPIO.output(self.AIN1, 0)
                GPIO.output(self.AIN2, 1)
            else:
                GPIO.output(self.AIN1, 1)
                GPIO.output(self.AIN2, 0)
        elif motor == 1:
            self.pwm1.ChangeDutyCycle(speed)
            if index == Dir[0]:
                GPIO.output(self.BIN1, 1)
                GPIO.output(self.BIN2, 0)
            else:
                GPIO.output(self.BIN1, 0)
                GPIO.output(self.BIN2, 1)

    def MotorStop(self, motor):
        if motor == 0:
            self.pwm0.ChangeDutyCycle(0)
        elif motor == 1:
            self.pwm1.ChangeDutyCycle(0)

    # 前进
    def t_up(self, speed, t_time):
        self.MotorRun(0, 'forward', abs(speed)*100)
        self.MotorRun(1, 'forward', abs(speed)*100)
        time.sleep(t_time)

    # 后退
    def t_down(self, speed, t_time):
        self.MotorRun(0, 'backward', abs(speed)*100)
        self.MotorRun(1, 'backward', abs(speed)*100)
        time.sleep(t_time)

    # 左转
    def turnLeft(self, speed, t_time):
        self.MotorRun(0, 'backward', abs(speed)*100)
        self.MotorRun(1, 'forward', abs(speed)*100)
        time.sleep(t_time)
    
    # 右转
    def turnRight(self, speed, t_time):
        self.MotorRun(0, 'forward', abs(speed)*100)
        self.MotorRun(1, 'backward', abs(speed)*100)
        time.sleep(t_time)

    # 停止
    def t_stop(self, t_time):
        self.MotorStop(0)
        self.MotorStop(1)
        time.sleep(t_time)

    def transfer(self,a):
        if a>0:
            return 'forward'
        elif a<0:
            return 'backward'

    def t_move(self, turn, forward):
        """四轮差速移动：参数依次为转向量、前进量。"""
        motor0 = forward + turn
        motor1 = forward - turn
        # 摇杆斜向输入时和可能超过 1，等比例缩放以免 PWM 超过 100%。
        scale = max(1.0, abs(motor0), abs(motor1))
        motor0 /= scale
        motor1 /= scale
        self.MotorRun(0, self.transfer(motor0), abs(motor0)*100)
        self.MotorRun(1, self.transfer(motor1), abs(motor1)*100)
        
    # 辅助功能，使设置舵机脉冲宽度更简单。
    def set_servo_pulse(self, channel, pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        self.pwm.setPWM(channel, 0, pulse)

    # 设置舵机角度函数  
    def set_servo_angle(self, channel, angle):
        angle = 4096 * ((angle * 11) + 500) / 20000
        self.pwm.setPWM(channel, 0, int(angle))

    def set_dir_angle(self, angle):
        self.set_servo_angle(self.dir, angle)

    def set_a_angle(self, angle):
        self.set_servo_angle(self.axisa, angle)
    
    def set_b_angle(self, angle):
        self.set_servo_angle(self.axisb, angle)

    def set_clip_angle(self, angle):
        self.set_servo_angle(self.clip, angle)

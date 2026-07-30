# ESP32-CAM 手机图传与 MP4 录像

这是一个可直接用 Arduino IDE 打开的独立工程。ESP32-CAM 自己建立 Wi-Fi 热点，不需要路由器。

## Arduino IDE

打开 `ESP-CAM.ino`，开发板选择 `ESP32 Arduino → AI Thinker ESP32-CAM`，串口波特率为 `115200`。

## 烧录

1. 烧录前将 GPIO0 接 GND，然后按一下 RST。
2. 在 Arduino IDE 中选择正确的 COM 端口并上传。
3. 上传完成后断开 GPIO0 与 GND，再按一下 RST。

## 手机使用

1. 手机连接热点 `ESP32-CAM`。
2. 密码为 `12345678`。
3. 提示无互联网时选择保持连接。
4. 浏览器打开 `http://192.168.4.1`。
5. 点击开始显示并录制，点击结束生成 MP4。

供电建议使用稳定的 5V、至少 1A 电源。

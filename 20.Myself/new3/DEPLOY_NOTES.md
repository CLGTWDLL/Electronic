# steel_ball_416_motion_v3 K230 部署说明

本说明只陈述本目录的实际 ONNX 导出、nncase 2.9.0 编译和 K230 simulator 执行结果；没有重新训练，也没有 K230 相机实机测试。

## 文件和已验证接口

- KModel：`steel_ball_416_motion_v3_nncase_2.9.0.kmodel`，用于 K230 上的单类别 Steel_Ball YOLOv5 原始预测推理。
- KModel SHA-256：`0724923a59430d6e9af346f335bd78fc4bf3fc540d5882cd6aebe9df826e4184`，大小：2289312 bytes。
- 源权重：`/home/lhh/Projects/k230-yolo/yolov5/runs/train/steel_ball_416_motion_v3/weights/best.pt`，SHA-256：`54f3350b680e2e87025eb85b46593e482480a119d9022f1bd93fea5114d922b0`。
- ONNX：`/home/lhh/Projects/k230-yolo/deploy/steel_ball_416_motion_v3/steel_ball_416_motion_v3.onnx`，SHA-256：`cc1872a152df13a20e4d483068a59220ab603276cf19de90a9ad36463ca285ca`。
- 编译：K230，`nncase 2.9.0 + nncase-kpu 2.9.0`；PTQ、KLD、activation `uint8`、weight `uint8`，校准图 100 张。
- ONNX 已通过 checker。输入 `images`；输出 `output0`。
- simulator 实际加载并执行成功：输入 `[1, 3, 416, 416]`，输出 `[1, 10647, 6]`，运行时喂入 input 的 dtype 为 `float32`，输出 dtype 为 `float32`；两者均为有限值。

## 输入预处理（已由转换和 simulator 实测）

- 固定 shape：`[1, 3, 416, 416]`；布局 **NCHW**，即 `tensor[0, c, y, x]`，不是 HWC。
- 外部运行时输入：**float32**。`options.preprocess=False`，而且 simulator 实际接收的是 float32 NCHW 数据。量化 activation/weight 为 uint8 仅是编译内部量化配置，**不代表运行时输入可改为 uint8**。
- 通道顺序：**RGB**。若相机/API 给出 BGR，转 NCHW 前必须交换 R/B。
- 输入像素范围：原始 RGB 通道 `P∈[0,255]`；模型张量范围为 `[0,1]`。必须除以 255：

```text
T[0, 0, y, x] = R_letterbox(y, x) / 255.0
T[0, 1, y, x] = G_letterbox(y, x) / 255.0
T[0, 2, y, x] = B_letterbox(y, x) / 255.0
T = (P / 255.0 - [0,0,0]) / [1,1,1]
```

- 采用保持比例 letterbox，不是拉伸 resize。对于输入原图宽高 `W0,H0`：

```text
r = min(416/W0, 416/H0)
Rw = round(W0*r); Rh = round(H0*r)
pad_left = (416-Rw)//2; pad_top = (416-Rh)//2
pad_right = 416-Rw-pad_left; pad_bottom = 416-Rh-pad_top
```

先 RGB 双线性插值到 `Rw×Rh`，再粘贴至 `416×416`；填充值为 RGB `(114,114,114)`。奇数 padding 时右/下侧多一像素。

## 输出和后处理（由导出模型接口与 YOLOv5 Detect 导出路径确认）

- 输出 shape：`[1,10647,6]`，dtype `float32`；本次 simulator 输出范围为 `[-3.265625, 1378.0]`。该范围仅适用于验证图 `old_181.jpg`，不能当作所有图的固定范围。
- `10647 = 3×(52×52 + 26×26 + 13×13)`。不得把后处理循环上限写死为旧模型的 `25200`；必须读取/校验实际输出 shape 的第二维，并按每行 6 个字段处理。
- `output0[0,i,:]`：`[cx, cy, w, h, objectness, Steel_Ball_probability]`。前四项是 **416 letterbox 像素坐标**的 **xywh**，未归一化、不是 xyxy；输出尚未 NMS。

```text
score = objectness * Steel_Ball_probability
x1 = cx-w/2; y1 = cy-h/2; x2 = cx+w/2; y2 = cy+h/2
```

对 score 过滤后，在 `xyxy` 上 NMS。参考阈值为 `confidence=0.25`、`NMS IoU=0.45`，继承上一版部署配置；它们不在模型内，也未对本次 K230 相机画面调优，须按实机效果复核。

## 从 416 坐标还原到 640×640 相机画面

若且仅若推理源是完整、未裁剪/旋转/镜像的 `640×640` 相机帧，则 `r=416/640=13/20`，四边 padding 都为 0。因此精确还原为：

```text
cx_640 = cx_416 * 640/416 = cx_416 * 20/13
cy_640 = cy_416 * 20/13
w_640  = w_416  * 20/13
h_640  = h_416  * 20/13
x1_640 = x1_416 * 20/13; y1_640 = y1_416 * 20/13
x2_640 = x2_416 * 20/13; y2_640 = y2_416 * 20/13
```

对任意非 640×640 或有裁剪的原图，必须使用预处理时实际保存的 `r,pad_left,pad_top`：`x0=(x-pad_left)/r`、`y0=(y-pad_top)/r`、`w0=w/r`、`h0=h/r`。当前工作区未包含相机脚本或实机帧链路，裁剪/旋转/镜像、色彩源格式和显示坐标变换均**无法从模型或本次转换确认**，必须实机验证。

## 现有 640 相机脚本迁移

本工作区没有现有相机脚本文件，故不能给出具体文件名/行号。迁移时**必须修改**：

1. KModel 路径和 SHA-256，输入 shape/缓冲区/AI2D 目标改为 `[1,3,416,416]`。
2. 输出 shape、DMA 大小、断言和遍历逻辑改为实际 `[1,10647,6]`；不得写死 `25200`。
3. 所有 640 相关 letterbox 与反变换，按本说明的 416 规则和逐帧 `r/pad` 修改；完整 640×640 帧才可用 `20/13`。

迁移时**不得修改或省略**：RGB 通道契约、NCHW float32、`/255.0` 归一化、114 letterbox 填充、`score=objectness×class_probability`、xywh→xyxy、NMS 需求和对实际输出 shape 的检查。尤其不要因为量化权重为 uint8 而省略模型要求的 float32 `/255.0` 输入。

## 验证边界

K230 simulator 已对校准图 `old_181.jpg` 真实执行并确认输入和输出均有限。它不等同于 K230 板端/相机实机验证；板端 runtime 兼容性、相机颜色格式、AI2D 与 Pillow 插值的一致性、DMA/cache 同步、显示映射及阈值效果均待实机确认。

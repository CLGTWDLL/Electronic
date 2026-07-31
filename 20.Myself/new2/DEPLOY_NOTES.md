# steel_ball_416 K230 部署说明

本文只根据以下现有文件整理，不包含重新训练、重新转换或相机实测结果：

- `best.onnx`（已检查输入/输出张量）
- `convert_k230_nncase.py`
- `model_metadata.json`
- `nncase_simulator_report.json`
- 原 640 部署配置 `../../../../deploy/steel_ball_full_v2_180/k230_deploy_config.json`

## 已确认的模型与运行时接口

- KModel：`steel_ball_416_nncase_2.9.0.kmodel`
- KModel SHA-256：`37199fe17c88332f06155e5fe46db766c6de568adc1548f0f1d1003290bf7835`
- 目标：K230，`nncase 2.9.0 + nncase-kpu 2.9.0`。
- 输入名：`images`；固定 shape：`[1, 3, 416, 416]`。
- **KModel 运行时输入 dtype：`float32`**。虽然编译采用 uint8/uint8 PTQ，但编译脚本设置 `options.preprocess = False`，并向 simulator 传入 `float32` NCHW 张量；因此外部调用方仍须提供下述 float32 输入。
- 布局：**NCHW**，即 `tensor[0, c, y, x]`；不是 HWC。
- 通道顺序：**RGB**。模型前的图像排列必须为 R、G、B；若相机/API 输出 BGR，必须在转 NCHW 前交换 R 与 B。

### 输入预处理（已确认）

设原始图像宽高为 `W0`、`H0`，其每个 RGB 像素通道值为 `P_rgb`（范围 0 到 255）。转换脚本使用 Pillow 的双线性插值并执行：

```text
r  = min(416 / W0, 416 / H0)
Rw = round(W0 * r)
Rh = round(H0 * r)
pad_left = (416 - Rw) // 2
pad_top  = (416 - Rh) // 2
pad_right  = 416 - Rw - pad_left
pad_bottom = 416 - Rh - pad_top
```

先将 RGB 图像双线性缩放为 `Rw × Rh`，再贴到 `416 × 416` 画布；四周填充值为 **RGB `(114, 114, 114)`**。这是保持比例的 letterbox，不是拉伸 resize。脚本中的 `round()` 与 `//` 是实际采用的取整规则；当总 padding 为奇数时，右/下侧会多一个像素。

最终张量公式为：

```text
T[0, 0, y, x] = R_letterbox(y, x) / 255.0
T[0, 1, y, x] = G_letterbox(y, x) / 255.0
T[0, 2, y, x] = B_letterbox(y, x) / 255.0
```

也就是先 RGB、再 HWC→NCHW、转 `float32`、再除以 `255.0`。没有额外的 mean 减法、std 除法、`[-1, 1]` 映射或 KModel 内置预处理；以标准归一化记法即 `T = (P / 255.0 - [0,0,0]) / [1,1,1]`。

> 待相机实测：实际相机帧的原始通道顺序（RGB/BGR/YUV）、像素步长、裁剪/缩放 API 的取整与双线性实现。无论相机 API 如何命名，送入模型的最终张量必须满足上述 RGB/NCHW/float32 约定。

## 输出与后处理（已确认）

- 输出名：`output0`
- shape：`[1, 10647, 6]`
- dtype：`float32`
- 格式：YOLOv5 原始预测；尚未执行 NMS。

每个 `output0[0, i, :]` 的六个字段依次为：

| 下标 | 字段 | 含义 |
|---:|---|---|
| 0 | `cx` | 预测框中心 x，位于 `416 × 416` letterbox 图坐标系 |
| 1 | `cy` | 预测框中心 y，位于 `416 × 416` letterbox 图坐标系 |
| 2 | `w` | 预测框宽度，单位为该 letterbox 图的像素坐标 |
| 3 | `h` | 预测框高度，单位为该 letterbox 图的像素坐标 |
| 4 | `objectness` | 目标存在置信度 |
| 5 | `Steel_Ball_probability` | 唯一类别 `Steel_Ball` 的类别概率 |

这些坐标不是归一化的 `0..1` 坐标，也不是 `xyxy`。YOLOv5 导出 Detect 头已按 stride `8/16/32` 解码为输入图像尺度的 `xywh`。`10647 = 3 × (52×52 + 26×26 + 13×13)`；对应三组 anchor、三种 stride。

后处理必须将：

```text
score = objectness * Steel_Ball_probability
x1 = cx - w / 2
y1 = cy - h / 2
x2 = cx + w / 2
y2 = cy + h / 2
```

然后以 `score` 过滤并对 `xyxy` 执行 NMS。部署配置记录的参考阈值为 `confidence_threshold = 0.25`、`nms_iou_threshold = 0.45`；它们是现有部署配置值，不是本次相机端性能实测的最优阈值。

### 从 416 letterbox 坐标还原到原图

使用本页预处理阶段实际得到的 `Rw`、`Rh`、`pad_left`、`pad_top`，定义：

```text
sx = Rw / W0
sy = Rh / H0
```

对模型输出的 `cx, cy, w, h`，几何还原公式为：

```text
cx0 = (cx - pad_left) / sx
cy0 = (cy - pad_top)  / sy
w0  = w / sx
h0  = h / sy

x1_0 = (x1 - pad_left) / sx
y1_0 = (y1 - pad_top)  / sy
x2_0 = (x2 - pad_left) / sx
y2_0 = (y2 - pad_top)  / sy
```

其中带 `_0` 的坐标属于原始 `W0 × H0` 图像；绘制前应按显示画布的实际尺寸进行裁剪、取整或再次缩放。

> 待相机实测：相机显示缓冲与推理源图是否同尺寸、是否存在裁剪/旋转/镜像、以及最终绘制 API 的边界裁剪和取整规则。若这些路径与推理输入图不同，上式之后仍需加相机显示路径的坐标变换。

## 从原 640 相机脚本迁移时必须修改的代码项

工作区未包含原 640 相机脚本本体，因此无法提供具体文件名或行号。以下是根据原 640 部署配置（输入 `[1,3,640,640]`、输出 `[1,25200,6]`）与本 416 接口确认的必改常量/逻辑：

1. **模型文件路径与完整性校验**：改为 `steel_ball_416_nncase_2.9.0.kmodel`，并将期望 SHA-256 改为本文开头所列值。
2. **推理输入尺寸**：所有模型宽高、AI2D/resize 目标、输入 tensor/buffer 尺寸、shape 检查由 `640` 改为 `416`；固定 shape 为 `[1,3,416,416]`，batch 仍为 `1`。
3. **预处理数据契约**：保留外部预处理；将 letterbox 目标改为 `416×416`，填充 RGB 值保持 `114`，双线性缩放、RGB、HWC→NCHW、`float32`、`/255.0` 均不可删除或重复执行。
4. **颜色转换**：送入张量前必须得到 RGB。原 640 配置提示 OpenCV/CanMV 类 API 可能解码为 BGR；新脚本仍须在实际输入为 BGR 时转换为 RGB。相机当前输出格式本工作区无法确认，**待相机实测**。
5. **输出 tensor/buffer 与遍历上限**：由 `[1,25200,6]`、`25200` 改为 `[1,10647,6]`、`10647`；所有输出 DMA buffer 大小、循环上限、断言和内存复制长度都必须同步修改。
6. **后处理输入**：保持六字段 `cx,cy,w,h,objectness,class_probability` 和 `score = objectness × class_probability`；NMS 输入为转换后的 `xyxy`。不要把该输出当作已 NMS 的检测列表。
7. **坐标反变换**：原脚本中基于 640 的比例、padding、反 letterbox 与显示映射必须改为本页 `416` 公式，并保存每帧的 `Rw/Rh/pad_left/pad_top`，不能只用一个固定的 640 比例。
8. **运行时接口断言**：加载后应校验 KModel 输入 `[1,3,416,416]`、输出 `[1,10647,6]` 与外部 `float32` 输入约定；模型内部 PTQ 不代表相机端可改为 uint8 张量。
9. **阈值配置**：可保留现有配置的 `0.25/0.45` 作为起点，但其在相机画面上的效果**待相机实测**；未经实测不应声称是最优值。

## 已完成与未覆盖的验证边界

已完成的是 nncase K230 simulator 验证：输入 `[1,3,416,416]`、输出 `[1,10647,6]`，输出为有限 `float32` 数值。它验证了 KModel 可被该 simulator 加载和运行，不等同于相机实机验证。

以下事项均为**待相机实测**：K230 板端 runtime 版本兼容性、相机原始颜色格式与帧尺寸、AI2D/图像 API 是否精确复现此处 letterbox、NCHW tensor 写入、DMA/缓存同步、端侧 NMS 数值一致性、显示坐标还原与阈值效果。

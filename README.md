## K230 钢球视觉检测与端侧部署

基于 YOLOv5 完成钢球检测，并实现
PyTorch → ONNX → nncase → KModel 的 K230 端侧部署。

- 数据标注与 ROI 处理
- K230 实时推理
- AI2D 预处理对齐
- INT8 量化问题排查
- 视觉坐标与机器人坐标映射

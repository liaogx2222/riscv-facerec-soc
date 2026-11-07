# 第三方组件与数据声明

| 组件 | 位置 | 许可/说明 |
|---|---|---|
| EEMBC CoreMark v1.0 | `sw/coremark/` | 遵循 EEMBC 原始许可，保留文件头版权声明；仅用于基准测试 |
| ChokePoint 人脸数据集 | 未包含 | 原始数据仅用于训练/评测，**不随仓库分发**；仓库内 `rtl/accel/mem/framesrc.mem` 为 `tools/pack_frames.py` 生成的合成占位图，非真人图像 |
| Pango PDS 2022.2 / 2025.2 | 未包含 | 商业 EDA 工具，需自行获取并配置 hostid 绑定许可证 |
| RISC-V 指令集规范 | 参考文献 | 仅用于实现依据，未包含规范文本 |

模型权重（`rtl/accel/mem/cnn_*.mem`）为本项目自行训练、量化后的产物，依 MIT 许可随仓库提供。

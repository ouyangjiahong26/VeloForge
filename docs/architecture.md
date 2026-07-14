# VeloForge Architecture

## 模块

- `veloforge-core` — 协议解码、单位转换、姿态/重力/静止/积分；**不依赖** Python、UI、OS BLE。
- `veloforge-device` — 基于 `btleplug + Tokio` 的跨平台 BLE 后端；面向 Windows/macOS/Linux。
- `veloforge-py` — PyO3 adapter，把 `SensorEngine` 暴露为 Python 私有扩展 `veloforge._core`。
- `veloforge` (Python) — PySide6 桌面应用、replay 会话、CSV 持久化。

## 线程模型

- BLE 通知由 `veloforge-device` 内部 Tokio 任务处理，主机单调时间戳与通知 payload 一起入队。
- 协议解码和运动管线在 `veloforge-core` 中以同步方法执行，可被独立测试。
- Python 端使用 `QTimer`（20–30 Hz）调用 `SensorEngine::poll(max_events)`，从有界事件队列拉取结果并刷新界面。
- 队列溢出被记录为事件，UI 收到后显示质量警告，不静默丢弃。

## 配置

`veloforge-core::ProcessingConfig` 描述：

- `nominal_rate_hz`：标称采样率（200）。
- `gravity_mps2`：当地重力（默认 9.7915）。
- `body_to_bar_axis`：体坐标到杠铃轴的投影向量。
- `stationary_accel_threshold_mps2`：加速度范数阈值。
- `stationary_gyro_threshold_rads`：角速度范数阈值。
- `stationary_enter_ms`、`stationary_exit_ms`：静止状态机迟滞。

任何与协议字节、寄存器、单位相关的常量必须出现在 `veloforge-core::wit` 命名空间下，**禁止**分散到 UI 或测试代码中。

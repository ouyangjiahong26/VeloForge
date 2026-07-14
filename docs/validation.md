# Validation

## 自动化

- `cargo fmt --check`
- `cargo clippy --workspace --all-targets -- -D warnings`
- `cargo test --workspace`
- `maturin develop -m crates/veloforge-py/Cargo.toml`
- `pytest tests/python`（使用 Qt offscreen 平台）
- `tests/fixtures/protocol/` 提供基于协议公式构造的原始 20 字节帧。

## 人工

- BLE 扫描能在 Windows/macOS/Linux 列出名称含 `WT` 的设备。
- 连接 BWT901BLE5.0 后能订阅 FFE4 通知、写入 FFE9 命令。
- 设置回传率为 200 Hz 后观察到稳定 200 Hz 通知；`0x0A/0x0B` 映射需要现场确认。
- 连续 30 分钟采集不丢帧或丢帧被明确标记。
- 加计/磁场校准命令能写入并得到设备响应。
- 静止→运动→静止的速度积分在断电后回到零附近；漂移修正在线校正。
- 平均向心速度、峰值速度、重复次数与线性编码器/视频参考比较。

## 已知未验证

- 当前没有连接真实传感器，因此：
  - 200 Hz 实际映射未在硬件上确认。
  - 速度精度未与独立真值比较。
  - macOS CoreBluetooth 设备 ID 行为需要在 Apple 平台机器上验证。
  - Linux BlueZ 行为需要在目标发行版上验证。

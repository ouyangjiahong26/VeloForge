# VeloForge

VeloForge 是维特智能 BWT901BLE 蓝牙 IMU 的跨平台桌面应用，使用 Rust 核心加 PySide6 界面完成运动采集、姿态估计和杠铃速度检测。

## 状态

当前是初始骨架，已包含：

- Rust workspace：`veloforge-core`（协议与算法）、`veloforge-device`（btleplug 跨平台 BLE）、`veloforge-py`（PyO3 adapter）。
- Python 包 `veloforge`，集成 PySide6 桌面界面、replay 会话和 CSV 持久化。
- 测试与参考仓库快照 `references/WitBluetooth_BWT901BLE5_0/`。

尚未完成真实硬件验证；当前所有 BLE 与算法路径都通过离线 replay 验证。

## 架构

```text
BLE 字节
  └── veloforge-device  btleplug + Tokio
        └── veloforge-core  协议解码、单位转换、姿态/重力/静止/积分
              └── veloforge-py  PyO3 adapter
                    └── veloforge (Python)
                          ├── PySide6 界面
                          ├── pyqtgraph 实时曲线
                          ├── replay 离线会话
                          └── CSV 持久化
```

## 开发

```sh
# Python 3.12 隔离环境
py -3.12 -m venv .venv
. .venv/Scripts/activate
pip install -U pip maturin pytest

# 准备 Rust 与 Python 测试
maturin develop -m crates/veloforge-py/Cargo.toml
cargo test --workspace
pytest tests/python
```

详细计划见 `docs/architecture.md`、`docs/data-contract.md`、`docs/validation.md`。

## 硬件

测试需要维特智能 BWT901BLE5.0 蓝牙 IMU。连接前需要：

- Windows：系统蓝牙开启
- macOS：在 `Info.plist` 中允许蓝牙，并在首次运行时授权
- Linux：BlueZ ≥ 5.55

硬件验证条目见 `docs/validation.md`；当前在没有传感器时仅做 replay 验证。

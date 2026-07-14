# Data Contract

## 传感器输入

```rust
pub struct ImuSample {
    pub monotonic_ns: i64,        // 主机单调时间，避免墙钟漂移
    pub accel_g: [f64; 3],         // 原始 g 数值
    pub gyro_dps: [f64; 3],        // 原始 °/s
    pub euler_deg: Option<[f64; 3]>,
    pub quaternion_wxyz: Option<[f64; 4]>,
}
```

## 处理输出

```rust
pub struct ProcessedSample {
    pub monotonic_ns: i64,
    pub linear_accel_nav_mps2: [f64; 3],
    pub velocity_nav_mps: [f64; 3],
    pub bar_velocity_mps: f64,
    pub stationary: bool,
    pub phase: MotionPhase,
    pub quality: QualityFlags,
}
```

## 单位与约定

- 协议解码层只输出厂商单位（g、°/s、°、四元数）。
- 运动管线内部统一使用 SI：m/s²、m/s、m、rad/s。
- 四元数顺序固定为 `wxyz`；缺失四元数时回退到欧拉角（注意欧拉角万向锁）。
- 时间差始终来自 `monotonic_ns`；**不**假设固定采样率。

## 错误

- `DecodeError`：原始帧在缓冲中无法重新同步。
- `CommandError`：构造命令时寄存器号越界或参数越界。
- `PipelineError`：时间倒退、NaN 输入、四元数非单位化。
- 上述错误映射到 Python `ValueError` 或专用异常；Rust panic 不允许穿过 FFI。

# VeloForge

VeloForge 是维特智能 BWT901BLE 蓝牙 IMU 的跨平台桌面应用：在杠铃训练场景下采集 IMU 数据、估计姿态与重力影响、做 ZUPT 与积分，给出杠铃轴方向上的瞬时速度。

## Language

**ImuSample**:
单次 IMU 测量在同一瞬间包含加速度（g）、角速度（°/s）、姿态（欧拉角或四元数）。
_Avoid_: SensorReading, Frame

**ProcessedSample**:
管线从一次 `ImuSample` 解出的导航系下线性加速度、速度、静止判定、运动阶段与质量标志。
_Avoid_: OutputFrame, Result

**Recording**:
从一次真机采集固定下来的、可重放的数据集；与现场实验绑定，未来回灌到 `SensorEngine` 必须产出与采集时**一致**的 `ProcessedSample` 序列（按字段、字节级）。
_Avoid_: Log, Capture, Dataset

**Replay**:
把 `Recording` 内的数据按原顺序送回 `SensorEngine`，用于离线回归与人工检查；不依赖硬件、不重新计时。
_Avoid_: Playback, Simulate

**ReturnRate**:
BWT901BLE 上位机配置的"每秒通知包数"，协议层通过 `0x0A` 等寄存命令切换；经验值 200 Hz = `0x0A`。
_Avoid_: SampleRate, BandwidthOnly

**Calibration Lifecycle**:
原工程成对存在的协议序列：必须先 `UnlockRegister`，才能下发 `AppliedCalibration` / `StartFieldCalibration` / `SetReturnRate` 等；变更后通常需要 `SaveRegister`。跳过解锁会被设备忽略或拒绝。
_Avoid_: Configuration, Setup

# UR5 位姿数学测试台

## 目的

`Ur5PoseMathTestBench` 是 9.2 阶段的离线位姿数学测试台，用于验证遥操作位姿映射。它把 Unity world、XR origin、controller、`robot_base` 和 TCP 都表示为显式 `FramePose`，因此可以在不依赖真实场景和 Quest3 设备的情况下定位坐标系转换和四元数乘法方向问题。

## 坐标约定

- `ControllerWorldDelta`: controller 在 Unity world 下的原始位移。
- `ControllerXrOriginDelta`: 同一位移在 XR origin 下的表达，用于排查父级层级继承问题。
- `ControllerRobotBaseDelta`: controller 位移在 `robot_base` 下的表达；这是 TCP 平移映射唯一使用的输入。
- `MappedTcpRobotBaseDelta`: 缩放后的 TCP 位移，仍在 `robot_base` 下表达。
- `MappedTcpWorldDelta`: 映射后的 TCP 位移，转换回 Unity world。
- `MappedTcpRotationDelta`: 缩放后的 controller 相对旋转，按世界坐标左乘方式应用。
- `FinalTargetWorld`: 最终 TCP 目标位姿，位于 Unity world。

四元数合成固定采用世界坐标左乘：

```csharp
finalTcpRotation = mappedTcpRotationDelta * clutchTcpRotation;
```

这表示 controller 相对旋转先以世界坐标增量的形式作用，再得到离合起点 TCP 姿态之后的目标姿态。`RotationScale = 0` 是显式的仅平移锁定模式，会把 TCP 姿态保持在离合快照。

## 验证范围

EditMode 测试覆盖：

- `robot_base` 下 X/Y/Z 单轴 controller 平移。
- Rx/Ry/Rz 单轴 controller 旋转，确认不会引入 TCP 平移耦合。
- 右手仅平移模式，确认 TCP 旋转被锁定。
- 90 度单轴旋转的世界坐标左乘方向检查。
- controller delta、映射后 TCP delta 和最终目标 pose 的诊断输出字段。

9.2 阶段不需要 Build & Run 到 Quest3。设备验证应等到后续阶段把数学测试台接入运行时遥操作行为之后再进行。

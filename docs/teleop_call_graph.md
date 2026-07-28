# 9.1 Unity Teleop Call Graph Audit

> 阶段9.1只读审计产物。本文记录 Unity/Quest3/UR5 遥操作从 XR 输入、TcpTarget 命令、Unity IK 预览、UDP 影子遥测到可选 speedl 真机输出的真实调用链。本文不修改运行时代码。

## 审计基线

- 仓库：`/Users/imi-1/Desktop/project/ur5`
- 审计分支：`docs/teleop-call-chain-audit`
- 基线提交：`baa8698 Merge pull request #2 from wkcherish/fix/Refactor-control-logic`
- 主要范围：`Assets/Scripts/UR5/`
- 产物配套表：`docs/write_site_inventory.md`
- 只读原则：本审计只新增文档，不改 Unity 控制脚本、场景、Prefab 或 ProjectSettings。

## 当前默认控制链

```text
Ur5ControlBootstrap.Awake
  -> ResolveTcpTarget
  -> ConfigureTcpTargetSafety
     -> TcpTargetWorkspaceLimiter.constrainInLateUpdate = false
     -> TcpTargetCollisionGuard.preventObstaclePenetration = false
     -> TcpTargetWriteMonitor.enableDiagnostics = false
  -> DisableQuestPoseController / DisableLegacyVrTeleoperationControllers / DisableKeyboardTargetController
  -> ConfigureVelocityTeleop
     -> Ur5CartesianVelocityTeleopController
  -> ConfigureTcpTargetFollower
     -> Ur5TcpTargetFollower
     -> Ur5JointTrajectoryPlayer
     -> Ur5ArticulationJointController
  -> ConfigureQuestUdpShadowTelemetry
     -> Quest3UdpTeleopSender, optional and read-only
  -> ConfigureGripperController
     -> Quest3RobotiqGripperController
  -> ConfigureGraspAssist / Diagnostics / Recorder
```

当前默认手柄控制不是旧的 `Quest3TcpTargetController`。`Ur5ControlBootstrap.Awake` 在 `enableCartesianVelocityTeleop = true` 时会禁用旧 Quest pose 控制器、旧 VR 控制器和键盘 TcpTarget 控制器。保留下来的主写入者是 `Ur5CartesianVelocityTeleopController` 的 FixedUpdate 预览路径；抓取辅助、follower 安全释放和启动对齐是显式的例外写入者。

## 生命周期顺序

Unity 主线程中与控制相关的顺序可以按生命周期分层理解：

1. `Awake`
   - `Ur5ControlBootstrap` 成为 runtime owner，查找或创建 `TcpTarget`。
   - Bootstrap 挂载/配置 workspace limiter、collision guard、write monitor、velocity teleop、follower、trajectory player、gripper、diagnostics 和 recorder。
   - 默认禁用旧控制器，避免同一 `TcpTarget` 多 writer。
2. `Update`
   - `Ur5CartesianVelocityTeleopController` 读取 XR InputDevice 的右手位置、左手旋转/摇杆、Grip、A、X/Y 等按键，计算 clutch、precision 和原始速度状态。
   - `Quest3UdpTeleopSender` 可选发送原始 Quest tracking frame UDP JSON；它不走 XR Origin，也不连接真机。
   - `Quest3RobotiqGripperController` 读取右手 trigger 和 Grip deadman，更新夹爪目标开合量。
3. `FixedUpdate`
   - `Ur5CartesianVelocityTeleopController` 根据 clutch anchor 生成逻辑 `TcpTarget` pose，并在主路径中调用 `tcpPreviewTarget.SetPositionAndRotation(...)`。
   - 同一控制器将速度命令传给 `Ur5UrScriptSpeedlClient.SetCommand(...)`；真机输出仍受 `enableRealRobotOutput`、armed、超时和限幅保护。
   - `Ur5TcpTargetFollower` 读取 `TcpTarget` 与实际两指中心误差，生成一帧关节 waypoint。
   - `Ur5JointTrajectoryPlayer` 用 latest-only 队列把最新 waypoint 写入 `Ur5ArticulationJointController.SetJointTargetsDegrees(...)`。
   - `Ur5ArticulationJointController` 平滑 `jointTargets -> appliedJointTargets -> ArticulationBody.xDrive.target`。
   - `Quest3RobotiqGripperController` 平滑夹爪开合量并写入夹爪关节 `xDrive.target`。
4. `LateUpdate`
   - 默认控制链中不应有手柄控制写入 `TcpTarget`。
   - `TcpTargetWorkspaceLimiter` 和 `TcpTargetCollisionGuard` 仍有 legacy LateUpdate 写入能力，但 Bootstrap 默认关闭它们的写入开关。
   - `Quest3ControllerVisualizer`、`Ur5ActualTcpMarker` 和 spectator camera 是可视化/观察者，不是机器人命令源。

`Ur5JointTrajectoryPlayer` 使用 `DefaultExecutionOrder(10)`，workspace limiter 为 `DefaultExecutionOrder(100)`，collision guard 为 `DefaultExecutionOrder(125)`。`Ur5CartesianVelocityTeleopController` 与 `Ur5TcpTargetFollower` 没有显式 execution order；两者同为默认 FixedUpdate 时，同帧内谁先执行由 Unity 脚本执行顺序决定。因此审计结论只把它们描述为同一物理帧层级的生产者/消费者，不假设固定的前后顺序。

## 主手柄路径

```text
RightHand / LeftHand InputDevice
  -> Ur5CartesianVelocityTeleopController.Update
     -> TryReadControllerPosition / TryReadControllerRotation / TryReadRotationJoystick
     -> ReadGripDeadman / ReadAPrecisionModifier / ReadSecondaryButton
     -> UpdateLeftControllerSafetyPose
     -> CaptureClutchOrigins
        -> anchoredPoseTeleop.Resume(hand pose, actual tool pose)
        -> relativePoseClutchMapper.Begin(...)
        -> HoldTargetRotationAtCurrentGraspFrame("QuestTeleop") when pure right-hand translation needs orientation lock
     -> ApplySafetyLimitsAndFiltering
  -> Ur5CartesianVelocityTeleopController.FixedUpdate
     -> ApplyUnityPreview
     -> ApplyRelativePosePreview
        -> requestedRotation from orientation lock, left-stick yaw, secondary pose yaw, controller pose delta, or persistent target
        -> requestedPosition from anchoredPoseTeleop.TryGetRequestedPose(...)
        -> workspaceLimiter.ClampWorldPosition(...)
        -> anchoredPoseTeleop.FilterRequestedPose(...)
        -> tcpPreviewTarget.SetPositionAndRotation(...)
        -> targetWriteMonitor.RecordWrite("QuestTeleopAnchoredPose")
     -> speedlClient.SetCommand(..., active = IsCommandActive && IsInputPoseValid)
```

关键审计结论：

- 右手纯平移时，主路径会优先使用 `positionOrientationLock`，所以右手 Grip 不应主动改变完整 TCP 旋转。
- 左手摇杆旋转是有意的 yaw 通道，当前默认 pitch/roll 速度配置为 0，实际 yaw 轴来自两指中心的物理对称轴。
- `anchoredPoseTeleop` 捕获的是实际 tool pose，而不是滞后的命令目标；这避免松手或重新 clutch 时追赶旧 TcpTarget。
- 默认配置关闭 `useRelativePoseCommandFilter`，主路径只保留 anchored strategy 的单级滤波。

## Unity IK 预览路径

```text
TcpTarget world pose
  -> Ur5TcpTargetFollower.FixedUpdate
     -> StepTowardTarget / StepStrictTranslationWithLockedOrientation / ApplyDampedLeastSquaresStep
     -> CommitJointWaypoint
        -> Ur5JointTrajectoryPlayer.EnqueueWaypointDegrees(...)
  -> Ur5JointTrajectoryPlayer.FixedUpdate
     -> Dequeue latest waypoint
     -> Ur5ArticulationJointController.SetJointTargetsDegrees(...)
  -> Ur5ArticulationJointController.FixedUpdate
     -> smooth joint target toward requested joint target
     -> ApplyDriveTarget(...)
     -> ArticulationBody.xDrive.target
```

`Ur5TcpTargetFollower` 还负责两类非手柄写入：

- 启动/ready pose 完成时把 `TcpTarget` 对齐到实际两指中心。
- Grip release 后若仍有可见命令领先量，通过 `SnapTargetToActualPose(...)` 或 timeout 安全释放把命令目标一次性拉回实际 TCP。

这些写入是显式安全边界，不应被当作普通手柄输入。

## UDP 影子遥测路径

```text
Quest3UdpTeleopSender.Update
  -> InputDevices.GetDeviceAtXRNode(controllerNode)
  -> CommonUsages.devicePosition / deviceRotation / grip / trigger / primaryButton / secondaryButton
  -> QuestPacketPayload
     - sequence
     - quest_timestamp_ns
     - controller.position
     - controller.quaternion_xyzw
     - buttons.clutch
     - buttons.trigger
     - buttons.recenter
     - buttons.stop_episode
  -> udpClient.Send(...)
```

该路径是只读影子遥测。代码注释明确保留 Quest tracking frame 原始 pose，不经过 XR Origin；后续 PC/Python 侧应自行做 clutch 相对位姿与 UR5 base frame 映射。它不关联 `Ur5UrScriptSpeedlClient`，也不会直接开启真机输出。

## 夹爪输入路径

```text
RightHand trigger + Grip deadman
  -> Quest3RobotiqGripperController.Update
     -> targetCloseAmount
  -> Quest3RobotiqGripperController.FixedUpdate
     -> currentCloseAmount = MoveTowards(...)
     -> ApplyCloseAmount(...)
     -> gripper ArticulationBody.xDrive.target
```

夹爪路径与 TCP 位姿路径分离。手动 trigger 会被 Grip deadman 保护；抓取辅助期间可临时关闭手动 trigger 输入，防止自动 Close 阶段与手动 trigger 竞争。

## 真机 speedl 输出路径

```text
Ur5CartesianVelocityTeleopController.FixedUpdate
  -> speedlClient.SetCommand(filteredBaseLinearVelocity, filteredBaseAngularVelocity, active)
Ur5UrScriptSpeedlClient.FixedUpdate
  -> if enableRealRobotOutput == false: return
  -> TryEnsureConnected
  -> commandActive / timeout / armed gate
  -> LimitOutputAcceleration
  -> SendStopl on stop edge
  -> SendSpeedl(...)
```

审计边界：阶段9.1只确认 Unity 侧调用链。后续 9.4/9.5 仍需要把 Unity-Python 协议、RobotCommand、安全层和 UR5 executor 明确定义到 Python/机器人侧；当前 Unity speedl client 不能替代最终安全执行层。

## 额外旋转来源假设与证据

| 假设类别 | 当前证据 | 结论 | 后续验证证据 |
|---|---|---|---|
| 坐标变换错误 | 主手柄路径可经 `xrOrigin` 转到 Unity world；UDP 影子遥测故意不经 XR Origin。主控制器默认使用 robot base frame 做速度/约束映射。 | 若“手柄单轴移动导致 TCP 旋转”只在特定 XR Origin/场景父级下出现，优先检查 XR Origin 与 robot base 的相对姿态。 | 9.2 单轴测试：分别记录 raw controller pose、world pose、base-frame delta、LogicalCommandRotation。 |
| 重复写入 | Bootstrap 默认禁用旧 Quest pose、旧 VR、键盘和 workspace/collision legacy writer；主路径写入标签为 `QuestTeleopAnchoredPose`。 | 若当前场景仍出现额外旋转，第一优先级是检查是否绕过 Bootstrap 或有旧组件重新启用。 | 打开 `TcpTargetWriteMonitor.enableDiagnostics`，观察单帧是否出现多个 writer tag。 |
| 父子层级继承 | 代码多用 world-space `position`/`rotation` 或 `SetPositionAndRotation`；但 `TcpTarget` 若挂在非单位缩放/旋转父级下，Inspector 里的 local pose 仍会受父级影响。 | 额外旋转若只在某个场景层级出现，应检查 `TcpTarget.parent`、父级 rotation/scale 和 robot root transform。 | 运行时打印 `TcpTarget.parent`、`parent.lossyScale`、`parent.rotation`、`TcpTarget.rotation` 与 `localRotation`。 |
| 角度累积 | 旧 `Quest3TcpTargetController` 会把 controller delta 乘到 clutch 起始旋转；当前主控制器的左摇杆 yaw 从 persistent orientation 积分，注释指出从 filtered TcpTarget 回灌会形成二次伺服。 | 当前主路径中，右手纯平移被 `positionOrientationLock` 锁住，不应产生主动旋转；左手 yaw 通道出现的旋转属于设计行为，若发生 pitch/roll 才是异常。 | 9.2 四元数测试：右手-only translation 的 `LogicalCommandRotation` 恒定；左摇杆 yaw 只绕物理两指中心轴变化。 |

## 当前额外旋转的解释优先级

不改代码即可得出的优先级如下：

1. **重复写入优先排查**：如果某个场景未由 `Ur5ControlBootstrap` 接管，旧 `Quest3TcpTargetController` 或 `VRTeleoperationController` 可能仍会写 `TcpTarget.rotation`。
2. **父级/坐标系排查**：如果 write monitor 只看到 `QuestTeleopAnchoredPose`，但画面仍出现非预期旋转，应检查 `TcpTarget` 父级、XR Origin 与 robot base 的相对旋转。
3. **角度累积排查**：如果现象只在左手摇杆或左手姿态通道出现，检查 `persistentOrientationTarget`、`CalculateJoystickPreviewRotation(...)` 和物理两指中心轴。
4. **workspace/collision 排查**：workspace limiter 默认在主路径前 clamp，LateUpdate 写入关闭；如果被手动打开，它只写 position，但会改变 follower 后续 IK 目标，可能表现为腕部补偿旋转。

## 9.2 输入建议

9.2 坐标与旋转数学测试台应优先把以下状态暴露为可记录字段：

- `RawControllerPositionWorld`
- `StabilizedControllerPositionWorld`
- `LogicalCommandPosition`
- `LogicalCommandRotation`
- `FilteredCommandPosition`
- `FilteredCommandRotation`
- `ConstrainedCommandPosition`
- `IsPreviewLeadLimited`
- `targetWriteMonitor.LastWriter`
- `tcpFollower.ActualGraspRotation`
- `tcpFollower.RotationErrorDegrees`

这些字段可以直接验证 9.1 中的四类假设，避免用肉眼判断“多转了一点”。

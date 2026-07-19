# Quest 3 AR UR5 Teleoperation

本项目当前目标是先把 Unity/Quest 3 端的机械臂预览控制做稳定，再扩展到真机 UR5 数据采集。当前控制链已按 Unity Robotics Hub pick-and-place 的思路重构：不要让手柄每帧直接推关节，而是先生成关节路点，再按固定节拍统一写入每个 `ArticulationBody.xDrive.target`。

## 控制架构

```text
Quest 3 手柄
  -> Ur5CartesianVelocityTeleopController
  -> TcpTarget 相对位姿目标
  -> Ur5TcpTargetFollower 生成小步关节路点
  -> Ur5JointTrajectoryPlayer 固定节拍播放路点
  -> Ur5ArticulationJointController / ArticulationBody.xDrive.target
```

核心变化：

- `Ur5CartesianVelocityTeleopController` 只负责根据手柄输入移动 `TcpTarget`，默认使用 `RelativePoseTarget`，按下 grip 时锁定手柄起点和 TCP 起点。
- `Ur5TcpTargetFollower` 不再到处直接调用 `AddJointTargetDegrees`，而是每次 IK step 只生成一个候选关节路点。
- `Ur5JointTrajectoryPlayer` 负责像 Unity Robotics Hub 的 trajectory playback 一样，把关节路点按 `jointAssignmentIntervalSeconds` 固定写入 xDrive。
- `QueueMode.LatestOnly` 会丢弃积压旧路点，只执行最新路点，避免 Quest 手柄噪声和帧率波动造成控制滞后。

## 手柄操作

- 右手 grip：平移 TCP 目标。
- 左手 grip：旋转 TCP 目标。
- primary button：细控制模式，降低线速度、角速度和相对位姿映射比例，用于靠近物体或采集精细动作。
- trigger/grip 夹爪控制：仍由原夹爪脚本处理，和机械臂 TCP 控制分离。
- 松开 grip：进入 deadman idle hold，清空轨迹队列，并锁定当前关节姿态。

## 关键脚本

- `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`：Quest 输入、相对位姿预览、未来真机 `speedl` 速度命令源。
- `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs`：TCP 误差到关节路点的局部求解器。
- `Assets/Scripts/UR5/Ur5JointTrajectoryPlayer.cs`：关节路点队列和固定节拍 xDrive 写入。
- `Assets/Scripts/UR5/Ur5ArticulationJointController.cs`：UR5 六关节发现、drive 参数、批量关节目标写入。
- `Assets/Scripts/UR5/Ur5UrScriptSpeedlClient.cs`：真机 URScript `speedl` 输出，默认关闭，需要显式 enable/arm。
- `Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs`：记录 TCP、关节目标、速度、轨迹队列状态和真机输出状态。

## 推荐调参顺序

先只在 Unity/Quest 里调稳定性，不连接真机：

1. 确认松开手柄时 `trajectory_pending_waypoints` 为 0，机械臂不应自发晃动。
2. 当前默认是快速预览档：`relativePreviewPositionScale = 2.40`、`previewMaxLinearSpeed = 0.35`、`maxJointStepDegrees = 1.20`、`jointAssignmentIntervalSeconds = 0.016`。
3. 如果运动仍抖，优先降低 `Ur5TcpTargetFollower.maxJointStepDegrees`，例如从 `1.20` 降到 `0.80`。
4. 如果普通移动太灵敏，降低 `relativePreviewPositionScale`，例如从 `2.40` 降到 `1.80`。
5. 如果跟随仍太慢，再小幅降低 `Ur5JointTrajectoryPlayer.jointAssignmentIntervalSeconds`，例如从 `0.016` 到 `0.014`。
6. 如果手柄目标本身太慢，提高 `previewMaxLinearSpeed`，例如从 `0.35` 到 `0.45`。
7. 如果手柄轻微抖动会触发目标移动，提高 `linearDeadbandMeters` 或 `angularDeadbandDegrees`。
8. 如果普通模式太灵敏，按住 primary button 进入细控；如果仍太灵敏，降低 `relativePreviewPositionScale` 和 `relativePreviewRotationScale`。

## 真机扩展策略

Unity 阶段使用本地关节路点播放来稳定数字孪生；真机阶段不要直接照搬 Unity 的 ArticulationBody 状态。建议路径：

- 保持 Quest 端生成稳定的 TCP 线速度/角速度命令。
- 先用 `Ur5UrScriptSpeedlClient` 小速度、短周期、需 arm 的方式做真机验证。
- 真机闭环必须接入 UR 实际 TCP/关节反馈，再用反馈刷新 Unity 数字孪生。
- 真机输出默认关闭：`enableRealRobotOutput = false`，不要在人员靠近或工作空间未标定时开启。

## 常见问题

### 进入应用后不操作也晃

检查：

- `Ur5CartesianVelocityTeleopController.IsCommandActive` 应为 false。
- `Ur5JointTrajectoryPlayer.PendingWaypointCount` 应为 0。
- `Ur5TcpTargetFollower.pauseIkWhenVelocityTeleopIdle` 应为 true。
- `useGripperPadCenter` 应为 false，避免夹爪开闭时 TCP 参考点变化带动机械臂补偿。

### 夹爪能动但机械臂不动

检查：

- `Ur5ControlBootstrap.enableCartesianVelocityTeleop` 是否开启。
- Quest 控制器是否能读到 grip，Console 应显示 Quest position/rotation controller connected。
- `TcpTarget` 是否存在，且被 `Ur5CartesianVelocityTeleopController.tcpPreviewTarget` 引用。
- `Ur5JointTrajectoryPlayer` 是否挂在 UR5 robot root 上。

### 运动有明显延迟

检查：

- `Ur5JointTrajectoryPlayer.queueMode` 应为 `LatestOnly`。
- `maxQueuedWaypoints` 保持 1，不要让手柄历史输入排队。
- `trajectory_pending_waypoints` 长时间大于 1 时，说明路点产生速度超过播放速度。

## 参考项目

- Unity Robotics Hub: <https://github.com/Unity-Technologies/Unity-Robotics-Hub>
- 参考思路：`tutorials/pick_and_place/Scripts/TrajectoryPlanner.cs` 中的 trajectory execution，会逐个轨迹点把每个关节的 `xDrive.target` 更新为规划结果。

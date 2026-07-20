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

## 是否符合主流流程

当前控制流程符合主流 XR/VR 机械臂遥操作的 Unity 预览阶段做法：输入层只产生目标 TCP 位姿或速度，控制层按固定周期限速执行，松手 deadman 停止，轨迹队列采用 latest-only，夹爪和 TCP 控制分离。

还不应直接视为真机生产级闭环控制。接入真机前必须补齐实际 TCP/关节反馈闭环、机器人侧速度/加速度/jerk 限制、急停/保护停止、工作空间与自碰撞约束、相机/基座/机器人坐标标定，以及数据集记录的时间同步。

## 手柄操作

- 右手 grip：移动 TCP；手柄的自然转动不会改变夹爪朝向，因此可以稳定地下探抓取。
- 保持右手 grip 并推右摇杆：显式调整夹爪朝向（左右为绕机器人竖直轴偏航、上下为绕机器人 X 轴俯仰）。松开摇杆时保持当前夹爪朝向。
- 右手 trigger：控制 Robotiq 夹爪开合，输入带死区和平滑滤波。
- 右手 primary/A：细控制模式，降低线速度、角速度和相对位姿映射比例，用于靠近物体或采集精细动作。
- 右手 secondary/B：启动或中止抓取辅助流程；键盘 `G` 启动，`X` 中止。
- 松开右手 grip：进入 deadman idle hold，清空轨迹队列，并锁定当前关节姿态。
- 左手柄默认不参与控制。若要恢复“手腕姿态直接控制夹爪”，可将 `rotationInputMode` 改回 `ControllerPoseDelta`；这会重新引入下探时因手腕自然转动造成夹爪翻转的风险。

## 抓取辅助

`Ur5GraspAssistController` 将 GitHub Pick-and-Place 的抓取分段迁移到当前非 ROS 控制链：

```text
选择 TCP 附近小物体
  -> PreGrasp: 移到物体上方
  -> Grasp: 沿上方接近方向垂直下探
  -> Close: 暂停手柄夹爪输入并闭合夹爪
  -> Lift: 抬升到安全高度
  -> 交还手柄控制
```

- 默认自动选择 `TcpTarget` 附近 `0.35m` 内、尺寸小于 `0.35m` 的非机器人碰撞体。
- 抓取辅助期间会暂停 `Ur5CartesianVelocityTeleopController.enableUnityPreview`，防止手柄输入和自动抓取目标互相打架。
- 抓取辅助默认保持当前 TCP 姿态；实际使用时先用右手 grip 一手调整夹爪位姿，再按右手 secondary/B 启动辅助。
- 安全检查会阻止 TCP 目标进入机器人本体近距离区域，降低撞到自身手臂的风险。

## 关键脚本

- `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`：Quest 输入、相对位姿预览、未来真机 `speedl` 速度命令源。
- `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs`：TCP 误差到关节路点的局部求解器。
- `Assets/Scripts/UR5/Ur5JointTrajectoryPlayer.cs`：关节路点队列和固定节拍 xDrive 写入。
- `Assets/Scripts/UR5/Ur5GraspAssistController.cs`：半自动 PreGrasp/Grasp/Close/Lift 抓取辅助。
- `Assets/Scripts/UR5/Ur5ArticulationJointController.cs`：UR5 六关节发现、drive 参数、批量关节目标写入。
- `Assets/Scripts/UR5/Ur5UrScriptSpeedlClient.cs`：真机 URScript `speedl` 输出，默认关闭，需要显式 enable/arm。
- `Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs`：记录 TCP、关节目标、速度、轨迹队列状态和真机输出状态。

## 推荐调参顺序

先只在 Unity/Quest 里调稳定性，不连接真机：

1. 确认松开手柄时 `trajectory_pending_waypoints` 为 0，机械臂不应自发晃动。
2. 当前默认是快速预览档：`relativePreviewPositionScale = 2.40`、`previewMaxLinearSpeed = 0.35`、`maxJointStepDegrees = 1.45`、`maxWristStepDegrees = 2.00`、`jointAssignmentIntervalSeconds = 0.016`。
3. 右手一手控制默认参数：`positionControllerNode = RightHand`、`rotationControllerNode = RightHand`、`rotationInputMode = Joystick`。位移和夹爪朝向由不同输入通道控制，手腕物理姿态不会干扰抓取方向。
4. 如果运动仍抖，优先降低 `Ur5TcpTargetFollower.maxJointStepDegrees` 和 `maxWristStepDegrees`，例如从 `1.45` / `2.00` 降到 `1.20` / `1.50`。
5. 如果普通移动太灵敏，降低 `relativePreviewPositionScale`，例如从 `2.40` 降到 `1.80`。
6. 如果跟随仍太慢，再小幅降低 `Ur5JointTrajectoryPlayer.jointAssignmentIntervalSeconds`，例如从 `0.016` 到 `0.014`。
7. 如果手柄目标本身太慢，提高 `previewMaxLinearSpeed`，例如从 `0.35` 到 `0.45`。
8. 如果摇杆旋转太快，降低 `joystickYawSpeedDegreesPerSecond` 和 `joystickPitchSpeedDegreesPerSecond`；如果仍慢，再小幅提高它们。
9. 如果手柄轻微抖动会触发目标移动，提高 `linearDeadbandMeters` 或 `angularDeadbandDegrees`。
10. 如果普通模式太灵敏，按住 primary/A 进入细控；如果仍太灵敏，降低 `relativePreviewPositionScale` 或 `relativePreviewRotationScale`。

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
- Unitree XR Teleoperate: <https://github.com/unitreerobotics/xr_teleoperate>
- 参考思路：手柄模式下将 XR 控制器输入转换为限速运动命令；本项目默认将右手位移与摇杆姿态命令分离，避免抓取时的自然腕部转动干扰 TCP，不引入 Unitree SDK。

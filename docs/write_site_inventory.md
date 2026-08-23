# 9.1 Write-Site Inventory

> 本表列出 Unity 主线程中可能写入控制目标、机器人关节目标、夹爪目标或网络输出的站点。排序按典型生命周期：Awake/Start -> Update -> FixedUpdate -> LateUpdate -> public safety call。

## 读取方式

- **Active by default**：在 `Ur5ControlBootstrap.enableCartesianVelocityTeleop = true` 的当前默认配置下会启用。
- **Legacy/disabled by bootstrap**：代码仍存在，但默认 runtime bootstrap 会禁用或关闭写入开关。
- **Conditional safety writer**：只有启动、ready pose、release、assist 或异常边界发生时写入。
- **Observer/non-command**：可视化、诊断或记录，不是控制命令源。

| ID | File / symbol | Lifecycle | Status | Input source | Written target | Downstream consumer | Notes |
|---:|---|---|---|---|---|---|---|
| 1 | `Ur5ControlBootstrap.ResolveTcpTarget` | `Awake` | Active by default | Scene lookup / fallback creation | `TcpTarget.transform.position`, scale, renderer color | All teleop controllers | Only creates fallback target when scene has no `TcpTarget`. |
| 2 | `Ur5ControlBootstrap.ConfigureTcpTargetSafety` | `Awake` | Active by default | Bootstrap config | `workspaceLimiter.constrainInLateUpdate = false`, `collisionGuard.preventObstaclePenetration = false`, write monitor disabled | Manual control path | Disables legacy LateUpdate position writers so manual control has one explicit owner. |
| 3 | `Ur5ControlBootstrap.DisableQuestPoseController` | `Awake` | Active by default | `enableCartesianVelocityTeleop` | `Quest3TcpTargetController.enabled = false` | TcpTarget writer arbitration | Prevents old pose controller from competing with velocity teleop. |
| 4 | `Ur5ControlBootstrap.DisableLegacyVrTeleoperationControllers` | `Awake` | Active by default | `enableCartesianVelocityTeleop` | `VRTeleoperationController.enabled = false` | TcpTarget writer arbitration | Prevents old VR teleop from writing smoothed TCP target. |
| 5 | `Ur5ControlBootstrap.DisableKeyboardTargetController` | `Awake` | Active by default | `enableCartesianVelocityTeleop` | `TcpTargetKeyboardController.enabled = false` | TcpTarget writer arbitration | Keyboard remains a legacy/manual debug path only when explicitly enabled. |
| 6 | `Ur5CartesianVelocityTeleopController.Update` | `Update` | Active by default | Right/left XR InputDevice pose, grip, joystick, A/X/Y | Internal clutch, command, safety and velocity state | Same controller `FixedUpdate` | Reads input only; does not write `TcpTarget` in Update. |
| 7 | `Quest3UdpTeleopSender.TrySendLatestControllerPacket` | `Update` | Optional read-only | Raw Quest InputDevice pose/buttons | UDP JSON packet | PC/Python shadow receiver | Sends raw tracking-frame pose; no robot or TcpTarget write. |
| 8 | `Quest3RobotiqGripperController.Update` | `Update` | Active by default | Right trigger + Grip deadman, optional O/P keys | `targetCloseAmount` | Gripper `FixedUpdate` | Separates operator gripper intent from TCP pose control. |
| 9 | `Ur5GraspAssistController.Update` | `Update` | Active component, inactive unless commanded | Keyboard G/X or configured controller buttons | Assist state machine | Assist `FixedUpdate` | Current bootstrap disables A/B assist buttons to avoid competing with precision controls. |
| 10 | `Quest3TcpTargetController.ApplyTarget` | `Update` or `FixedUpdate` | Legacy/disabled by bootstrap | Old Quest pose controller | `transform.position`, `transform.rotation` | `Ur5TcpTargetFollower` | If re-enabled, it becomes a second TcpTarget writer and can explain extra rotation. |
| 11 | `VRTeleoperationController.UpdateTcpTargetFromControllerDelta` | `Update` | Legacy/disabled by bootstrap | Legacy controller delta | `tcpTargetTransform.SetPositionAndRotation(...)` | `Ur5TcpTargetFollower` | Legacy path; disabled when velocity teleop is active. |
| 12 | `TcpTargetKeyboardController.Update` | `Update` | Legacy/disabled by bootstrap | Keyboard WASD/QE/ZC | `transform.position += ...`, `transform.Rotate(...)` | `Ur5TcpTargetFollower` | Debug path; if enabled with Quest teleop it can create duplicate writes. |
| 13 | `Ur5CartesianVelocityTeleopController.ApplyRelativePosePreview` | `FixedUpdate` | Active by default | Clutched controller pose, orientation lock, joystick yaw, persistent target | `tcpPreviewTarget.SetPositionAndRotation(...)` | `Ur5TcpTargetFollower`, recorder, diagnostics | Main TcpTarget writer; records `QuestTeleopAnchoredPose`. |
| 14 | `Ur5CartesianVelocityTeleopController.ApplyVelocityIntegrationPreview` | `FixedUpdate` | Conditional alternate mode | Filtered base linear/angular velocity | `tcpPreviewTarget.SetPositionAndRotation(...)` | `Ur5TcpTargetFollower` | Only when `unityPreviewMode = VelocityIntegration`; default is `RelativePoseTarget`. |
| 15 | `Ur5CartesianVelocityTeleopController.FixedUpdate` | `FixedUpdate` | Active by default | Filtered base velocity + active/valid gates | `speedlClient.SetCommand(...)` | `Ur5UrScriptSpeedlClient` | Sets command buffer only; true robot output still gated later. |
| 16 | `Ur5TcpTargetFollower.SnapTargetToEndEffector` | `Start`, ready-pose complete | Conditional safety writer | Actual two-pad control point | `tcpTarget.position`, optional `tcpTarget.rotation` | Manual target baseline | Aligns command target to actual TCP at startup/ready completion. |
| 17 | `Ur5TcpTargetFollower.HoldTargetRotationAtCurrentGraspFrame` | Public helper | Conditional safety writer | Actual grasp frame | `tcpTarget.rotation` | Manual target orientation lock | Called by clutch capture, startup, freeze and safe release paths. |
| 18 | `Ur5TcpTargetFollower.SnapTargetToActualPose` | `FixedUpdate` via safe release | Conditional safety writer | Actual TCP pose | `tcpTarget.position`, optional `tcpTarget.rotation` | Idle hold and next clutch baseline | One-shot release correction when target lead remains after Grip release. |
| 19 | `Ur5TcpTargetFollower.FreezeAtCurrentPose` | Public safety call | Conditional safety writer | Measured joint/TCP pose | Joint hold + `tcpTarget.position`, optional rotation | Idle hold | Freezes arm and aligns command target to actual TCP. |
| 20 | `Ur5TcpTargetFollower.CommitJointWaypoint` | `FixedUpdate` | Active by default | TcpTarget error, IK/DLS solver | `trajectoryPlayer.EnqueueWaypointDegrees(...)` | Trajectory player | Usually queues latest waypoint instead of direct drive writes. |
| 21 | `Ur5JointTrajectoryPlayer.FixedUpdate` | `FixedUpdate`, execution order 10 | Active by default | Latest queued waypoint | `jointController.SetJointTargetsDegrees(...)` | Articulation controller | Latest-only queue drops stale waypoints. |
| 22 | `Ur5ArticulationJointController.SetJointTargetsDegrees` | Called by trajectory/follower | Active by default | Waypoint degrees | `jointTargets[]`; optionally `appliedJointTargets[]` and xDrive | Articulation controller `FixedUpdate` | With smoothing enabled, direct xDrive write is deferred to controller FixedUpdate. |
| 23 | `Ur5ArticulationJointController.FixedUpdate` | `FixedUpdate` | Active by default | `jointTargets[]` | `appliedJointTargets[]`, `ArticulationBody.xDrive.target` | Unity physics articulation | Smooths drive target speed/acceleration. |
| 24 | `Quest3RobotiqGripperController.FixedUpdate` | `FixedUpdate` | Active by default | `targetCloseAmount` | Gripper `ArticulationBody.xDrive.target` | Unity physics articulation | Independent from UR5 arm joints. |
| 25 | `Ur5GraspAssistController.MoveTowardWaypoint` | `FixedUpdate` while assist active | Conditional safety/assist writer | Assist state machine | `tcpTarget.SetPositionAndRotation(...)` | `Ur5TcpTargetFollower` | Owns TcpTarget during assist and pauses manual preview. |
| 26 | `Ur5GraspAssistController.HoldTargetPose` | `FixedUpdate` while assist active | Conditional safety/assist writer | Assist state machine | `tcpTarget.SetPositionAndRotation(...)` | `Ur5TcpTargetFollower` | Holds pose during settle/close stages. |
| 27 | `Ur5UrScriptSpeedlClient.FixedUpdate` | `FixedUpdate` | Disabled for real robot unless explicitly enabled | Command buffer from velocity teleop | Socket URScript `speedl(...)`, `stopl(...)` | UR controller | Returns immediately unless `enableRealRobotOutput` is true. |
| 28 | `Ur5UrScriptSpeedlClient.StopRobot` | Public safety call | Conditional safety writer | Disarm/disable/manual stop | Zero command + `stopl`/zero `speedl` if connected | UR controller | Safe stop edge for real robot output. |
| 29 | `TcpTargetWorkspaceLimiter.LateUpdate` | `LateUpdate`, execution order 100 | Legacy/disabled by bootstrap | Current `TcpTarget.position` | `transform.position = clamped` | `Ur5TcpTargetFollower` next frame | Position-only writer; bootstrap sets `constrainInLateUpdate = false`. |
| 30 | `TcpTargetCollisionGuard.LateUpdate` | `LateUpdate`, execution order 125 | Legacy/disabled by bootstrap | Physics overlap sphere | `transform.position = lastSafePosition` | `Ur5TcpTargetFollower` next frame | Position-only fallback; bootstrap sets `preventObstaclePenetration = false`. |
| 31 | `Quest3ControllerVisualizer.LateUpdate` | `LateUpdate` | Observer/non-command | XR InputDevice pose/trigger | Visual controller transforms/materials | Operator visual feedback | Does not write robot/TcpTarget command. |
| 32 | `Ur5ActualTcpMarker.LateUpdate` | `LateUpdate` | Observer/non-command | `Ur5TcpTargetFollower` actual TCP pose | Actual TCP marker transform | Visual feedback/diagnostics | Green marker is physical two-pad center, not command target. |
| 33 | `Ur5PoseCsvRecorder.Update` | `Update` | Observer/non-command | Current controller/TcpTarget/follower/joint/speedl state | In-memory CSV buffer / log file when toggled | Offline diagnosis | Reads target and actual state; does not command movement. |
| 34 | `Ur5PhysicsStabilizer.FixedUpdate` | `FixedUpdate` | Active by default | Locked root pose | `robotRoot.SetPositionAndRotation(...)` | Scene stability | Locks robot root; not a TcpTarget command but relevant for parent-frame audits. |

## High-risk combinations

- `Quest3TcpTargetController.enabled == true` while `Ur5CartesianVelocityTeleopController.enabled == true`: duplicate TCP pose writer.
- `TcpTargetKeyboardController.enabled == true` during Quest teleop: keyboard movement can silently add world yaw/position changes.
- `TcpTargetWorkspaceLimiter.constrainInLateUpdate == true`: LateUpdate can rewrite position after manual FixedUpdate.
- `TcpTargetCollisionGuard.preventObstaclePenetration == true`: LateUpdate can revert position after manual FixedUpdate.
- `TcpTarget` under a rotated/scaled parent: all world-space writes can appear as unexpected local rotation/scale effects in Inspector.
- `Ur5UrScriptSpeedlClient.enableRealRobotOutput == true` before 9.5 safety layer: Unity speedl client is not the final RobotCommand executor.

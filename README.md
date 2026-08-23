# Quest 3 AR UR5 Teleoperation

本项目用于在 Tuanjie/Unity 中构建 Quest 3 沉浸式 UR5 遥操作与数据采集预览环境。当前阶段仍以 Quest3 Build And Run 测试为主，真机 UR5 输出默认关闭，必须在完成空间标定、安全边界和低速联调后再开启。

## 当前控制方案

控制链路采用 UR10_Teleop/Open-Teach 风格的相对位姿离合：按下 Grip 时同时锚定右手柄位姿和当前 TCP 位姿，之后所有命令都由该锚点的相对变化推导，避免手柄静止时持续追赶旧目标。

```text
Quest 3 right controller
  -> Ur5CartesianVelocityTeleopController
  -> TcpTarget relative pose command
  -> Ur5TcpTargetFollower IK / measured-state servo
  -> ArticulationBody joint drive targets
```

### 手柄操作

- **右手 Grip**：主平移离合。按住后移动右手柄，机械臂 TCP 跟随平移；松开后进入 deadman hold。
- **右手 Grip + A**：稳定默认外翻下抓姿态。该模式只调姿态，不拖动 TCP 位置；无论先按 Grip 再按 A，还是同时按 Grip+A，都会进入同一个外翻下抓参考，避免腕部向内反。
- **右手 Grip + B**：自由腕部/小臂姿态。该模式只调姿态，不拖动 TCP 位置；完整映射右手柄相对四元数，可用于横抓、侧抓、侧向伸入等非朝下抓取任务。
- **右手 Trigger**：控制 Robotiq 夹爪开合，带死区和平滑滤波。

推荐使用方式：普通采集任务优先用 **Grip 平移 + Grip+A 下抓姿态**；当需要横着夹、侧向接近或调整腕部俯仰/翻滚时，再按住 **Grip+B** 进入自由姿态。

## Quest3-Python-UR5 通信协议

Stage 9.4 之后，Unity 不直接驱动真实 UR5。Quest3 应用只作为原始手柄输入发送端，Mac/Fedora 笔记本上的 Python bridge 才负责协议解析、安全过滤和真机执行。

```text
Quest3 Unity App
  -> UDP packets, port 8080
  -> Python bridge: scripts/teleop/quest3_ur5_openteach.py
  -> OpenTeachTeleopTracker relative-pose mapping
  -> SafeUr5Executor + Ur5SafetyFilter
  -> VirtualTcpBackend / RtdeUr5Backend
  -> UR5 controller through RTDE when real-robot is explicitly enabled
```

### 协议语义

Unity 端发送 protocol v2 packet，字段含义由 `Quest3UdpTeleopSender` 维护，Python 端按同一 schema 解析。关键语义如下：

- `mode=idle`：未按 Grip 或输入无效；Python bridge 不发送运动命令。
- `mode=translate`：右手 Grip；只映射右手相对平移，保持当前姿态锁。
- `mode=default_grasp`：右手 Grip+A；只调默认外翻下抓姿态，不拖动 TCP 位置。
- `mode=free_wrist`：右手 Grip+B；只调自由腕部/小臂姿态，不拖动 TCP 位置。
- `grip` / `clutch`：deadman 控制；松开后必须停止，不保持最后速度命令。
- `trigger`：夹爪开闭输入，作为采集字段保留；真机运动安全不依赖 trigger。
- `valid_controller_pose`、`valid_buttons`、`valid_tracking`：任一关键 valid flag 失效时，Python 端应 fail-closed。
- `sequence`、`host_time_ns`：用于丢包、乱序、延迟和 CSV gate 检查。

### Unity 端职责

- 读取 Quest3 右手柄位姿、Grip、A、B、Trigger 和 tracking valid flags。
- 按上述模式发送 UDP packet 到笔记本 IP 的 `8080` 端口。
- 保留 Unity 内部 TCP/IK 预览和 `ur5_pose_log.csv`，用于 Quest3 侧调试。
- 不在 Unity 内开启真实 UR5 输出；真实通信统一由 Python bridge 执行。

### Python / 真机端职责

- 接收 Quest3 UDP packet，并转为 `RobotCommand`。
- 在 `SafeUr5Executor` 中执行 workspace、单步位移、角度、速度和异常门控。
- `virtual-tcp` 后端只更新内部 TCP pose，不连接真实机器人。
- `real-robot` 后端必须显式带 `--confirm-real-robot`，并通过 RTDE 向 UR5 发送低速 servo 命令。
- 每次 gate 运行必须写 CSV，后续用 `validate_quest3_ur5_log.py` 生成验证报告。

## 关键脚本

- `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`：Quest 输入、Grip/A/B 离合状态机、相对位姿命令、CSV 诊断字段来源。
- `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs`：TCP 目标到 UR5 关节目标的 IK/测量状态跟随器；Grip+A/Grip+B 调姿时会启用腕部优先。
- `Assets/Scripts/UR5/Ur5ControlBootstrap.cs`：默认 Quest 遥操作参数配置入口，避免 Inspector 手动漏配。
- `Assets/Scripts/UR5/Quest3RobotiqGripperController.cs`：右手 Trigger 到夹爪开合的输入映射。
- `Assets/Scripts/UR5/Quest3UdpTeleopSender.cs`：Stage 9.4 Unity-Python 协议 v2 原始手柄发送端；只发送 Quest tracking-frame 位姿、Grip/A/B/Trigger、mode、valid flags、序号和时间戳，不直接控制真机。
- `Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs`：Quest 测试日志，记录 TCP、关节、IK、按键模式和限制状态。
- `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`：控制链关键行为的 EditMode 回归测试。

## Quest3 测试流程

1. 在 Tuanjie 打开 `/Users/imi-1/Desktop/project/ur5`。
2. Build And Run 到 Quest3。
3. 在沉浸式界面中先测 **Grip 静止**：右手柄不动时机械臂不应晃动。
4. 测 **Grip 平移**：确认跟手速度和停手即停。
5. 测 **Grip+A**：确认能快速进入稳定下抓姿态，之后 Grip 平移时姿态不乱飘。
6. 测 **Grip+B**：保持 TCP 位置，旋转右手柄，确认末端可以横抓/侧抓/自由调整腕部姿态。
7. 拉取 `ur5_pose_log.csv` 后检查模式字段。

常用拉日志命令：

```zsh
ADB="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb"
"$ADB" pull \
  /sdcard/Android/data/com.dgvlalab.ur5quest/files/ur5_pose_log.csv \
  /tmp/ur5_pose_log.csv
tail -n 30 /tmp/ur5_pose_log.csv
```

关键 CSV 字段：

- `position_clutched` / `rotation_clutched`：Grip 控制是否处于离合状态。
- `ur10_rotation_adjust_active`：Grip+A 默认下抓姿态是否激活。
- `ur10_free_wrist_adjust_active`：Grip+B 自由腕部姿态是否激活。
- `position_orientation_locked`：Grip 单独平移时姿态锁是否生效。
- `preview_lead_limited` / `active_preview_lead_limit_m`：目标是否被实际 TCP 跟随距离限制截断。
- `near_singularity` / `dls_min_pivot`：IK 是否接近奇异或数值不稳定区域。

## 真机前安全约束

当前项目仍是 Quest/Unity 预览优先。连接真实 UR5 前必须确认：

- `Ur5UrScriptSpeedlClient.enableRealRobotOutput = false`，直到完成低速真机联调。
- 人员远离机械臂工作空间，急停可触达。
- UR5 基座、Quest tracking frame、桌面和相机坐标完成标定。
- TCP 工作空间最低高度、桌面高度和夹爪几何与真实环境一致。
- 先使用低速、小范围、无障碍动作验证 Grip、Grip+A、Grip+B，再做真实抓取。

## 根目录文件说明

- `Assets/`、`Packages/`、`ProjectSettings/`：Unity/Tuanjie 项目核心目录，需要保留并提交。
- `Library/`、`Temp/`、`Logs/`、`UserSettings/`：Unity/Tuanjie 本地缓存和日志，已在 `.gitignore` 中忽略，不应提交。
- `*.csproj`、`*.sln`：Unity/Tuanjie 为 C# IDE 自动生成的工程文件；它们便于 Rider/VSCode 跳转代码，通常可删除后由 Editor 再生成，本项目已通过 `.gitignore` 忽略。
- `mono_crash.mem.*.blob`：Mono/Unity 崩溃内存转储，用于崩溃排查；不属于项目源码，已通过 `.gitignore` 忽略，可安全清理。
- `.superpowers/`、`docs/superpowers/`：早期 Superpowers 工作流生成的计划/临时文件；当前 ur5 项目不依赖这些文件，已清理。
- `RuntimeActionBindings.json`：Tuanjie/Unity 运行时动作绑定占位文件，当前由项目跟踪；若后续确认 Editor 会自动生成且场景不依赖，可再单独移除。

## 常见问题

### 按住 Grip 不动时机械臂晃动

优先检查 `ur5_pose_log.csv`：

- `position_clutched=1` 时，`raw_hand_*` 是否仍有明显变化。
- `position_orientation_locked` 是否为 1。
- `near_singularity` 是否为 1。
- `preview_lead_limited` 是否频繁为 1。

### 控制不跟手

先确认不是安全限制导致：

- `preview_lead_limited=1` 且 `active_preview_lead_limit_m` 很小，说明目标被实际 TCP 跟随距离限制截断。
- `near_singularity=1` 或 `dls_min_pivot` 很低，说明当前姿态接近 IK 奇异区域。

### Grip+A 和 Grip+B 的区别

- Grip+A 是稳定下抓姿态，适合当前精密采集和桌面抓取。
- Grip+B 是通用自由姿态，适合未来横抓、侧抓和更复杂腕部姿态。
- 不建议把 Grip+A 改成完全自由姿态，因为它现在承担安全、稳定、可复现的默认抓取入口。

## 参考项目

- Unity Robotics Hub: <https://github.com/Unity-Technologies/Unity-Robotics-Hub>
- elpis-lab UR10_Teleop: <https://github.com/elpis-lab/UR10_Teleop>
- Unitree XR Teleoperate: <https://github.com/unitreerobotics/xr_teleoperate>

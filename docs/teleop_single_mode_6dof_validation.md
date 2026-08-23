# UR5 单模式 6DoF 验证记录

## UR10 风格返修基线（2026-08-03）
- 标准 Quest Profile 选择 `enableUr10StyleAnchoredPoseClutch=true`，并明确关闭旧 `enableContinuous6DofClutch`。
- 右手 Grip 是唯一的 6DoF 离合：按下时记录原始右手位置/四元数和实际 TCP；按住时使用固定平移映射 `(0.5, 0.5, 0.5)` 和完整相对旋转 `q_hand * inverse(q_anchor) * q_tcp_anchor`。
- 唯一目标平滑为每个 FixedUpdate 的 `0.05` 步长。默认 Profile 关闭手柄滤波、渐进增益、目标死区、命令二次滤波、目标领先限制和加速度轨迹。
- 松开 Grip 时控制器和跟随器均不写回 `TcpTarget`。跟随器只执行一次物理关节保持，保留最终逻辑目标，且默认关闭 `enablePrecisionAssemblyTracking` 和 `holdJointPoseWhenTargetSettled`。
- 左手安全姿态、旧三模式和摇杆入口不属于标准 Profile；UDP 继续只用于遥测。
- 本地验证：临时工程副本经 Tuanjie 编译，8/8 个 UR10 定向回归通过，63 个相关 EditMode 直连用例通过。直连运行器跳过两个与既有 UDP/键盘控制改动冲突的用例，以及一个依赖 Unity TestRunner `LogAssert` scope 的记录器用例。
- Quest Build & Run：尚未执行。真实机器人输出仍未启用。

## 固定交互
- 右手 Grip：完整 6DoF
- 松开 Grip：立即保持
- 右手 Trigger：夹爪
- 左手 Grip + 短按 X：保持当前 TCP 位置，只把物理抓取坐标系转为竖直向下
- 左手 Grip + 长按 X 0.45 s：Ready Pose；松开 X 可取消
- 右手 Grip 与左手安全姿态互斥；安全姿态结束后右手必须松开并重新按 Grip
- A/B、左摇杆、左手 Y：不生成运动

## 固定初始参数
| 参数 | 值 |
|---|---:|
| 平移近端增益 | 0.15 |
| 平移远端增益 | 0.80 |
| 平移过渡距离 | 0.040 m |
| 旋转近端增益 | 0.25 |
| 旋转远端增益 | 0.80 |
| 旋转过渡角 | 20° |
| 位置时间常数 | 0.055 s |
| 旋转时间常数 | 0.055 s |

## 本地测试结果
- 日期：2026-07-29
- 提交 SHA：`21a87734b73a8f893c1c77ae4d311b8f81f92140`
- 目标测试类：`Ur5Continuous6DofClutchControllerTests`、`Ur5Continuous6DofBootstrapTests`、`Ur5RelativePoseClutchMapperTests`、`Ur5PoseMathTestBenchTests`、`Ur5ClutchModeControllerTests`
- 测试 XML：本地 Tuanjie 命令行环境未生成 XML；combined 目标路径为 `/tmp/ur5-single-mode-all-tests.xml`，逐类目标路径为 `/tmp/<TestClass>.xml`
- 测试日志：`/tmp/ur5-single-mode-all-tests.log`、`/tmp/Ur5Continuous6DofClutchControllerTests.log`、`/tmp/Ur5Continuous6DofBootstrapTests.log`、`/tmp/Ur5RelativePoseClutchMapperTests.log`、`/tmp/Ur5PoseMathTestBenchTests.log`、`/tmp/Ur5ClutchModeControllerTests.log`、`/tmp/ur5-task6-direct.log`
- 通过数量：44/44（direct runner 明确输出 `PASS 44/44 Task 6 local validation`）
- 失败数量：0
- 备注：官方 combined `-runTests` 和五个逐类 `-runTests` 均 exit 0，逐类日志未发现 `error CS` 或 `Scripts have compiler errors`；XML 未生成是当前本地 Tuanjie CLI 的已知表现。

## Android 编译前静态条件
- Quest Profile：`ApplyDefaultQuestTeleopProfile` 明确设置 `enableContinuous6DofClutch=true`、`positionControllerNode=RightHand`、`rotationControllerNode=RightHand`、`safetyControllerNode=LeftHand`、`enableThreeModeController=false`、`snapGraspApproachToVertical=false`
- 真实机器人输出：未把 `enableRealRobotOutput` 默认值或运行配置改为 true；当前命中仅为字段声明、读取和安全判断
- 单级滤波和单写入源：`ApplyContinuous6DofPreview` 只调用一次 `FilterByTimeConstants`，不调用 `SnapGraspApproachToVertical` 或 `AdvancePreview*`，只记录一次 `QuestTeleopContinuous6Dof`
- 诊断记录：Android 自动开始，采样间隔 0.02 s，每 1 s 及暂停、退出、禁用时覆盖保存完整 CSV 快照
- 工作区状态：诊断改动尚未提交；Package、ProjectSettings 和 Windows `.meta` 的既有脏文件未被还原、覆盖或纳入本轮修改

## 首次 Quest 失败记录
- 日期：2026-07-29
- 现象：右手柄保持静止并持续按住 Grip 时，肩、肘、腕关节都会持续剧烈摆动
- 对照：按住 Grip 并主动移动时相对稳定；松开 Grip 后摆动停止并恢复正常
- 当前结论：尚未定位根因；首次 Build 没有自动生成可用 CSV，不能在无证据时归因或调参
- 下一步：部署只读诊断版本，按输入、逻辑目标、滤波目标、IK 目标、Drive 目标、实测关节的顺序定位最早异常层

## 诊断版本本地回归
- 日期：2026-07-30
- 工作树基线 SHA：`e20fd7a2b9dd2a6e62cc62ab2ef23a91b85e4566`（诊断改动尚未提交）
- 测试类：`Ur5LeftSafetyPoseControllerTests`、`Ur5Continuous6DofClutchControllerTests`、`Ur5Continuous6DofBootstrapTests`、`Ur5RelativePoseClutchMapperTests`、`Ur5PoseMathTestBenchTests`、`Ur5ClutchModeControllerTests`
- 直接回归：57/57 个可直接调用的 EditMode 用例通过；另 1 个保存错误日志用例由等价生命周期断言通过
- Task 4 记录器集成：13/13，通过 Android-only 自动策略、CSV schema、暂停/重复落盘、单表头、失败日志节流和 Bootstrap 固定值
- 官方 CLI：combined `-runTests` exit 0，Tundra 编译成功且无 `error CS`；当前 Tuanjie CLI 仍未生成 XML，因此不把该退出码虚报为官方测试计数
- 测试日志：`/tmp/ur5-task7-final-direct.log`、`/tmp/ur5-task7-final-editmode.log`
- 参数审查：连续 6DoF 增益、DLS、IK 容差、关节速度和 Articulation Drive 参数均未改变

## 诊断 Build 门禁
- 状态：本地验证完成，等待用户再次确认 Build & Run
- Build SHA：等待 Build & Run
- Quest 刷新率：等待 Build & Run
- 六轴结果：等待 Build & Run
- 松手结果：等待 Build & Run
- Fault 结果：等待 Build & Run
- 主观手感：等待 Build & Run
- 日志路径：等待 Build & Run
- 固定复现时序：5 s 松手基线 -> 10 s 静止 Grip -> 10 s 移动 Grip -> 5 s 松手恢复
- 安全动作检查：右手松开 Grip 后，验证左手 Grip + 短按 X 朝下和长按 X Ready Pose

## 禁止事项
- 不连接真实 UR5。
- 不新增模式。
- 不在无日志证据时调参。

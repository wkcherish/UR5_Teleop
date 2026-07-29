# UR5 单模式 6DoF 验证记录

## 固定交互
- 右手 Grip：完整 6DoF
- 松开 Grip：立即保持
- 右手 Trigger：夹爪
- 左手 X：仅 Grip 未按下时复位
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
- Quest Profile：`ApplyDefaultQuestTeleopProfile` 明确设置 `enableContinuous6DofClutch=true`、`rotationControllerNode=RightHand`、`enableThreeModeController=false`、`snapGraspApproachToVertical=false`
- 真实机器人输出：未把 `enableRealRobotOutput` 默认值或运行配置改为 true；当前命中仅为字段声明、读取和安全判断
- 单级滤波和单写入源：`ApplyContinuous6DofPreview` 只调用一次 `FilterByTimeConstants`，不调用 `SnapGraspApproachToVertical` 或 `AdvancePreview*`，只记录一次 `QuestTeleopContinuous6Dof`
- 工作区状态：Task 6 变更仅包含 `docs/teleop_single_mode_6dof_validation.md`

## Quest 验证结果
- 状态：等待用户确认 Build & Run
- Build SHA：等待 Build & Run
- Quest 刷新率：等待 Build & Run
- 六轴结果：等待 Build & Run
- 松手结果：等待 Build & Run
- Fault 结果：等待 Build & Run
- 主观手感：等待 Build & Run
- 日志路径：等待 Build & Run

## 禁止事项
- 不连接真实 UR5。
- 不新增模式。
- 不在无日志证据时调参。

# UR5 Quest Static Oscillation Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变 6DoF、IK 或 Articulation 参数的前提下，为 Quest 静止摆动采集可定位根因的完整日志，并恢复独立、可验证的左手短按 X 朝下与长按 X Ready Pose 控制。

**Architecture:** 新增纯逻辑 `Ur5LeftSafetyPoseController` 管理短按、长按、朝下收敛、超时和右手互斥；`Ur5CartesianVelocityTeleopController` 只负责读取独立左手设备、计算物理抓取帧目标并应用单一安全写入。现有记录器追加只读诊断字段，在 Android 自动记录并周期/生命周期落盘，控制增益和 IK 参数保持原值。

**Tech Stack:** Tuanjie Editor 2022.3.62t11 / Tuanjie 1.9.3、Unity C#、UnityEngine.XR、NUnit EditMode、Quest 3 Android、ADB。

## Global Constraints

- 工程根目录固定为 `/Users/imi-1/Desktop/project/ur5`，分支固定为 `fix/teleop-pose-math-testbench`。
- 设计依据为 `docs/superpowers/specs/2026-07-29-ur5-quest-static-oscillation-diagnostics-design.md`。
- 本轮不修改 `Ur5Continuous6DofConfig`、DLS、IK 容差、关节速度或 Articulation Drive 参数。
- 关键输入所有权、超时停止、四元数与抓取坐标系转换必须保留必要中文注释。
- `enableRealRobotOutput` 保持 `false`；不连接真实 UR5，不启动 Python RTDE。
- 不修改或提交现有 `ProjectSettings/ProjectSettings.asset`、Package 改动和两个 Windows `.meta` 删除。
- 当前工作区已有用户改动；所有 `git add` 必须使用显式文件路径。
- 未经用户明确许可，不执行本地提交或远程推送；计划中的提交步骤只能先汇报并请求许可。
- 未经用户再次确认，不执行 Quest Build & Run。
- 当前 GUI 工程不直接运行 batchmode；命令行测试必须使用安全临时工程副本。

每轮命令行测试前用以下命令创建副本；`UR5_TEST_COPY` 只指向 `mktemp` 返回目录下的确定子目录：

```bash
UR5_TEST_PARENT="$(mktemp -d /tmp/ur5-task7-tests.XXXXXX)"
UR5_TEST_COPY="$UR5_TEST_PARENT/ur5"
mkdir "$UR5_TEST_COPY"
rsync -a \
  --exclude '.git/' \
  --exclude 'Library/' \
  --exclude 'Temp/' \
  --exclude 'Logs/' \
  --exclude 'obj/' \
  --exclude 'Builds/' \
  ./ "$UR5_TEST_COPY/"
test -f "$UR5_TEST_COPY/ProjectSettings/ProjectVersion.txt"
```

最后一条命令必须成功后才能把该目录交给 Tuanjie。测试结束只保留 `/tmp` 日志和 XML；不对当前工程执行清理命令。

---

## File Map

### 新增文件

- `Assets/Scripts/UR5/ControlLogic/Ur5LeftSafetyPoseController.cs`
  - 纯逻辑管理左手短按/长按、朝下命令生命周期、超时和互斥。
- `Assets/Tests/EditMode/Editor/Ur5LeftSafetyPoseControllerTests.cs`
  - 验证状态机，不依赖 XR 设备或场景。

Unity/Tuanjie 生成的对应 `.meta` 只与新增 C# 文件一起纳入后续授权提交。

### 修改文件

- `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`
  - 读取独立左手安全设备，接入纯逻辑状态机，应用朝下目标并暴露原始姿态诊断。
- `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs`
  - 暴露只读静止/IK 诊断属性。
- `Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs`
  - Android 自动记录、定期/生命周期落盘和追加诊断列。
- `Assets/Scripts/UR5/Ur5ControlBootstrap.cs`
  - 固定左手安全节点及诊断记录默认值，不改变控制参数。
- `Assets/Scripts/UR5/Ur5TeleopDiagnostics.cs`
  - 在节流日志中补充静止保持、原始四元数和奇异性字段。
- `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`
  - 验证启动配置、CSV schema 和自动记录策略。
- `docs/teleop_single_mode_6dof_validation.md`
  - 记录首次 Quest 失败现象和诊断 Build 待验证状态。

---

### Task 1: 左手安全姿态纯逻辑状态机

**Files:**

- Create: `Assets/Scripts/UR5/ControlLogic/Ur5LeftSafetyPoseController.cs`
- Create: `Assets/Tests/EditMode/Editor/Ur5LeftSafetyPoseControllerTests.cs`

**Interfaces:**

- Produces: `Ur5LeftSafetyPoseState`，值为 `Idle`、`ButtonHeld`、`SnapDownActive`、`ReadyPoseActive`。
- Produces: `Ur5LeftSafetyPoseStepInput(bool rightGripHeld, bool leftPoseValid, bool leftGripHeld, bool primaryPressed, bool snapTargetReached, bool readyPoseActive, float deltaTimeSeconds)`。
- Produces: `Ur5LeftSafetyPoseStepResult`，至少包含 `State`、`BlocksRightGrip`、`RequestSnapDown`、`RequestReadyPose`、`CancelReadyPose`、`SnapTimedOut`。
- Produces: `Ur5LeftSafetyPoseController(float readyPoseHoldSeconds, float snapTimeoutSeconds)`、`Step(...)` 和 `Reset()`。

- [ ] **Step 1: 写短按、长按和互斥失败测试**

创建测试，核心断言如下：

```csharp
[Test]
public void ShortPress_RequestsOneSnapAndBlocksRightGripUntilReached()
{
    var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);

    Ur5LeftSafetyPoseStepResult down = controller.Step(Input(
        rightGripHeld: false, leftGripHeld: true, primaryPressed: true, deltaTime: 0.10f));
    Assert.AreEqual(Ur5LeftSafetyPoseState.ButtonHeld, down.State);
    Assert.IsTrue(down.BlocksRightGrip);

    Ur5LeftSafetyPoseStepResult released = controller.Step(Input(
        rightGripHeld: false, leftGripHeld: true, primaryPressed: false, deltaTime: 0.01f));
    Assert.IsTrue(released.RequestSnapDown);
    Assert.AreEqual(Ur5LeftSafetyPoseState.SnapDownActive, released.State);

    Ur5LeftSafetyPoseStepResult reached = controller.Step(Input(
        rightGripHeld: false, leftGripHeld: false, primaryPressed: false,
        snapTargetReached: true, deltaTime: 0.02f));
    Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, reached.State);
}

[Test]
public void LongPress_RequestsReadyPoseInsteadOfSnapDown()
{
    var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);
    controller.Step(Input(false, true, true, false, false, 0.20f));
    controller.Step(Input(false, true, true, false, false, 0.20f));
    Ur5LeftSafetyPoseStepResult result =
        controller.Step(Input(false, true, true, false, false, 0.06f));

    Assert.IsTrue(result.RequestReadyPose);
    Assert.IsFalse(result.RequestSnapDown);
    Assert.AreEqual(Ur5LeftSafetyPoseState.ReadyPoseActive, result.State);
}

[Test]
public void RightGripHeld_PreventsLeftSafetyPoseFromStarting()
{
    var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);
    Ur5LeftSafetyPoseStepResult result =
        controller.Step(Input(true, true, true, false, false, 0.50f));

    Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, result.State);
    Assert.IsFalse(result.BlocksRightGrip);
}
```

另加测试：无效左手位姿不启动；短按超时 3 秒只发出一次 `SnapTimedOut`；Ready Pose 未完成时松开 X 发出一次 `CancelReadyPose`；重置后回到 Idle。

- [ ] **Step 2: 运行测试并确认因类型不存在而失败**

在不触碰 GUI 工程 `Library` 的安全临时副本运行：

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$UR5_TEST_COPY" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5LeftSafetyPoseControllerTests \
  -testResults /tmp/Ur5LeftSafetyPoseControllerTests-red.xml \
  -logFile /tmp/Ur5LeftSafetyPoseControllerTests-red.log -quit
```

Expected: FAIL，错误指向 `Ur5LeftSafetyPoseController` 或相关类型不存在；不得出现无关 C# 编译错误。

- [ ] **Step 3: 实现最小纯逻辑状态机**

实现固定阈值由构造函数注入并校验为非负；`Step` 只基于输入和累计秒数改变状态。关键规则：

```csharp
if (State == Ur5LeftSafetyPoseState.Idle)
{
    if (!input.RightGripHeld
        && input.LeftPoseValid
        && input.LeftGripHeld
        && input.PrimaryPressed)
    {
        State = Ur5LeftSafetyPoseState.ButtonHeld;
        heldSeconds = 0.0f;
    }
}

// 短按必须等到 X 释放后才发朝下请求，避免长按过程中先旋转再切 Ready Pose。
if (State == Ur5LeftSafetyPoseState.ButtonHeld && !input.PrimaryPressed)
{
    State = Ur5LeftSafetyPoseState.SnapDownActive;
    snapElapsedSeconds = 0.0f;
    requestSnapDown = true;
}
```

长按只触发一次 `RequestReadyPose`；朝下达到目标或超时后返回 Idle；所有结果使用 readonly 字段，避免 Unity 适配层修改状态机内部数据。

- [ ] **Step 4: 运行状态机测试并确认全部通过**

Run: 与 Step 2 相同，结果路径改为 `/tmp/Ur5LeftSafetyPoseControllerTests.xml`。

Expected: 目标测试 0 FAIL，日志无 `error CS`。

- [ ] **Step 5: 审查任务差异并暂停提交**

```bash
git diff --check -- \
  Assets/Scripts/UR5/ControlLogic/Ur5LeftSafetyPoseController.cs \
  Assets/Tests/EditMode/Editor/Ur5LeftSafetyPoseControllerTests.cs
```

Expected: 无 whitespace error。不得提交；等全部诊断版本完成后统一向用户请求本地提交许可。

---

### Task 2: 独立左手设备与朝下 IK 所有权

**Files:**

- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`
- Modify: `Assets/Scripts/UR5/Ur5ControlBootstrap.cs`
- Modify: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`
- Test: `Assets/Tests/EditMode/Editor/Ur5LeftSafetyPoseControllerTests.cs`

**Interfaces:**

- Consumes: Task 1 的 `Ur5LeftSafetyPoseController.Step(...)`。
- Produces: `public XRNode safetyControllerNode`，Quest 默认值为 `XRNode.LeftHand`。
- Produces: `public bool IsSafetyPoseCommandActive` 和 `public Ur5LeftSafetyPoseState SafetyPoseState`。
- Produces: `public Quaternion RawControllerRotationWorld`，只读返回右手世界四元数。

- [ ] **Step 1: 写启动配置和控制所有权失败测试**

在 `DefaultQuestProfile_UsesOnlyRightHandContinuous6Dof` 增加：

```csharp
Assert.AreEqual(XRNode.LeftHand, teleop.safetyControllerNode);
Assert.AreEqual(0.45f, teleop.leftPrimaryReadyPoseHoldSeconds, 0.000001f);
Assert.AreEqual(3.0f, teleop.leftPrimarySnapTimeoutSeconds, 0.000001f);
Assert.AreEqual(0.003f, teleop.leftPrimarySnapPositionToleranceMeters, 0.000001f);
Assert.AreEqual(0.50f, teleop.leftPrimarySnapRotationToleranceDegrees, 0.000001f);
```

为主控制器增加不依赖 XR 设备的适配测试入口或 internal 方法测试，验证：

- `SnapDownActive` 时 `IsCommandActive=true`，follower 不进入 idle hold；
- 安全命令激活时 continuous controller 被 Pause；
- 朝下写入只使用 `GetToolRotationForGraspApproach(Vector3.down, baseForward)`；
- 超时时调用一次 `FreezeAtCurrentPose()` 语义，不继续写目标。

- [ ] **Step 2: 运行目标测试并确认新字段/行为缺失导致失败**

Run filters:

```text
Ur5LeftSafetyPoseControllerTests;Ur5Continuous6DofBootstrapTests
```

Expected: FAIL，缺少 `safetyControllerNode` 或安全命令适配属性。

- [ ] **Step 3: 接入独立左手安全设备**

在主控制器新增：

```csharp
[Header("Left Controller Safety Pose")]
public XRNode safetyControllerNode = XRNode.LeftHand;
public float leftPrimarySnapTimeoutSeconds = 3.0f;
public float leftPrimarySnapPositionToleranceMeters = 0.003f;
public float leftPrimarySnapRotationToleranceDegrees = 0.50f;

private InputDevice safetyDevice;
private bool safetyGripLatched;
```

`Start` 和设备刷新路径独立调用 `InputDevices.GetDeviceAtXRNode(safetyControllerNode)`。`UpdateLeftControllerSafetyPose` 只读 `safetyDevice`，不得再检查或复用 `rotationControllerNode`/`rotationDevice`。

Quest Profile 显式设置：

```csharp
velocityTeleop.safetyControllerNode = UnityEngine.XR.XRNode.LeftHand;
velocityTeleop.leftPrimaryReadyPoseHoldSeconds = 0.45f;
velocityTeleop.leftPrimarySnapTimeoutSeconds = 3.0f;
velocityTeleop.leftPrimarySnapPositionToleranceMeters = 0.003f;
velocityTeleop.leftPrimarySnapRotationToleranceDegrees = 0.50f;
```

- [ ] **Step 4: 接入短按朝下和长按 Ready Pose**

按以下顺序处理：

1. Update 读取右 Grip 状态后调用安全状态机；
2. `RequestReadyPose` 调用 follower 的 `BeginReadyPose()`；
3. `RequestSnapDown` 捕获当前实际 TCP 位置，并调用
   `GetToolRotationForGraspApproach(Vector3.down, robotBaseFrame.forward)` 生成目标；
4. `FixedUpdate` 中安全朝下写入优先于 continuous 6DoF，写入者标记为 `LeftSafetySnapDown`；
5. `IsCommandActive` 在 `SnapDownActive` 时返回 true，使 follower 继续 IK；
6. 达到位置 `0.003 m` 和姿态 `0.50°` 后停止安全写入；
7. 3 秒超时调用一次 `FreezeAtCurrentPose()` 并保持；
8. 安全状态持有写入权时调用 `continuous6DofController.Pause()`，要求右 Grip 释放后重建基准。

关键所有权注释使用中文，明确短按在释放 X 后才开始、超时必须停止、两条路径不能同帧写 `TcpTarget`。

- [ ] **Step 5: 运行目标测试并确认通过**

Expected: 两个目标测试类 0 FAIL；原 Profile 仍保持 position/rotation 为 RightHand，安全节点为 LeftHand。

- [ ] **Step 6: 审查没有改变连续与 IK 参数**

```bash
git diff -- Assets/Scripts/UR5/Ur5ControlBootstrap.cs \
  Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs
rg -n "dlsDamping|dlsGain|rotationToleranceDegrees|continuous6DofConfig" \
  Assets/Scripts/UR5/Ur5ControlBootstrap.cs
```

Expected: 只增加安全输入参数；原连续配置和 follower 参数值不变。

---

### Task 3: 只读静止与 IK 诊断接口

**Files:**

- Modify: `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs`
- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`
- Modify: `Assets/Scripts/UR5/Ur5TeleopDiagnostics.cs`
- Modify: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`

**Interfaces:**

- Produces: `Ur5TcpTargetFollower.TargetStationarySeconds`。
- Produces: `Ur5TcpTargetFollower.IsSettledTargetHoldActive`。
- Reuses: `IsNearSingularity`、`LastDlsMinimumPivot`、`IkFailureCount`、`WasIkCommandLeadLimited`。
- Reuses: `Ur5ArticulationJointController.GetJointTargetDegrees`、`GetDriveTargetDegrees`、`GetMeasuredJointDegrees`。

- [ ] **Step 1: 写诊断日志失败测试**

扩展 `DiagnosticsLog_ReportsContinuousFieldsAndLegacyModeLabel`：

```csharp
Assert.That(capturedLog, Does.Contain("rawHandRotation="));
Assert.That(capturedLog, Does.Contain("stationarySeconds="));
Assert.That(capturedLog, Does.Contain("settledHold="));
Assert.That(capturedLog, Does.Contain("nearSingularity="));
```

增加反射测试确认上述 follower 属性只有 getter，没有 public setter，保证记录行为只读。

- [ ] **Step 2: 运行测试并确认缺少字段导致失败**

Expected: FAIL at `rawHandRotation=` 或属性不存在。

- [ ] **Step 3: 添加只读属性和节流日志**

主控制器返回 `latestRotationWorld`：

```csharp
public Quaternion RawControllerRotationWorld => latestRotationValid
    ? latestRotationWorld
    : Quaternion.identity;
```

Follower 只暴露现有私有状态：

```csharp
public float TargetStationarySeconds => targetStationaryTime;
public bool IsSettledTargetHoldActive => isSettledTargetHoldActive;
```

诊断日志追加原始四元数、静止秒数、保持锁存和近奇异状态，不在 getter 中调用 `ResolveReferences`、IK 或任何写入方法。

- [ ] **Step 4: 运行诊断测试并确认通过**

Expected: `Ur5Continuous6DofBootstrapTests` 0 FAIL。

---

### Task 4: Quest 自动 CSV 与可靠落盘

**Files:**

- Modify: `Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs`
- Modify: `Assets/Scripts/UR5/Ur5ControlBootstrap.cs`
- Modify: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`

**Interfaces:**

- Produces: `public bool autoRecordOnAndroid = true`。
- Produces: `public float flushIntervalSeconds = 1.0f`。
- Produces: `public static bool ShouldAutoStartRecording(RuntimePlatform platform)`。
- Produces: `public void SaveRecordingSnapshot()`，无样本或未初始化时安全返回。

- [ ] **Step 1: 写自动启动、落盘幂等和 schema 失败测试**

增加：

```csharp
[Test]
public void PoseCsvRecorder_AutoStartsOnlyOnAndroid()
{
    Assert.IsTrue(Ur5PoseCsvRecorder.ShouldAutoStartRecording(RuntimePlatform.Android));
    Assert.IsFalse(Ur5PoseCsvRecorder.ShouldAutoStartRecording(RuntimePlatform.OSXEditor));
}
```

将 CSV tail 断言更新为完整新增尾部，至少包含：

```text
raw_hand_rot_x,raw_hand_rot_y,raw_hand_rot_z,raw_hand_rot_w,
logical_rot_x,logical_rot_y,logical_rot_z,logical_rot_w,
filtered_rot_x,filtered_rot_y,filtered_rot_z,filtered_rot_w,
actual_grasp_rot_x,actual_grasp_rot_y,actual_grasp_rot_z,actual_grasp_rot_w,
target_stationary_s,settled_hold,dls_min_pivot,near_singularity,ik_failure_count,ik_lead_limited,
j1_drive_deg,...,j6_drive_deg,j1_measured_deg,...,j6_measured_deg
```

测试表头列数等于样本列数；连续调用 `SaveRecordingSnapshot()` 不改变 StringBuilder 内容和表头数量。

- [ ] **Step 2: 运行测试并确认失败**

Expected: FAIL，因为自动策略、保存 API 或新列不存在。

- [ ] **Step 3: 实现 Android 自动开始与定期落盘**

`Start` 初始化表头和路径后：

```csharp
if (autoRecordOnAndroid && ShouldAutoStartRecording(Application.platform))
{
    StartRecording();
}
```

采样后若到达下一次落盘时间则调用 `SaveRecordingSnapshot()`。以下生命周期只在 CSV 已初始化且有内容时保存：

```csharp
private void OnApplicationPause(bool paused)
{
    if (paused) SaveRecordingSnapshot();
}

private void OnApplicationQuit() => SaveRecordingSnapshot();
private void OnDisable() => SaveRecordingSnapshot();
```

保存继续覆盖同一路径的完整快照，不清空 StringBuilder，避免重复表头和部分行。捕获 `IOException`/`UnauthorizedAccessException` 时输出一次明确错误，不改变控制状态。

- [ ] **Step 4: 追加完整诊断列**

所有新列只追加到现有 `logical_to_filtered_rotation_deg` 之后。四元数按 `x,y,z,w` 写入；六个关节按命令、Drive、实测三组分别输出。空引用输出零值和 false，不抛异常。

- [ ] **Step 5: 显式配置 Quest 记录器**

`ConfigureRecorder` 设置：

```csharp
recorder.autoRecordOnAndroid = true;
recorder.flushIntervalSeconds = 1.0f;
recorder.sampleInterval = 0.02f;
```

Editor 仍由 `Application.platform` 判断而不自动开始。

- [ ] **Step 6: 运行 CSV/Bootstrap 测试并确认通过**

Expected: `Ur5Continuous6DofBootstrapTests` 0 FAIL，表头与样本行列数一致。

---

### Task 5: 本地回归、验证记录与 Quest 门禁

**Files:**

- Modify: `docs/teleop_single_mode_6dof_validation.md`
- Review only: all files from Tasks 1-4

**Interfaces:**

- Consumes: Tasks 1-4 的安全姿态和诊断版本。
- Produces: 本地测试证据、诊断 Build 待确认状态和明确的 Quest 复现步骤。

- [ ] **Step 1: 运行新增目标测试**

安全临时工程副本中运行：

```text
Ur5LeftSafetyPoseControllerTests
Ur5Continuous6DofBootstrapTests
```

Expected: 0 FAIL，日志无 `error CS` 或 `Scripts have compiler errors`。

- [ ] **Step 2: 运行 9.3 回归测试**

运行六个测试类：

```text
Ur5LeftSafetyPoseControllerTests
Ur5Continuous6DofClutchControllerTests
Ur5Continuous6DofBootstrapTests
Ur5RelativePoseClutchMapperTests
Ur5PoseMathTestBenchTests
Ur5ClutchModeControllerTests
```

Expected: 全部 0 FAIL；若 Tuanjie 不生成 XML，保留每类 exit code 和日志，并使用已有 direct runner 等价验证路径。

- [ ] **Step 3: 静态安全审查**

```bash
rg -n "enableRealRobotOutput" Assets/Scripts/UR5
rg -n "safetyControllerNode|positionControllerNode|rotationControllerNode" \
  Assets/Scripts/UR5/Ur5ControlBootstrap.cs
git diff --check
git status --short
```

Expected: 真实输出未被设为 true；右手 position/rotation 与左手 safety 节点明确分离；无 whitespace error；用户脏文件未被覆盖。

- [ ] **Step 4: 更新验证记录**

在验证文档记录：

- 首次 Quest 失败：Grip 静止时全臂持续摆动，移动时相对稳定，松手恢复；
- 当前根因状态：等待诊断 CSV，不写成已定位；
- 诊断版本测试数量、日志路径和当前 SHA；
- Build & Run 状态：等待用户再次确认；
- 固定复现时序：5 秒松手、10 秒静止 Grip、10 秒移动 Grip、5 秒松手。

- [ ] **Step 5: 汇报并请求 Build & Run**

汇报新增/回归测试、`enableRealRobotOutput=false`、未改变参数、工作区脏文件隔离情况。获得用户确认前停止，不点击 Build And Run。

- [ ] **Step 6: 用户确认后 Build & Run 并拉取证据**

只使用 Tuanjie GUI Build And Run。复现后：

```bash
ADB="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb"
"$ADB" pull \
  /sdcard/Android/data/com.dgvlalab.ur5quest/files/ur5_pose_log.csv \
  /tmp/ur5_pose_log_task7_diagnostics.csv
"$ADB" logcat -d -v threadtime > /tmp/ur5_task7_diagnostics_logcat.txt
```

Expected: CSV 存在、至少覆盖完整复现区间、每行列数一致；日志无真实机器人连接。

- [ ] **Step 7: 按证据形成单一根因假设**

严格按设计第 4.3 节从原始输入到 Drive 逐层比较。只汇报命中的第一处异常，不在同一步调多个参数。

- [ ] **Step 8: 请求本地提交许可**

只有诊断版本本地测试通过且用户同意时，才执行显式暂存；不得夹带工作区已有脏文件。建议提交信息：

```text
test(teleop): add quest oscillation diagnostics
```

远程 push 仍需另行取得用户明确许可。

---

## Final Review Checklist

- [ ] 短按 X 只在释放后启动朝下，长按不会先触发朝下。
- [ ] 朝下命令保持当前 TCP 位置，只改变物理抓取帧朝向。
- [ ] 朝下达到 `0.003 m / 0.50°` 后停止，3 秒超时安全保持。
- [ ] Ready Pose 保持现有长按与松开取消语义。
- [ ] 右手 Grip 和左手安全姿态不会同帧写 `TcpTarget`。
- [ ] 右手位置/旋转节点仍为 RightHand，安全节点独立为 LeftHand。
- [ ] CSV 在 Android 自动启动，每 1 秒及暂停/退出时落盘。
- [ ] 新诊断列全部追加在旧 schema 尾部，表头和数据列数一致。
- [ ] 诊断 getter 只读，不调用控制或 IK 写入。
- [ ] 连续 6DoF、DLS、IK 和 Drive 参数未改变。
- [ ] `enableRealRobotOutput=false`，未连接真实机器人。
- [ ] 用户现有 Package、ProjectSettings 和 Windows `.meta` 改动未提交或还原。
- [ ] Build & Run 与本地/远程提交均经过用户单独确认。

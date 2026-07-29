# UR5 单模式 6DoF 离合控制 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 Quest3—UR5 标准遥操作收敛为右手 Grip 驱动的单模式 6DoF 相对位姿控制，使小动作精细、大动作连续提速、三轴旋转有效，并保证松手即停与安全异常默认停止。

**Architecture:** 新增纯逻辑的版本化连续响应配置和离合状态机，从固定的右手柄/实际 TCP 基准计算完整位置与四元数目标。Unity 主控制器只负责 XR 输入、工作空间约束、单级时间不变滤波和 `TcpTarget` 写入；旧三模式与左手旋转分支保留为兼容代码，但 Quest 默认运行链路不再进入。

**Tech Stack:** Tuanjie Editor 2022.3.62t11 / Tuanjie 1.9.3、Unity C#、UnityEngine.XR、NUnit EditMode tests、Quest 3 Android build。

## Global Constraints

- Unity 工程根目录固定为 `/Users/imi-1/Desktop/project/ur5`。
- 所有实现和提交直接在当前分支 `fix/teleop-pose-math-testbench` 完成，不为 9.3 创建新功能分支。
- 设计依据固定为 `docs/superpowers/specs/2026-07-29-ur5-single-mode-6dof-clutch-design.md`，设计提交为 `c76e107`。
- 标准交互只有右手 Grip + 右手柄完整位置/四元数；A/B、左摇杆、左手 Y 不切换或生成标准运动。
- 小动作与大动作使用同一条连续 SmoothStep 响应曲线，不得新增运动模式。
- Grip 控制期间关闭自动竖直吸附和姿态轴锁；左手 X 只在右手 Grip 未按下时执行显式安全姿态复位。
- 右手扳机夹爪控制保持不变。
- 只保留一层最终位姿滤波；不得叠加控制器低通、目标死区、第二级 TCP 平滑或松手后的加速度追赶。
- `enableRealRobotOutput` 必须保持 `false`；本计划的 Quest 验证只控制 Unity 虚拟 UR5。
- 不修改用户当前的 `ProjectSettings/ProjectSettings.asset`。
- 不提交 Tuanjie 自动删除的
  `Packages/com.unity.robotics.urdf-importer/Runtime/UnityMeshImporter/Plugins/AssimpNet/Native/win/x86.meta`
  和
  `Packages/com.unity.robotics.urdf-importer/Runtime/UnityMeshImporter/Plugins/AssimpNet/Native/win/x86_64.meta`。
- 关键四元数顺序、安全锁存和输入所有权必须添加必要中文注释；明显代码不添加复述式注释。
- 每个实现任务采用测试驱动：先写失败测试、确认失败、最小实现、确认通过、审查差异、独立提交。
- 优先在当前工程使用 Tuanjie Test Runner 执行 EditMode 测试。计划中的命令行测试是可复现等价命令；不得在当前 GUI 工程目录直接运行 batchmode，以免改写 `Library/LastSceneManagerSetup.txt`。必须使用命令行时，使用不创建 Git 分支的安全临时工程副本。
- 到达 Quest 3 Build & Run 节点时必须先向用户汇报并等待确认，不得自行执行。

---

## File Map

### 新增文件

- `Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofConfig.cs`
  - 定义版本化单模式参数、默认值和参数校验。
- `Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofClutchController.cs`
  - 定义输入、结果、Fault 原因和 `Idle/Clutched/Paused/Fault` 纯逻辑状态机。
- `Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs`
  - 验证连续增益、六自由度映射、离合、Fault 和四元数行为。
- `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`
  - 验证 Quest 默认配置只启用右手单模式链路。
- `docs/teleop_single_mode_6dof_validation.md`
  - 保存本地与 Quest 验证清单、结果字段和唯一参数表。

Unity/Tuanjie 会为 `Assets/` 下新增的 C# 文件生成 `.meta`；根目录 `docs/` 不属于 Unity Asset Database，不应创建或提交 `.meta`。

### 修改文件

- `Assets/Scripts/UR5/Ur5RelativePoseCommandFilter.cs`
  - 增加按秒表示的时间常数滤波与硬速度限幅，同时保留旧 retention API 供兼容路径使用。
- `Assets/Tests/EditMode/Editor/Ur5RelativePoseClutchMapperTests.cs`
  - 增加 72/90/120 Hz 时间常数滤波一致性、线速度和角速度限幅测试。
- `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`
  - 接入右手单模式输入与目标链路；旧链路只在兼容开关关闭时执行。
- `Assets/Scripts/UR5/Ur5ControlBootstrap.cs`
  - 将 Quest 默认配置切换为右手完整 6DoF，关闭三模式、左手旋转和自动吸附。
- `Assets/Scripts/UR5/Ur5TeleopDiagnostics.cs`
  - 输出单模式状态、Fault、相对距离/角度、连续增益与姿态滤波差。
- `Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs`
  - 追加单模式诊断列，保留原列以避免破坏旧日志读取器。

---

### Task 1: 版本化单模式配置

**Files:**

- Create: `Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofConfig.cs`
- Create: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs`

**Interfaces:**

- Produces: `Ur5Continuous6DofConfig.Default`
- Produces: `Ur5Continuous6DofConfig Sanitized()`
- Produces: `float EvaluateTranslationGain(float controllerDistanceMeters)`
- Produces: `float EvaluateRotationGain(float controllerAngleDegrees)`

- [ ] **Step 1: 写配置默认值和连续响应的失败测试**

在新测试类中先加入以下测试：

```csharp
using NUnit.Framework;
using UnityEngine;

public class Ur5Continuous6DofClutchControllerTests
{
    [Test]
    public void DefaultConfig_UsesApprovedSingleModeParameters()
    {
        Ur5Continuous6DofConfig config = Ur5Continuous6DofConfig.Default;

        Assert.AreEqual(1, config.Version);
        Assert.AreEqual(0.15f, config.TranslationNearGain, 0.000001f);
        Assert.AreEqual(0.80f, config.TranslationFarGain, 0.000001f);
        Assert.AreEqual(0.040f, config.TranslationTransitionMeters, 0.000001f);
        Assert.AreEqual(0.25f, config.RotationNearGain, 0.000001f);
        Assert.AreEqual(0.80f, config.RotationFarGain, 0.000001f);
        Assert.AreEqual(20.0f, config.RotationTransitionDegrees, 0.000001f);
        Assert.AreEqual(0.055f, config.PositionTimeConstantSeconds, 0.000001f);
        Assert.AreEqual(0.055f, config.RotationTimeConstantSeconds, 0.000001f);
    }

    [Test]
    public void ContinuousGains_AreMonotonicAndMatchBothEndpoints()
    {
        Ur5Continuous6DofConfig config = Ur5Continuous6DofConfig.Default;

        Assert.AreEqual(0.15f, config.EvaluateTranslationGain(0.0f), 0.000001f);
        Assert.AreEqual(0.80f, config.EvaluateTranslationGain(0.040f), 0.000001f);
        Assert.Less(
            config.EvaluateTranslationGain(0.010f),
            config.EvaluateTranslationGain(0.020f));
        Assert.Less(
            config.EvaluateTranslationGain(0.020f),
            config.EvaluateTranslationGain(0.030f));

        Assert.AreEqual(0.25f, config.EvaluateRotationGain(0.0f), 0.000001f);
        Assert.AreEqual(0.80f, config.EvaluateRotationGain(20.0f), 0.000001f);
        Assert.Less(
            config.EvaluateRotationGain(5.0f),
            config.EvaluateRotationGain(10.0f));
        Assert.Less(
            config.EvaluateRotationGain(10.0f),
            config.EvaluateRotationGain(15.0f));
    }

    [Test]
    public void SanitizedConfig_RejectsInvalidOrInvertedRanges()
    {
        Ur5Continuous6DofConfig config = Ur5Continuous6DofConfig.Default;
        config.TranslationNearGain = -1.0f;
        config.TranslationFarGain = 0.10f;
        config.RotationNearGain = 0.70f;
        config.RotationFarGain = 0.20f;
        config.TranslationTransitionMeters = float.NaN;
        config.PositionTimeConstantSeconds = -0.5f;

        Ur5Continuous6DofConfig sanitized = config.Sanitized();

        Assert.GreaterOrEqual(sanitized.TranslationNearGain, 0.0f);
        Assert.GreaterOrEqual(sanitized.TranslationFarGain, sanitized.TranslationNearGain);
        Assert.GreaterOrEqual(sanitized.RotationFarGain, sanitized.RotationNearGain);
        Assert.Greater(sanitized.TranslationTransitionMeters, 0.0f);
        Assert.Greater(sanitized.PositionTimeConstantSeconds, 0.0f);
    }
}
```

- [ ] **Step 2: 运行目标测试并确认因类型不存在而失败**

在安全临时工程副本中运行以下等价命令；若当前工程使用 GUI Test Runner，则选择同名测试类并核对相同通过/失败结果：

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
"$TUANJIE_EDITOR" \
  -batchmode -nographics \
  -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofClutchControllerTests \
  -testResults /tmp/ur5-continuous-config-tests.xml \
  -logFile /tmp/ur5-continuous-config-tests.log \
  -quit
```

Expected: FAIL，编译错误明确指出 `Ur5Continuous6DofConfig` 不存在。

- [ ] **Step 3: 实现版本化配置**

新文件必须包含 `[Serializable]`，并实现以下字段与方法：

```csharp
using System;
using UnityEngine;

[Serializable]
public struct Ur5Continuous6DofConfig
{
    public int Version;
    public float TranslationNearGain;
    public float TranslationFarGain;
    public float TranslationTransitionMeters;
    public float RotationNearGain;
    public float RotationFarGain;
    public float RotationTransitionDegrees;
    public float PositionTimeConstantSeconds;
    public float RotationTimeConstantSeconds;
    public float MaxClutchTranslationMeters;
    public float MaxClutchRotationDegrees;
    public float MaxLinearSpeedMetersPerSecond;
    public float MaxAngularSpeedDegreesPerSecond;

    public static Ur5Continuous6DofConfig Default => new Ur5Continuous6DofConfig
    {
        Version = 1,
        TranslationNearGain = 0.15f,
        TranslationFarGain = 0.80f,
        TranslationTransitionMeters = 0.040f,
        RotationNearGain = 0.25f,
        RotationFarGain = 0.80f,
        RotationTransitionDegrees = 20.0f,
        PositionTimeConstantSeconds = 0.055f,
        RotationTimeConstantSeconds = 0.055f,
        MaxClutchTranslationMeters = 0.35f,
        MaxClutchRotationDegrees = 45.0f,
        MaxLinearSpeedMetersPerSecond = 0.26f,
        MaxAngularSpeedDegreesPerSecond = 4.0f * Mathf.Rad2Deg
    };

    public float EvaluateTranslationGain(float controllerDistanceMeters)
    {
        Ur5Continuous6DofConfig value = Sanitized();
        float t = Mathf.Clamp01(
            Mathf.Max(0.0f, controllerDistanceMeters) / value.TranslationTransitionMeters);
        return Mathf.Lerp(value.TranslationNearGain, value.TranslationFarGain, SmoothStep01(t));
    }

    public float EvaluateRotationGain(float controllerAngleDegrees)
    {
        Ur5Continuous6DofConfig value = Sanitized();
        float t = Mathf.Clamp01(
            Mathf.Abs(controllerAngleDegrees) / value.RotationTransitionDegrees);
        return Mathf.Lerp(value.RotationNearGain, value.RotationFarGain, SmoothStep01(t));
    }

    private static float SmoothStep01(float value)
    {
        value = Mathf.Clamp01(value);
        return value * value * (3.0f - 2.0f * value);
    }

    public Ur5Continuous6DofConfig Sanitized()
    {
        Ur5Continuous6DofConfig defaults = Default;
        float translationNear = NonNegativeOrDefault(
            TranslationNearGain, defaults.TranslationNearGain);
        float rotationNear = NonNegativeOrDefault(
            RotationNearGain, defaults.RotationNearGain);

        return new Ur5Continuous6DofConfig
        {
            Version = Mathf.Max(1, Version),
            TranslationNearGain = translationNear,
            TranslationFarGain = Mathf.Max(
                translationNear,
                NonNegativeOrDefault(TranslationFarGain, defaults.TranslationFarGain)),
            TranslationTransitionMeters = PositiveOrDefault(
                TranslationTransitionMeters, defaults.TranslationTransitionMeters, 0.0001f),
            RotationNearGain = rotationNear,
            RotationFarGain = Mathf.Max(
                rotationNear,
                NonNegativeOrDefault(RotationFarGain, defaults.RotationFarGain)),
            RotationTransitionDegrees = PositiveOrDefault(
                RotationTransitionDegrees, defaults.RotationTransitionDegrees, 0.01f),
            PositionTimeConstantSeconds = PositiveOrDefault(
                PositionTimeConstantSeconds, defaults.PositionTimeConstantSeconds, 0.001f),
            RotationTimeConstantSeconds = PositiveOrDefault(
                RotationTimeConstantSeconds, defaults.RotationTimeConstantSeconds, 0.001f),
            MaxClutchTranslationMeters = NonNegativeOrDefault(
                MaxClutchTranslationMeters, defaults.MaxClutchTranslationMeters),
            MaxClutchRotationDegrees = NonNegativeOrDefault(
                MaxClutchRotationDegrees, defaults.MaxClutchRotationDegrees),
            MaxLinearSpeedMetersPerSecond = NonNegativeOrDefault(
                MaxLinearSpeedMetersPerSecond, defaults.MaxLinearSpeedMetersPerSecond),
            MaxAngularSpeedDegreesPerSecond = NonNegativeOrDefault(
                MaxAngularSpeedDegreesPerSecond, defaults.MaxAngularSpeedDegreesPerSecond)
        };
    }

    private static float NonNegativeOrDefault(float value, float fallback)
    {
        return IsFinite(value) ? Mathf.Max(0.0f, value) : fallback;
    }

    private static float PositiveOrDefault(float value, float fallback, float minimum)
    {
        return IsFinite(value) ? Mathf.Max(minimum, value) : fallback;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
```

过渡距离、过渡角和时间常数的最小值分别为 `0.0001 m`、`0.01°`、`0.001 s`。

- [ ] **Step 4: 运行配置测试并确认通过**

运行 Step 2 的同一命令。

Expected: `Ur5Continuous6DofClutchControllerTests` 当前 3 个测试全部 PASS。

- [ ] **Step 5: 检查差异并提交**

```bash
git diff --check
git add \
  Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofConfig.cs \
  Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofConfig.cs.meta \
  Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs \
  Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs.meta
git commit -m "feat(teleop): add continuous 6dof configuration"
```

不得暂存 `ProjectSettings.asset` 或两个 Windows `.meta` 删除。

---

### Task 2: 单模式离合、完整四元数和 Fault 锁存

**Files:**

- Create: `Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofClutchController.cs`
- Modify: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs`

**Interfaces:**

- Consumes: `Ur5Continuous6DofConfig`
- Produces: `Ur5Continuous6DofFaultReason`
- Produces: `Ur5Continuous6DofStepInput`
- Produces: `Ur5Continuous6DofStepResult`
- Produces: `Ur5Continuous6DofClutchController.Configure(Ur5Continuous6DofConfig config)`
- Produces: `Ur5Continuous6DofClutchController.Step(Ur5Continuous6DofStepInput input)`
- Produces: `Ur5Continuous6DofClutchController.Pause()`

- [ ] **Step 1: 添加离合、六轴、连续响应和 Fault 的失败测试**

在同一测试类追加以下测试，辅助方法固定实际 TCP 为
`position=(0.40,0.50,0.60)`、`rotation=Quaternion.Euler(5,10,15)`：

```csharp
[Test]
public void GripPressAnchorsWithoutJump_ReleaseStopsMotion()
{
    var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
    Ur5Continuous6DofStepResult anchored = controller.Step(CreateInput(
        gripHeld: true, Vector3.zero, Quaternion.identity));

    Assert.AreEqual(Ur5TeleopControllerState.Clutched, anchored.State);
    Assert.IsTrue(anchored.WasAnchoredThisStep);
    AssertVectorNear(new Vector3(0.40f, 0.50f, 0.60f), anchored.TargetPosition, 0.000001f);

    Ur5Continuous6DofStepResult released = controller.Step(CreateInput(
        gripHeld: false,
        new Vector3(0.20f, 0.0f, 0.0f),
        Quaternion.AngleAxis(30.0f, Vector3.up)));

    Assert.AreEqual(Ur5TeleopControllerState.Idle, released.State);
    Assert.IsFalse(released.IsMotionCommandActive);
}

[TestCase(1.0f, 0.0f, 0.0f)]
[TestCase(0.0f, 1.0f, 0.0f)]
[TestCase(0.0f, 0.0f, 1.0f)]
public void TranslationSingleAxis_DoesNotCouple(float x, float y, float z)
{
    var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
    controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));

    Vector3 handDelta = new Vector3(x, y, z) * 0.001f;
    Ur5Continuous6DofStepResult result =
        controller.Step(CreateInput(true, handDelta, Quaternion.identity));

    Vector3 tcpDelta = result.TargetPosition - new Vector3(0.40f, 0.50f, 0.60f);
    AssertVectorNear(handDelta * 0.15f, tcpDelta, 0.000001f);
}

[TestCase(1.0f, 0.0f, 0.0f)]
[TestCase(0.0f, 1.0f, 0.0f)]
[TestCase(0.0f, 0.0f, 1.0f)]
public void RotationSingleAxis_MapsFullQuaternion(float x, float y, float z)
{
    var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
    controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
    Vector3 axis = new Vector3(x, y, z);

    Ur5Continuous6DofStepResult result = controller.Step(CreateInput(
        true, Vector3.zero, Quaternion.AngleAxis(1.0f, axis)));

    Assert.AreEqual(1.0f, result.ControllerAngleDegrees, 0.001f);
    Assert.AreEqual(0.25f, result.MappedAngleDegrees, 0.001f);
    Quaternion expected =
        Quaternion.AngleAxis(0.25f, axis) * Quaternion.Euler(5.0f, 10.0f, 15.0f);
    Assert.LessOrEqual(Quaternion.Angle(expected, result.TargetRotation), 0.001f);
}

[Test]
public void RotationAcrossQuaternionSign_UsesShortestPath()
{
    var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
    Quaternion anchor = Quaternion.AngleAxis(179.0f, Vector3.up);
    controller.Step(CreateInput(true, Vector3.zero, anchor));
    Quaternion current = Quaternion.AngleAxis(181.0f, Vector3.up);

    Ur5Continuous6DofStepResult result =
        controller.Step(CreateInput(true, Vector3.zero, current));

    Assert.AreEqual(2.0f, result.ControllerAngleDegrees, 0.01f);
    Assert.Less(result.MappedAngleDegrees, 1.0f);
}

[Test]
public void InvalidInputFault_LatchesUntilGripRelease()
{
    var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
    Ur5Continuous6DofStepInput invalid =
        CreateInput(true, new Vector3(float.NaN, 0.0f, 0.0f), Quaternion.identity);

    Ur5Continuous6DofStepResult fault = controller.Step(invalid);
    Assert.AreEqual(Ur5TeleopControllerState.Fault, fault.State);
    Assert.AreEqual(Ur5Continuous6DofFaultReason.InvalidControllerPose, fault.FaultReason);

    Ur5Continuous6DofStepResult stillFaulted =
        controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
    Assert.AreEqual(Ur5TeleopControllerState.Fault, stillFaulted.State);

    controller.Step(CreateInput(false, Vector3.zero, Quaternion.identity));
    Ur5Continuous6DofStepResult recovered =
        controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
    Assert.AreEqual(Ur5TeleopControllerState.Clutched, recovered.State);
}

[Test]
public void Pause_RequiresGripReleaseBeforeReclutch()
{
    var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
    controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
    controller.Pause();

    Assert.AreEqual(
        Ur5TeleopControllerState.Paused,
        controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity)).State);

    controller.Step(CreateInput(false, Vector3.zero, Quaternion.identity));
    Assert.AreEqual(
        Ur5TeleopControllerState.Clutched,
        controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity)).State);
}

private static Ur5Continuous6DofStepInput CreateInput(
    bool gripHeld,
    Vector3 controllerPosition,
    Quaternion controllerRotation)
{
    return new Ur5Continuous6DofStepInput(
        gripHeld,
        controllerPosition,
        controllerRotation,
        new Vector3(0.40f, 0.50f, 0.60f),
        Quaternion.Euler(5.0f, 10.0f, 15.0f),
        isInputPoseValid: true,
        isRobotStateValid: true,
        isSafetyAccepted: true);
}

private static void AssertVectorNear(Vector3 expected, Vector3 actual, float tolerance)
{
    Assert.LessOrEqual(Vector3.Distance(expected, actual), tolerance);
}
```

- [ ] **Step 2: 运行测试并确认因控制器类型不存在而失败**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
"$TUANJIE_EDITOR" \
  -batchmode -nographics \
  -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofClutchControllerTests \
  -testResults /tmp/ur5-continuous-controller-red-tests.xml \
  -logFile /tmp/ur5-continuous-controller-red-tests.log \
  -quit
```

Expected: FAIL，错误指向 `Ur5Continuous6DofClutchController`、输入或结果类型不存在。

- [ ] **Step 3: 实现输入、结果和 Fault 类型**

新文件定义：

```csharp
public enum Ur5Continuous6DofFaultReason
{
    None,
    InvalidControllerPose,
    InvalidRobotPose,
    SafetyRejected
}

public struct Ur5Continuous6DofStepInput
{
    public bool GripHeld;
    public Vector3 ControllerPosition;
    public Quaternion ControllerRotation;
    public Vector3 ActualTcpPosition;
    public Quaternion ActualTcpRotation;
    public bool IsInputPoseValid;
    public bool IsRobotStateValid;
    public bool IsSafetyAccepted;

    public Ur5Continuous6DofStepInput(
        bool gripHeld,
        Vector3 controllerPosition,
        Quaternion controllerRotation,
        Vector3 actualTcpPosition,
        Quaternion actualTcpRotation,
        bool isInputPoseValid,
        bool isRobotStateValid,
        bool isSafetyAccepted)
    {
        GripHeld = gripHeld;
        ControllerPosition = controllerPosition;
        ControllerRotation = controllerRotation;
        ActualTcpPosition = actualTcpPosition;
        ActualTcpRotation = actualTcpRotation;
        IsInputPoseValid = isInputPoseValid;
        IsRobotStateValid = isRobotStateValid;
        IsSafetyAccepted = isSafetyAccepted;
    }
}

public readonly struct Ur5Continuous6DofStepResult
{
    public readonly Ur5TeleopControllerState State;
    public readonly Ur5Continuous6DofFaultReason FaultReason;
    public readonly bool IsMotionCommandActive;
    public readonly bool WasAnchoredThisStep;
    public readonly float ControllerDistanceMeters;
    public readonly float ControllerAngleDegrees;
    public readonly float TranslationGain;
    public readonly float RotationGain;
    public readonly float MappedAngleDegrees;
    public readonly Vector3 TargetPosition;
    public readonly Quaternion TargetRotation;
}
```

为输入和结果提供完整构造函数，调用位置不得依赖对象初始化器漏填安全字段。

- [ ] **Step 4: 实现状态机与映射**

控制器公开接口固定为：

```csharp
public sealed class Ur5Continuous6DofClutchController
{
    public Ur5TeleopControllerState State { get; private set; }
    public Ur5Continuous6DofFaultReason FaultReason { get; private set; }

    public Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig config)
    {
        Configure(config);
    }

    public void Configure(Ur5Continuous6DofConfig config)
    {
        this.config = config.Sanitized();
    }

    public void Pause()
    {
        hasAnchor = false;
        pauseLatched = true;
        State = Ur5TeleopControllerState.Paused;
        FaultReason = Ur5Continuous6DofFaultReason.None;
    }
}
```

`public Ur5Continuous6DofStepResult Step(Ur5Continuous6DofStepInput input)` 的顺序固定为：

1. Grip 未按下：清除 anchor、Fault 和 Paused 锁存，返回 Idle；
2. Paused 且 Grip 仍按下：保持 Paused；
3. Fault 且 Grip 仍按下：保持 Fault；
4. 输入、安全或实际 TCP 无效：进入 Fault；
5. 首次有效按下：同时捕获手柄和实际 TCP，返回零位移 Clutched；
6. 后续有效帧：从固定基准计算目标。

旋转核心必须保持下列顺序，并添加中文注释说明左乘含义：

```csharp
Quaternion current = Normalize(input.ControllerRotation);
Quaternion anchorInverse = Quaternion.Inverse(controllerAnchorRotation);
Quaternion relative = Normalize(current * anchorInverse);
if (relative.w < 0.0f)
{
    relative = new Quaternion(-relative.x, -relative.y, -relative.z, -relative.w);
}

relative.ToAngleAxis(out float angleDegrees, out Vector3 axis);
if (angleDegrees > 180.0f)
{
    angleDegrees -= 360.0f;
}

float gain = config.EvaluateRotationGain(Mathf.Abs(angleDegrees));
float mappedDegrees = Mathf.Clamp(
    angleDegrees * gain,
    -config.MaxClutchRotationDegrees,
    config.MaxClutchRotationDegrees);
Quaternion mappedDelta = axis.sqrMagnitude > 0.000001f
    ? Quaternion.AngleAxis(mappedDegrees, axis.normalized)
    : Quaternion.identity;
Quaternion targetRotation = Normalize(mappedDelta * tcpAnchorRotation);
```

平移使用手柄位移模长评估增益，再对映射后位移执行单次离合最大距离裁剪。

- [ ] **Step 5: 运行目标测试并确认通过**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
"$TUANJIE_EDITOR" \
  -batchmode -nographics \
  -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofClutchControllerTests \
  -testResults /tmp/ur5-continuous-controller-green-tests.xml \
  -logFile /tmp/ur5-continuous-controller-green-tests.log \
  -quit
```

Expected: 新测试类全部 PASS，至少覆盖 3 个配置测试、离合、3 个平移轴、3 个旋转轴、180°、Fault 和 Pause。

- [ ] **Step 6: 检查差异并提交**

```bash
git diff --check
git add \
  Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofClutchController.cs \
  Assets/Scripts/UR5/ControlLogic/Ur5Continuous6DofClutchController.cs.meta \
  Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs
git commit -m "feat(teleop): add single-mode 6dof clutch mapping"
```

---

### Task 3: 时间常数滤波与硬速度限幅

**Files:**

- Modify: `Assets/Scripts/UR5/Ur5RelativePoseCommandFilter.cs`
- Modify: `Assets/Tests/EditMode/Editor/Ur5RelativePoseClutchMapperTests.cs`

**Interfaces:**

- Produces:

```csharp
bool FilterByTimeConstants(
    Vector3 targetPosition,
    Quaternion targetRotation,
    float positionTimeConstantSeconds,
    float rotationTimeConstantSeconds,
    float deltaTimeSeconds,
    float maxLinearSpeedMetersPerSecond,
    float maxAngularSpeedDegreesPerSecond,
    out Vector3 position,
    out Quaternion rotation)
```

- [ ] **Step 1: 写跨刷新率与限幅的失败测试**

追加以下三个测试：

```csharp
[Test]
public void TimeConstantFilter_IsConsistentAt72_90And120Hz()
{
    (Vector3 position, Quaternion rotation) at72 = SimulateTimeConstantFilter(72.0f);
    (Vector3 position, Quaternion rotation) at90 = SimulateTimeConstantFilter(90.0f);
    (Vector3 position, Quaternion rotation) at120 = SimulateTimeConstantFilter(120.0f);

    Assert.LessOrEqual(Vector3.Distance(at72.position, at90.position), 0.0005f);
    Assert.LessOrEqual(Vector3.Distance(at72.position, at120.position), 0.0005f);
    Assert.LessOrEqual(Quaternion.Angle(at72.rotation, at90.rotation), 0.1f);
    Assert.LessOrEqual(Quaternion.Angle(at72.rotation, at120.rotation), 0.1f);
}

[Test]
public void TimeConstantFilter_EnforcesLinearSpeedLimit()
{
    var filter = new Ur5RelativePoseCommandFilter();
    filter.Reset(Vector3.zero, Quaternion.identity);

    Assert.IsTrue(filter.FilterByTimeConstants(
        Vector3.right, Quaternion.identity,
        0.055f, 0.055f, 0.01f,
        0.26f, 4.0f * Mathf.Rad2Deg,
        out Vector3 position, out _));

    Assert.LessOrEqual(position.magnitude, 0.26f * 0.01f + 0.000001f);
}

[Test]
public void TimeConstantFilter_EnforcesAngularSpeedLimit()
{
    var filter = new Ur5RelativePoseCommandFilter();
    filter.Reset(Vector3.zero, Quaternion.identity);

    Assert.IsTrue(filter.FilterByTimeConstants(
        Vector3.zero, Quaternion.AngleAxis(90.0f, Vector3.up),
        0.055f, 0.055f, 0.01f,
        0.26f, 100.0f,
        out _, out Quaternion rotation));

    Assert.LessOrEqual(Quaternion.Angle(Quaternion.identity, rotation), 1.0001f);
}
```

辅助函数对 `Vector3.one * 0.1f` 和 `Quaternion.Euler(20,30,40)` 持续输入 `0.5 s`，每次调用均使用 `0.055 s` 时间常数，并返回最后结果。

- [ ] **Step 2: 运行过滤器测试并确认新 API 不存在**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
"$TUANJIE_EDITOR" \
  -batchmode -nographics \
  -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5RelativePoseClutchMapperTests \
  -testResults /tmp/ur5-pose-filter-tests.xml \
  -logFile /tmp/ur5-pose-filter-tests.log \
  -quit
```

Expected: FAIL，`FilterByTimeConstants` 未定义。

- [ ] **Step 3: 实现唯一的时间常数滤波 API**

新增方法时保留旧 `Filter(...)`，避免兼容路径编译回归。新方法遵循：

```csharp
float dt = Mathf.Max(0.000001f, deltaTimeSeconds);
float positionAlpha = 1.0f - Mathf.Exp(
    -dt / Mathf.Max(0.001f, positionTimeConstantSeconds));
float rotationAlpha = 1.0f - Mathf.Exp(
    -dt / Mathf.Max(0.001f, rotationTimeConstantSeconds));

Vector3 smoothedPosition =
    Vector3.Lerp(filteredPosition, targetPosition, positionAlpha);
Quaternion smoothedRotation =
    Quaternion.Slerp(filteredRotation, targetRotation, rotationAlpha);

filteredPosition = maxLinearSpeedMetersPerSecond > 0.0f
    ? Vector3.MoveTowards(
        filteredPosition,
        smoothedPosition,
        maxLinearSpeedMetersPerSecond * dt)
    : smoothedPosition;
filteredRotation = maxAngularSpeedDegreesPerSecond > 0.0f
    ? Quaternion.RotateTowards(
        filteredRotation,
        smoothedRotation,
        maxAngularSpeedDegreesPerSecond * dt)
    : smoothedRotation;
```

输入位置、四元数、时间和限幅必须进行有限值检查；非法输入返回 `false` 并保持上一有效输出。

- [ ] **Step 4: 运行过滤器与旧回归测试**

分别运行：

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"

"$TUANJIE_EDITOR" \
  -batchmode -nographics \
  -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5RelativePoseClutchMapperTests \
  -testResults /tmp/ur5-pose-filter-tests.xml \
  -logFile /tmp/ur5-pose-filter-tests.log \
  -quit

"$TUANJIE_EDITOR" \
  -batchmode -nographics \
  -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofClutchControllerTests \
  -testResults /tmp/ur5-continuous-regression-tests.xml \
  -logFile /tmp/ur5-continuous-regression-tests.log \
  -quit
```

Expected: 两个测试类全部 PASS。

- [ ] **Step 5: 检查差异并提交**

```bash
git diff --check
git add \
  Assets/Scripts/UR5/Ur5RelativePoseCommandFilter.cs \
  Assets/Tests/EditMode/Editor/Ur5RelativePoseClutchMapperTests.cs
git commit -m "feat(teleop): add time-invariant pose filtering"
```

---

### Task 4: 接入 Unity 右手单模式运行链路

**Files:**

- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs:28-240`
- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs:287-343`
- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs:356-489`
- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs:491-531`
- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs:899-1087`
- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs:2050-2110`
- Modify: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs`

**Interfaces:**

- Consumes: `Ur5Continuous6DofConfig`
- Consumes: `Ur5Continuous6DofClutchController`
- Consumes: `Ur5RelativePoseCommandFilter.FilterByTimeConstants(...)`
- Produces: `EnableContinuous6DofClutch`
- Produces: `Continuous6DofFaultReason`
- Produces: `ContinuousControllerDistanceMeters`
- Produces: `ContinuousControllerAngleDegrees`
- Produces: `ContinuousTranslationGain`
- Produces: `ContinuousRotationGain`

- [ ] **Step 1: 添加公开状态和输入所有权的失败测试**

在测试中创建临时 `GameObject`，挂载控制器，并断言：

```csharp
[Test]
public void ContinuousController_DefaultsToSingleRightHandOwnership()
{
    GameObject owner = new GameObject("teleop-test");
    try
    {
        var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();

        Assert.IsTrue(teleop.enableContinuous6DofClutch);
        Assert.AreEqual(UnityEngine.XR.XRNode.RightHand, teleop.positionControllerNode);
        Assert.AreEqual(UnityEngine.XR.XRNode.RightHand, teleop.rotationControllerNode);
        Assert.IsTrue(teleop.usePositionGripAsDeadman);
        Assert.IsTrue(teleop.useRotationGripAsDeadman);
    }
    finally
    {
        Object.DestroyImmediate(owner);
    }
}
```

- [ ] **Step 2: 运行测试并确认新开关不存在**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
"$TUANJIE_EDITOR" \
  -batchmode -nographics \
  -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofClutchControllerTests \
  -testResults /tmp/ur5-continuous-integration-red-tests.xml \
  -logFile /tmp/ur5-continuous-integration-red-tests.log \
  -quit
```

Expected: FAIL，控制器缺少 `enableContinuous6DofClutch`。

- [ ] **Step 3: 添加 Inspector 配置、纯逻辑实例与公开诊断属性**

在 Controllers 区域增加：

```csharp
[Header("单模式 6DoF 离合")]
[Tooltip("标准 Quest 控制：右手 Grip 同时控制 TCP 平移与完整三轴旋转。")]
public bool enableContinuous6DofClutch = true;
public Ur5Continuous6DofConfig continuous6DofConfig = Ur5Continuous6DofConfig.Default;
```

增加：

```csharp
private readonly Ur5Continuous6DofClutchController continuous6DofController =
    new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
private Ur5Continuous6DofStepResult latestContinuous6DofStep;
```

公开属性固定为：

```csharp
public bool EnableContinuous6DofClutch => enableContinuous6DofClutch;
public Ur5Continuous6DofFaultReason Continuous6DofFaultReason =>
    latestContinuous6DofStep.FaultReason;
public float ContinuousControllerDistanceMeters =>
    latestContinuous6DofStep.ControllerDistanceMeters;
public float ContinuousControllerAngleDegrees =>
    latestContinuous6DofStep.ControllerAngleDegrees;
public float ContinuousTranslationGain =>
    latestContinuous6DofStep.TranslationGain;
public float ContinuousRotationGain =>
    latestContinuous6DofStep.RotationGain;
```

当新开关启用时，
`TeleopControllerState` 返回新状态，`ActiveTeleopMode` 仅为旧诊断兼容返回 `Ur5TeleopMode.Free`。

- [ ] **Step 4: 将 Update 的标准输入收敛到右手完整位姿**

保留设备刷新、左手安全姿态和 `SynchronizeOrientationAfterReadyPose()`，但必须先计算一次右手 Grip。为
`UpdateLeftControllerSafetyPose` 增加 `bool blockPrimaryPose` 参数；当右手 Grip 已按下时，
左手 X 的 `primaryPressed` 必须被强制为 false，保证首次同帧输入也不会竞争写入。随后调用新的私有方法并提前结束旧输入分支：

```csharp
private void UpdateContinuous6DofInput(
    bool hasPosition,
    Vector3 positionWorld,
    bool hasRotation,
    Quaternion rotationWorld,
    bool gripHeld)
{
    bool poseValid = hasPosition && hasRotation;
    bool safetyPoseOwnsTarget = leftPrimaryWasPressed
        || (tcpFollower != null && tcpFollower.IsReadyPoseActive);

    if (safetyPoseOwnsTarget)
    {
        continuous6DofController.Pause();
        IsPositionClutched = false;
        IsRotationClutched = false;
        IsInputPoseValid = poseValid;
        return;
    }

    IsPositionClutched = gripHeld;
    IsRotationClutched = gripHeld;
    IsInputPoseValid = poseValid;
    IsFineControlActive = false;
    IsFinePositionControlActive = false;
    IsInsertModeActive = false;

    continuous6DofConfig = continuous6DofConfig.Sanitized();
    continuous6DofController.Configure(continuous6DofConfig);
    latestContinuous6DofStep = continuous6DofController.Step(
        new Ur5Continuous6DofStepInput(
            gripHeld,
            positionWorld,
            rotationWorld,
            tcpPreviewTarget != null ? GetActualToolPosition() : Vector3.zero,
            tcpPreviewTarget != null ? GetActualToolRotation() : Quaternion.identity,
            poseValid,
            tcpPreviewTarget != null && IsRobotOutputReady(),
            targetWriteMonitor == null || !targetWriteMonitor.HadWriteConflictThisFrame));

    if (latestContinuous6DofStep.WasAnchoredThisStep)
    {
        relativePoseCommandFilter.Reset(
            latestContinuous6DofStep.TargetPosition,
            latestContinuous6DofStep.TargetRotation);
    }

    rawBaseLinearVelocity = Vector3.zero;
    rawBaseAngularVelocity = Vector3.zero;
    limitedBaseLinearVelocity = Vector3.zero;
    limitedBaseAngularVelocity = Vector3.zero;
    filteredBaseLinearVelocity = Vector3.zero;
    filteredBaseAngularVelocity = Vector3.zero;
}
```

`Update()` 中的调用顺序固定为：

```csharp
bool continuousPoseValid = hasPosition && hasRotation;
bool continuousGripHeld = enableContinuous6DofClutch
    && continuousPoseValid
    && (!usePositionGripAsDeadman
        || ReadGripDeadman(positionDevice, ref positionGripLatched));

UpdateLeftControllerSafetyPose(blockPrimaryPose: continuousGripHeld);
SynchronizeOrientationAfterReadyPose();

if (enableContinuous6DofClutch)
{
    UpdateContinuous6DofInput(
        hasPosition,
        positionWorld,
        hasRotation,
        rotationWorld,
        continuousGripHeld);
    return;
}
```

兼容旧路径调用 `UpdateLeftControllerSafetyPose(blockPrimaryPose: false)`。方法内部计算左手 X 时使用：

```csharp
bool primaryPressed = !blockPrimaryPose
    && enableLeftPrimarySnapDown
    && ReadPrimaryButton(leftDevice);
```

构造函数参数顺序必须与 Task 2 的输入类型完全一致。Grip 松开时不得调用
`relativePoseCommandFilter.FilterByTimeConstants`。

- [ ] **Step 5: 新增唯一的 FixedUpdate 目标写入方法**

在 `ApplyUnityPreview` 最前面分流：

```csharp
if (enableContinuous6DofClutch)
{
    ApplyContinuous6DofPreview(deltaTime);
    return;
}
```

新方法固定执行：

```csharp
private void ApplyContinuous6DofPreview(float deltaTime)
{
    if (tcpPreviewTarget == null
        || latestContinuous6DofStep.State != Ur5TeleopControllerState.Clutched
        || !latestContinuous6DofStep.IsMotionCommandActive)
    {
        return;
    }

    Vector3 requestedPosition = latestContinuous6DofStep.TargetPosition;
    Quaternion requestedRotation = latestContinuous6DofStep.TargetRotation;
    LogicalCommandPosition = requestedPosition;
    LogicalCommandRotation = requestedRotation;

    if (clampPreviewWithWorkspaceLimiter && workspaceLimiter != null)
    {
        requestedPosition = workspaceLimiter.ClampWorldPosition(requestedPosition);
    }

    ConstrainedCommandPosition = requestedPosition;
    if (!relativePoseCommandFilter.FilterByTimeConstants(
        requestedPosition,
        requestedRotation,
        continuous6DofConfig.PositionTimeConstantSeconds,
        continuous6DofConfig.RotationTimeConstantSeconds,
        deltaTime,
        continuous6DofConfig.MaxLinearSpeedMetersPerSecond,
        continuous6DofConfig.MaxAngularSpeedDegreesPerSecond,
        out Vector3 filteredPosition,
        out Quaternion filteredRotation))
    {
        continuous6DofController.Pause();
        return;
    }

    FilteredCommandPosition = filteredPosition;
    FilteredCommandRotation = filteredRotation;
    tcpPreviewTarget.SetPositionAndRotation(filteredPosition, filteredRotation);
    targetWriteMonitor?.RecordWrite("QuestTeleopContinuous6Dof");
}
```

不得在该方法调用 `SnapGraspApproachToVertical`、
`ApplyActiveModePositionConstraints`、`AdvancePreview*Trajectory` 或旧
`anchoredPoseTeleop.FilterRequestedPose`。

- [ ] **Step 6: 处理 Disable、Validate 和左手 X 所有权**

- `OnDisable()` 调用 `continuous6DofController.Pause()`；
- `OnValidate()` 对 `continuous6DofConfig` 执行 `Sanitized()`；
- 左手 X 已取得所有权时新控制器进入 Paused，且不得在同帧调用 `Step(...GripHeld=false)` 把 Paused 提前清成 Idle；
- Grip 与 X 同时按下时 X 不得和遥操作在同一帧写 `TcpTarget`；
- X 释放但右手 Grip 未松开时保持 Paused，直到右手先松开再按下。

- [ ] **Step 7: 运行新控制器测试与现有 9.2/9.3 回归**

依次运行：

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofClutchControllerTests \
  -testResults /tmp/ur5-continuous-controller-tests.xml \
  -logFile /tmp/ur5-continuous-controller-tests.log -quit

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5PoseMathTestBenchTests \
  -testResults /tmp/ur5-pose-math-regression.xml \
  -logFile /tmp/ur5-pose-math-regression.log -quit

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5ClutchModeControllerTests \
  -testResults /tmp/ur5-legacy-clutch-regression.xml \
  -logFile /tmp/ur5-legacy-clutch-regression.log -quit
```

Expected: 三组测试全部 PASS；旧三模式测试保留通过，但不代表 Quest 默认仍使用三模式。

- [ ] **Step 8: 检查差异并提交**

```bash
git diff --check
git add \
  Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs \
  Assets/Tests/EditMode/Editor/Ur5Continuous6DofClutchControllerTests.cs
git commit -m "feat(teleop): integrate right-hand 6dof control"
```

---

### Task 5: Quest 默认配置、诊断和 CSV

**Files:**

- Create: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`
- Modify: `Assets/Scripts/UR5/Ur5ControlBootstrap.cs:428-512`
- Modify: `Assets/Scripts/UR5/Ur5TeleopDiagnostics.cs:12-110`
- Modify: `Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs:20-160`

**Interfaces:**

- Consumes: `Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(...)`
- Consumes: Task 4 的公开诊断属性
- Produces: Quest 默认单模式配置
- Produces: 可区分输入、映射、过滤和安全拒绝的日志字段

- [ ] **Step 1: 写 Quest 默认配置的失败测试**

新测试文件内容至少包含：

```csharp
using NUnit.Framework;
using UnityEngine;
using UnityEngine.XR;

public class Ur5Continuous6DofBootstrapTests
{
    [Test]
    public void DefaultQuestProfile_UsesOnlyRightHandContinuous6Dof()
    {
        GameObject owner = new GameObject("bootstrap-profile-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);

            Assert.IsTrue(teleop.enableContinuous6DofClutch);
            Assert.AreEqual(XRNode.RightHand, teleop.positionControllerNode);
            Assert.AreEqual(XRNode.RightHand, teleop.rotationControllerNode);
            Assert.IsFalse(teleop.enableThreeModeController);
            Assert.IsFalse(teleop.enableAPrecisionModifier);
            Assert.IsFalse(teleop.enableFineControlButton);
            Assert.IsFalse(teleop.enableLeftSecondaryPoseRotation);
            Assert.IsFalse(teleop.enableLeftSecondaryOrientationHold);
            Assert.IsFalse(teleop.snapGraspApproachToVertical);
            Assert.IsTrue(teleop.enableLeftPrimarySnapDown);
            Assert.IsTrue(teleop.enableLeftPrimaryReadyPose);
        }
        finally
        {
            Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void DefaultQuestProfile_UsesApprovedContinuousParameters()
    {
        GameObject owner = new GameObject("bootstrap-parameter-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);
            Ur5Continuous6DofConfig config = teleop.continuous6DofConfig;

            Assert.AreEqual(0.15f, config.TranslationNearGain, 0.000001f);
            Assert.AreEqual(0.80f, config.TranslationFarGain, 0.000001f);
            Assert.AreEqual(0.25f, config.RotationNearGain, 0.000001f);
            Assert.AreEqual(0.80f, config.RotationFarGain, 0.000001f);
            Assert.AreEqual(0.055f, config.PositionTimeConstantSeconds, 0.000001f);
            Assert.AreEqual(0.055f, config.RotationTimeConstantSeconds, 0.000001f);
        }
        finally
        {
            Object.DestroyImmediate(owner);
        }
    }
}
```

- [ ] **Step 2: 运行测试并确认旧 Quest Profile 不符合断言**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofBootstrapTests \
  -testResults /tmp/ur5-continuous-bootstrap-tests.xml \
  -logFile /tmp/ur5-continuous-bootstrap-tests.log -quit
```

Expected: FAIL，至少显示旋转节点仍为 LeftHand、三模式仍启用或自动吸附仍启用。

- [ ] **Step 3: 修改 Quest 默认 Profile**

`ApplyDefaultQuestTeleopProfile` 必须显式设置：

```csharp
velocityTeleop.enableContinuous6DofClutch = true;
velocityTeleop.continuous6DofConfig = Ur5Continuous6DofConfig.Default;
velocityTeleop.positionControllerNode = XRNode.RightHand;
velocityTeleop.rotationControllerNode = XRNode.RightHand;
velocityTeleop.usePositionGripAsDeadman = true;
velocityTeleop.useRotationGripAsDeadman = true;

velocityTeleop.enableThreeModeController = false;
velocityTeleop.useRightSecondaryButtonForInsertMode = false;
velocityTeleop.enableAPrecisionModifier = false;
velocityTeleop.enableFineControlButton = false;
velocityTeleop.enableLeftSecondaryPoseRotation = false;
velocityTeleop.enableLeftSecondaryOrientationHold = false;
velocityTeleop.snapGraspApproachToVertical = false;

velocityTeleop.filterControllerPosition = false;
velocityTeleop.useAdaptiveControllerPositionFilter = false;
velocityTeleop.previewTargetDeadbandMeters = 0.0f;
velocityTeleop.finePreviewTargetDeadbandMeters = 0.0f;
velocityTeleop.useAccelerationLimitedPreviewTrajectory = false;
velocityTeleop.limitPreviewLeadToActualTcp = false;

velocityTeleop.enableLeftPrimarySnapDown = true;
velocityTeleop.enableLeftPrimaryReadyPose = true;
```

旧 `rotationInputMode` 可设置为 `ControllerPoseDelta` 作为 Inspector 兼容显示，但新标准链路不得读取它决定行为。

- [ ] **Step 4: 扩充诊断属性和日志**

诊断日志新增：

```text
control=continuous6dof
fault=<reason>
handDistanceM=<value>
handAngleDeg=<value>
translationGain=<value>
rotationGain=<value>
rotationFilterGapDeg=<value>
```

`rotationFilterGapDeg` 使用
`Quaternion.Angle(teleop.LogicalCommandRotation, teleop.FilteredCommandRotation)`。
保留原 `mode` 字段，但在单模式下输出 `legacy-free`，避免后续 AI 把它误判为仍有模式切换。

- [ ] **Step 5: 追加 CSV 列**

在原表头末尾追加，不插入旧列中间：

```text
continuous_6dof_enabled,
continuous_state,
continuous_fault,
controller_distance_m,
controller_angle_deg,
translation_gain,
rotation_gain,
logical_to_filtered_rotation_deg
```

每个采样点按相同顺序追加值。旧列继续输出，保证已有分析脚本仍能读取其前缀 schema。

- [ ] **Step 6: 运行 Bootstrap、控制器和过滤器测试**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofBootstrapTests \
  -testResults /tmp/ur5-continuous-bootstrap-tests.xml \
  -logFile /tmp/ur5-continuous-bootstrap-tests.log -quit

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5Continuous6DofClutchControllerTests \
  -testResults /tmp/ur5-continuous-controller-tests.xml \
  -logFile /tmp/ur5-continuous-controller-tests.log -quit

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5RelativePoseClutchMapperTests \
  -testResults /tmp/ur5-pose-filter-tests.xml \
  -logFile /tmp/ur5-pose-filter-tests.log -quit
```

Expected: 三个测试类全部 PASS。

- [ ] **Step 7: 检查差异并提交**

```bash
git diff --check
git add \
  Assets/Scripts/UR5/Ur5ControlBootstrap.cs \
  Assets/Scripts/UR5/Ur5TeleopDiagnostics.cs \
  Assets/Scripts/UR5/Ur5PoseCsvRecorder.cs \
  Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs \
  Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs.meta
git commit -m "fix(teleop): enable single-mode quest control"
```

---

### Task 6: 本地全量验证与交接记录

**Files:**

- Create: `docs/teleop_single_mode_6dof_validation.md`
- Verify only: all files modified in Tasks 1–5

**Interfaces:**

- Produces: 可供 Quest 验证和后续 AI 接手的确定性检查表。

- [ ] **Step 1: 创建验证记录**

文档固定包含：

```markdown
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
记录测试 XML 路径、通过数、失败数、提交 SHA 和日期。

## Quest 验证结果
记录 Build SHA、刷新率、六轴结果、松手结果、Fault 结果、主观手感和日志路径。

## 禁止事项
- 不连接真实 UR5。
- 不新增模式。
- 不在无日志证据时调参。
```

- [ ] **Step 2: 运行相关 EditMode 全量测试**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"

"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -runTests -testPlatform EditMode \
  -testFilter "Ur5Continuous6DofClutchControllerTests;Ur5Continuous6DofBootstrapTests;Ur5RelativePoseClutchMapperTests;Ur5PoseMathTestBenchTests;Ur5ClutchModeControllerTests" \
  -testResults /tmp/ur5-single-mode-all-tests.xml \
  -logFile /tmp/ur5-single-mode-all-tests.log -quit
```

若 Tuanjie 的分号 filter 不被当前版本接受，执行以下确定性逐类命令：

```bash
for test_filter in \
  Ur5Continuous6DofClutchControllerTests \
  Ur5Continuous6DofBootstrapTests \
  Ur5RelativePoseClutchMapperTests \
  Ur5PoseMathTestBenchTests \
  Ur5ClutchModeControllerTests
do
  "$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$PWD" \
    -runTests -testPlatform EditMode \
    -testFilter "$test_filter" \
    -testResults "/tmp/${test_filter}.xml" \
    -logFile "/tmp/${test_filter}.log" -quit
done
```

Expected: 目标五个测试类 0 FAIL；日志中无 C# compilation error。

- [ ] **Step 3: 检查 Android 编译前静态条件**

```bash
rg -n "enableContinuous6DofClutch|rotationControllerNode|enableThreeModeController|snapGraspApproachToVertical" \
  Assets/Scripts/UR5/Ur5ControlBootstrap.cs
rg -n "enableRealRobotOutput" Assets/Scripts/UR5
git diff --check
git status --short
```

Expected:

- Profile 明确启用 continuous 6DoF；
- position/rotation 节点均为 RightHand；
- 三模式和自动吸附关闭；
- 没有任何改动把 `enableRealRobotOutput` 默认值或运行配置改为 true；
- `git status` 只包含任务文件，以及开始执行前已经存在且明确不提交的用户改动。

- [ ] **Step 4: 审查单级滤波和单写入源**

```bash
rg -n "ApplyContinuous6DofPreview|FilterByTimeConstants|QuestTeleopContinuous6Dof|SnapGraspApproachToVertical|AdvancePreview" \
  Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs
```

人工确认新方法只调用一次 `FilterByTimeConstants`，不调用自动吸附和旧轨迹，并且只记录一次 `QuestTeleopContinuous6Dof` 写入。

- [ ] **Step 5: 更新验证记录并提交**

将测试日期、XML、通过数量和当前提交 SHA 写入验证文档。Quest 部分明确标记为“等待用户确认 Build & Run”，而不是写成已通过。

```bash
git add \
  docs/teleop_single_mode_6dof_validation.md
git commit -m "docs(teleop): add single-mode validation checklist"
```

---

### Task 7: Quest 3 Build & Run 人工验证门

**Files:**

- Modify after evidence: `docs/teleop_single_mode_6dof_validation.md`
- Do not modify without evidence: continuous response parameters

**Interfaces:**

- Consumes: Tasks 1–6 全部通过的提交 SHA。
- Produces: Quest 虚拟 UR5 验证证据和是否接受当前参数的结论。

- [ ] **Step 1: 先向用户汇报**

汇报必须包含：

- 本地测试通过数量和测试 XML；
- 当前提交 SHA；
- `enableRealRobotOutput=false` 的检查结果；
- 当前已到 Quest 3 Build & Run 节点；
- 请求用户确认是否现在执行 Build & Run。

未得到确认时停止，不打开 Build & Run。

- [ ] **Step 2: 确认 Build 场景与真实输出开关**

用户确认后，在 Tuanjie GUI 中检查：

- Build Settings 目标为 Android；
- Scenes In Build 包含 `Assets/Scenes/SampleScene.scene`；
- `Ur5UrScriptSpeedlClient.enableRealRobotOutput` 为 false；
- Quest 3 通过 USB/ADB 可见；
- 不启动 Python RTDE、不连接真实 UR5。

- [ ] **Step 3: Build & Run 到 Quest 3**

使用 Tuanjie GUI 的 Build And Run。该动作由用户可见地执行；若 Tuanjie 请求 Android SDK、签名或 Meta XR 必需修复，记录原始错误并停止，不猜测修复。

- [ ] **Step 4: 执行固定手势验证**

按顺序验证：

1. 未按 Grip：移动/旋转右手，TCP 不动；
2. 按下 Grip 且手保持不动：TCP 不跳；
3. 右手沿 X、Y、Z 分别移动：TCP 只产生对应平移；
4. 右手绕 X、Y、Z 分别旋转：TCP 均产生对应旋转；
5. 组合旋转：无突然翻转、无自动吸附；
6. 进行约 1 mm 小平移和约 1° 小旋转：目标有细小、连续响应；
7. 做较大动作：响应连续加快，无档位切换感；
8. 松开 Grip：同帧后不再追赶；
9. Grip 按下时按左手 X：不得形成两个写入源；
10. 松开 Grip 后使用左手 X：执行显式安全姿态复位；
11. 遮挡右手柄或使跟踪失效：进入 Fault，保持 Grip 不恢复，松开再按后恢复；
12. A/B、左摇杆和左手 Y：不得改变 TCP 运动。

- [ ] **Step 5: 保存证据**

保存：

- `ur5_pose_log.csv`；
- Quest/Tuanjie 日志；
- Build SHA；
- Quest 刷新率；
- 六轴通过/失败表；
- 用户对平移精度、旋转精度、延迟、抖动的主观结论。

- [ ] **Step 6: 只按证据调整唯一参数组**

如果用户确认仍不满足要求，先用日志分类：

- 原始手柄无旋转变化：XR 输入问题；
- 原始旋转有效但逻辑目标不变：四元数映射问题；
- 逻辑目标有效但过滤目标滞后：时间常数或速度限幅问题；
- 过滤目标有效但实际模型不跟随：IK/写入所有权问题；
- 静止时逻辑目标抖动：Quest 跟踪噪声或近端增益问题。

只允许调整 `Ur5Continuous6DofConfig` 已有字段。每次只改变一类参数，重新运行相关 EditMode 测试并再次 Build & Run；不得新增 free/fine/insert 或其他控制入口。

- [ ] **Step 7: 记录 Quest 结论并提交**

Quest 通过后更新验证文档，写明参数版本、Build SHA、日志位置、通过项和残余限制：

```bash
git add docs/teleop_single_mode_6dof_validation.md
git commit -m "test(teleop): record quest single-mode validation"
```

---

## Final Review Checklist

- [ ] 标准 Quest 链路只有右手 Grip + 右手完整 6DoF。
- [ ] `Idle/Clutched/Paused/Fault` 均有测试，Fault 和 Pause 要求 Grip 释放。
- [ ] 平移与旋转增益都是连续曲线，没有 A/B 模式切换。
- [ ] 四元数使用 `current * inverse(anchor)`，映射后左乘 TCP 基准。
- [ ] 位置与旋转只执行一次、按秒定义的时间滤波。
- [ ] 松开 Grip 后不继续调用滤波器或写 `TcpTarget`。
- [ ] 左手 X 与右手 Grip 的写入所有权互斥。
- [ ] 自动竖直吸附、左摇杆、左手 Y 和三模式在 Quest Profile 关闭。
- [ ] 右手 Trigger 夹爪路径未改变。
- [ ] 工作空间、最大离合增量、最大线速度和最大角速度仍在必经路径。
- [ ] 诊断能区分输入、映射、滤波、安全拒绝和 IK 跟随问题。
- [ ] 72/90/120 Hz 一致性测试通过。
- [ ] 9.2 数学和旧 9.3 状态机回归测试通过。
- [ ] `enableRealRobotOutput` 保持 false。
- [ ] 未提交 `ProjectSettings.asset`、两个 Windows `.meta` 删除或其他用户改动。
- [ ] Quest Build & Run 前已经向用户汇报并获得确认。

## 后续 9.4 边界

本计划不实现 Unity—Python 通信。9.4 再参考
[elpis-lab/UR10_Teleop](https://github.com/elpis-lab/UR10_Teleop)：

- Quest/Unity 只发送原始手柄位姿、按钮、序号、时间戳、有效标志和心跳；
- Python 负责相对映射、连续响应、过滤和 `RobotCommand`；
- 独立 Quest 仍需至少安装一次稳定发送端；
- 后续只调整 Python 映射参数时不需要反复 Build & Run；
- 不直接复制其 `127.0.0.1`、100 Hz 或固定 smoothing step，必须适配本项目的局域网、刷新率和安全协议。

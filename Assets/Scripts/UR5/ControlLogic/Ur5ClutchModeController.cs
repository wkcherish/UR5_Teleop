using System;
using UnityEngine;

public enum Ur5TeleopMode
{
    Free,
    Fine,
    Insert
}

public enum Ur5TeleopControllerState
{
    Idle,
    Clutched,
    Paused,
    Fault
}

/// <summary>
/// Versioned per-mode tuning used by the clutch state machine. Units are
/// explicit: translation is meters, rotation is degrees, smoothing values are
/// normalized filter coefficients.
/// </summary>
[Serializable]
public struct Ur5TeleopModeConfig
{
    [Tooltip("配置版本号；用于后续迁移三模式参数。")]
    public int Version;
    [Tooltip("平移增益：手部位移映射到 TCP 目标位移或速度增益的倍率。")]
    public float TranslationGain;
    [Tooltip("旋转增益：手柄相对旋转映射到 TCP 旋转的倍率。")]
    public float RotationGain;
    [Tooltip("相对位姿目标单步平滑系数，范围 0-1。")]
    public float PoseSmoothingStep;
    [Tooltip("最终命令低通保留比例，范围 0-0.95。")]
    public float CommandFilterRetention;
    [Tooltip("手柄位移死区，单位米。")]
    public float DeadbandMeters;
    [Tooltip("单次 clutch 内允许的最大 TCP 位移，单位米。")]
    public float MaxLinearDeltaMeters;
    [Tooltip("单次 clutch 内允许的最大 TCP 旋转，单位度。")]
    public float MaxAngularDeltaDegrees;
    [Tooltip("insert 模式插入轴，按 robot base frame 表达。")]
    public Vector3 InsertAxis;
    [Tooltip("是否将平移投影到 insert 插入轴。")]
    public bool ConstrainToInsertAxis;

    public Ur5TeleopModeConfig(
        int version,
        float translationGain,
        float rotationGain,
        float poseSmoothingStep,
        float commandFilterRetention,
        float deadbandMeters,
        float maxLinearDeltaMeters,
        float maxAngularDeltaDegrees,
        Vector3 insertAxis,
        bool constrainToInsertAxis)
    {
        Version = version;
        TranslationGain = translationGain;
        RotationGain = rotationGain;
        PoseSmoothingStep = poseSmoothingStep;
        CommandFilterRetention = commandFilterRetention;
        DeadbandMeters = deadbandMeters;
        MaxLinearDeltaMeters = maxLinearDeltaMeters;
        MaxAngularDeltaDegrees = maxAngularDeltaDegrees;
        InsertAxis = insertAxis;
        ConstrainToInsertAxis = constrainToInsertAxis;
    }

    public Ur5TeleopModeConfig Sanitized()
    {
        return new Ur5TeleopModeConfig(
            Mathf.Max(1, Version),
            Mathf.Max(0.0f, TranslationGain),
            Mathf.Max(0.0f, RotationGain),
            Mathf.Clamp01(PoseSmoothingStep),
            Mathf.Clamp(CommandFilterRetention, 0.0f, 0.95f),
            Mathf.Max(0.0f, DeadbandMeters),
            Mathf.Max(0.0f, MaxLinearDeltaMeters),
            Mathf.Max(0.0f, MaxAngularDeltaDegrees),
            IsFinite(InsertAxis) && InsertAxis.sqrMagnitude > 0.000001f ? InsertAxis.normalized : Vector3.forward,
            ConstrainToInsertAxis);
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}

public struct Ur5TeleopStepInput
{
    public bool GripHeld;
    public Ur5TeleopMode RequestedMode;
    public Vector3 ControllerPosition;
    public Quaternion ControllerRotation;
    public Vector3 ActualTcpPosition;
    public Quaternion ActualTcpRotation;
    public bool IsInputPoseValid;
    public bool IsRobotStateValid;
    public bool IsSafetyAccepted;

    public Ur5TeleopStepInput(
        bool gripHeld,
        Ur5TeleopMode requestedMode,
        Vector3 controllerPosition,
        Quaternion controllerRotation,
        Vector3 actualTcpPosition,
        Quaternion actualTcpRotation,
        bool isInputPoseValid,
        bool isRobotStateValid,
        bool isSafetyAccepted)
    {
        GripHeld = gripHeld;
        RequestedMode = requestedMode;
        ControllerPosition = controllerPosition;
        ControllerRotation = controllerRotation;
        ActualTcpPosition = actualTcpPosition;
        ActualTcpRotation = actualTcpRotation;
        IsInputPoseValid = isInputPoseValid;
        IsRobotStateValid = isRobotStateValid;
        IsSafetyAccepted = isSafetyAccepted;
    }
}

public readonly struct Ur5TeleopStepResult
{
    public readonly Ur5TeleopControllerState State;
    public readonly Ur5TeleopMode ActiveMode;
    public readonly Ur5TeleopModeConfig Config;
    public readonly bool IsMotionCommandActive;
    public readonly bool WasRebasedThisStep;
    public readonly Vector3 LinearDelta;
    public readonly float AngularDeltaDegrees;
    public readonly Vector3 TargetPosition;
    public readonly Quaternion TargetRotation;

    public Ur5TeleopStepResult(
        Ur5TeleopControllerState state,
        Ur5TeleopMode activeMode,
        Ur5TeleopModeConfig config,
        bool isMotionCommandActive,
        bool wasRebasedThisStep,
        Vector3 linearDelta,
        float angularDeltaDegrees,
        Vector3 targetPosition,
        Quaternion targetRotation)
    {
        State = state;
        ActiveMode = activeMode;
        Config = config;
        IsMotionCommandActive = isMotionCommandActive;
        WasRebasedThisStep = wasRebasedThisStep;
        LinearDelta = linearDelta;
        AngularDeltaDegrees = angularDeltaDegrees;
        TargetPosition = targetPosition;
        TargetRotation = targetRotation;
    }
}

/// <summary>
/// Pure clutch and mode state machine for UR5 teleoperation. Unity-facing code
/// reads XR/network/safety state, while this class owns the mode transition
/// rules and the no-jump rebasing contract.
/// </summary>
public sealed class Ur5ClutchModeController
{
    private Ur5TeleopModeConfig freeConfig;
    private Ur5TeleopModeConfig fineConfig;
    private Ur5TeleopModeConfig insertConfig;
    private Vector3 controllerAnchorPosition;
    private Quaternion controllerAnchorRotation = Quaternion.identity;
    private Vector3 tcpAnchorPosition;
    private Quaternion tcpAnchorRotation = Quaternion.identity;
    private Vector3 lastTargetPosition;
    private Quaternion lastTargetRotation = Quaternion.identity;
    private bool hasAnchor;
    private bool faultLatched;

    public Ur5TeleopControllerState State { get; private set; }
    public Ur5TeleopMode ActiveMode { get; private set; }

    public Ur5ClutchModeController()
        : this(DefaultFreeConfig, DefaultFineConfig, DefaultInsertConfig)
    {
    }

    public Ur5ClutchModeController(
        Ur5TeleopModeConfig freeConfig,
        Ur5TeleopModeConfig fineConfig,
        Ur5TeleopModeConfig insertConfig)
    {
        Configure(freeConfig, fineConfig, insertConfig);
    }

    public static Ur5TeleopModeConfig DefaultFreeConfig => new Ur5TeleopModeConfig(
        version: 1,
        translationGain: 0.80f,
        rotationGain: 0.80f,
        poseSmoothingStep: 0.18f,
        commandFilterRetention: 0.55f,
        deadbandMeters: 0.0006f,
        maxLinearDeltaMeters: 0.35f,
        maxAngularDeltaDegrees: 45.0f,
        insertAxis: Vector3.forward,
        constrainToInsertAxis: false);

    public static Ur5TeleopModeConfig DefaultFineConfig => new Ur5TeleopModeConfig(
        version: 1,
        translationGain: 0.15f,
        rotationGain: 0.25f,
        poseSmoothingStep: 0.18f,
        commandFilterRetention: 0.40f,
        deadbandMeters: 0.0002f,
        maxLinearDeltaMeters: 0.10f,
        maxAngularDeltaDegrees: 12.0f,
        insertAxis: Vector3.forward,
        constrainToInsertAxis: false);

    public static Ur5TeleopModeConfig DefaultInsertConfig => new Ur5TeleopModeConfig(
        version: 1,
        translationGain: 0.30f,
        rotationGain: 0.0f,
        poseSmoothingStep: 0.12f,
        commandFilterRetention: 0.65f,
        deadbandMeters: 0.0002f,
        maxLinearDeltaMeters: 0.06f,
        maxAngularDeltaDegrees: 0.0f,
        insertAxis: Vector3.forward,
        constrainToInsertAxis: true);

    public void Configure(
        Ur5TeleopModeConfig freeConfig,
        Ur5TeleopModeConfig fineConfig,
        Ur5TeleopModeConfig insertConfig)
    {
        this.freeConfig = freeConfig.Sanitized();
        this.fineConfig = fineConfig.Sanitized();
        this.insertConfig = insertConfig.Sanitized();
    }

    public void Pause()
    {
        hasAnchor = false;
        faultLatched = false;
        State = Ur5TeleopControllerState.Paused;
    }

    public Ur5TeleopStepResult Step(Ur5TeleopStepInput input)
    {
        Ur5TeleopModeConfig config = GetConfig(input.RequestedMode);
        if (!input.GripHeld)
        {
            hasAnchor = false;
            faultLatched = false;
            ActiveMode = input.RequestedMode;
            State = Ur5TeleopControllerState.Idle;
            return CreateStoppedResult(State, ActiveMode, config, input);
        }

        if (faultLatched || !IsStepInputSafe(input))
        {
            // 任何输入、网络或安全异常都锁存为停止状态，直到操作者松开 Grip。
            hasAnchor = false;
            faultLatched = true;
            ActiveMode = input.RequestedMode;
            State = Ur5TeleopControllerState.Fault;
            return CreateStoppedResult(State, ActiveMode, config, input);
        }

        bool wasRebased = false;
        if (!hasAnchor)
        {
            CaptureAnchor(
                input.ControllerPosition,
                input.ControllerRotation,
                input.ActualTcpPosition,
                input.ActualTcpRotation);
            ActiveMode = input.RequestedMode;
        }
        else if (input.RequestedMode != ActiveMode)
        {
            // 模式切换以当前命令为新 TCP 基准，保证采样点本身不会发生跳变。
            CaptureAnchor(
                input.ControllerPosition,
                input.ControllerRotation,
                lastTargetPosition,
                lastTargetRotation);
            ActiveMode = input.RequestedMode;
            config = GetConfig(ActiveMode);
            wasRebased = true;
        }

        State = Ur5TeleopControllerState.Clutched;
        Vector3 linearDelta = MapLinearDelta(input.ControllerPosition, config);
        Quaternion rotationDelta = MapRotationDelta(input.ControllerRotation, config, out float angularDeltaDegrees);
        lastTargetPosition = tcpAnchorPosition + linearDelta;
        lastTargetRotation = rotationDelta * tcpAnchorRotation;

        return new Ur5TeleopStepResult(
            State,
            ActiveMode,
            config,
            isMotionCommandActive: true,
            wasRebasedThisStep: wasRebased,
            linearDelta,
            angularDeltaDegrees,
            lastTargetPosition,
            lastTargetRotation);
    }

    private void CaptureAnchor(
        Vector3 controllerPosition,
        Quaternion controllerRotation,
        Vector3 tcpPosition,
        Quaternion tcpRotation)
    {
        controllerAnchorPosition = controllerPosition;
        controllerAnchorRotation = Normalize(controllerRotation);
        tcpAnchorPosition = tcpPosition;
        tcpAnchorRotation = Normalize(tcpRotation);
        lastTargetPosition = tcpPosition;
        lastTargetRotation = tcpAnchorRotation;
        hasAnchor = true;
    }

    private Vector3 MapLinearDelta(Vector3 controllerPosition, Ur5TeleopModeConfig config)
    {
        Vector3 delta = ApplyDeadband(controllerPosition - controllerAnchorPosition, config.DeadbandMeters);
        delta *= config.TranslationGain;
        if (config.ConstrainToInsertAxis)
        {
            delta = Vector3.Project(delta, config.InsertAxis);
        }

        return config.MaxLinearDeltaMeters > 0.0f
            ? Vector3.ClampMagnitude(delta, config.MaxLinearDeltaMeters)
            : Vector3.zero;
    }

    private Quaternion MapRotationDelta(
        Quaternion controllerRotation,
        Ur5TeleopModeConfig config,
        out float angularDeltaDegrees)
    {
        angularDeltaDegrees = 0.0f;
        if (config.RotationGain <= 0.0f || config.MaxAngularDeltaDegrees <= 0.0f)
        {
            return Quaternion.identity;
        }

        Quaternion rotationDelta = Normalize(controllerRotation) * Quaternion.Inverse(controllerAnchorRotation);
        rotationDelta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (angleDegrees > 180.0f)
        {
            angleDegrees -= 360.0f;
        }

        if (axis.sqrMagnitude < 0.000001f || Mathf.Abs(angleDegrees) <= 0.0001f)
        {
            return Quaternion.identity;
        }

        float scaledDegrees = Mathf.Clamp(
            angleDegrees * config.RotationGain,
            -config.MaxAngularDeltaDegrees,
            config.MaxAngularDeltaDegrees);
        angularDeltaDegrees = Mathf.Abs(scaledDegrees);
        return Quaternion.AngleAxis(scaledDegrees, axis.normalized);
    }

    private Ur5TeleopModeConfig GetConfig(Ur5TeleopMode mode)
    {
        switch (mode)
        {
            case Ur5TeleopMode.Fine:
                return fineConfig;
            case Ur5TeleopMode.Insert:
                return insertConfig;
            default:
                return freeConfig;
        }
    }

    private Ur5TeleopStepResult CreateStoppedResult(
        Ur5TeleopControllerState state,
        Ur5TeleopMode mode,
        Ur5TeleopModeConfig config,
        Ur5TeleopStepInput input)
    {
        Vector3 targetPosition = IsFinite(input.ActualTcpPosition)
            ? input.ActualTcpPosition
            : lastTargetPosition;
        Quaternion targetRotation = IsValidRotation(input.ActualTcpRotation)
            ? Normalize(input.ActualTcpRotation)
            : lastTargetRotation;
        return new Ur5TeleopStepResult(
            state,
            mode,
            config,
            isMotionCommandActive: false,
            wasRebasedThisStep: false,
            Vector3.zero,
            0.0f,
            targetPosition,
            targetRotation);
    }

    private static Vector3 ApplyDeadband(Vector3 value, float deadband)
    {
        float magnitude = value.magnitude;
        if (magnitude <= deadband || magnitude < 0.000001f)
        {
            return Vector3.zero;
        }

        return value.normalized * (magnitude - Mathf.Max(0.0f, deadband));
    }

    private static bool IsStepInputSafe(Ur5TeleopStepInput input)
    {
        return input.IsInputPoseValid
            && input.IsRobotStateValid
            && input.IsSafetyAccepted
            && IsFinite(input.ControllerPosition)
            && IsValidRotation(input.ControllerRotation)
            && IsFinite(input.ActualTcpPosition)
            && IsValidRotation(input.ActualTcpRotation);
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static bool IsValidRotation(Quaternion value)
    {
        return IsFinite(value.x)
            && IsFinite(value.y)
            && IsFinite(value.z)
            && IsFinite(value.w)
            && SquaredMagnitude(value) > 0.000001f;
    }

    private static Quaternion Normalize(Quaternion value)
    {
        if (!IsValidRotation(value))
        {
            return Quaternion.identity;
        }

        float inverseMagnitude = 1.0f / Mathf.Sqrt(SquaredMagnitude(value));
        return new Quaternion(
            value.x * inverseMagnitude,
            value.y * inverseMagnitude,
            value.z * inverseMagnitude,
            value.w * inverseMagnitude);
    }

    private static float SquaredMagnitude(Quaternion value)
    {
        return value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w;
    }
}

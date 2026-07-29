using UnityEngine;

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

    public Ur5Continuous6DofStepResult(
        Ur5TeleopControllerState state,
        Ur5Continuous6DofFaultReason faultReason,
        bool isMotionCommandActive,
        bool wasAnchoredThisStep,
        float controllerDistanceMeters,
        float controllerAngleDegrees,
        float translationGain,
        float rotationGain,
        float mappedAngleDegrees,
        Vector3 targetPosition,
        Quaternion targetRotation)
    {
        State = state;
        FaultReason = faultReason;
        IsMotionCommandActive = isMotionCommandActive;
        WasAnchoredThisStep = wasAnchoredThisStep;
        ControllerDistanceMeters = controllerDistanceMeters;
        ControllerAngleDegrees = controllerAngleDegrees;
        TranslationGain = translationGain;
        RotationGain = rotationGain;
        MappedAngleDegrees = mappedAngleDegrees;
        TargetPosition = targetPosition;
        TargetRotation = targetRotation;
    }
}

/// <summary>
/// 纯逻辑单模式 6DoF 离合控制器。Unity 层负责 XR/场景输入，本类只维护
/// Grip 基准、Fault/Pause 锁存和从固定基准计算出的 TCP 目标。
/// </summary>
public sealed class Ur5Continuous6DofClutchController
{
    private Ur5Continuous6DofConfig config;
    private Vector3 controllerAnchorPosition;
    private Quaternion controllerAnchorRotation = Quaternion.identity;
    private Vector3 tcpAnchorPosition;
    private Quaternion tcpAnchorRotation = Quaternion.identity;
    private Vector3 lastTargetPosition;
    private Quaternion lastTargetRotation = Quaternion.identity;
    private bool hasAnchor;
    private bool faultLatched;
    private bool pauseLatched;

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

    public Ur5Continuous6DofStepResult Step(Ur5Continuous6DofStepInput input)
    {
        if (!input.GripHeld)
        {
            hasAnchor = false;
            faultLatched = false;
            pauseLatched = false;
            State = Ur5TeleopControllerState.Idle;
            FaultReason = Ur5Continuous6DofFaultReason.None;
            return CreateInactiveResult(Ur5TeleopControllerState.Idle, Ur5Continuous6DofFaultReason.None);
        }

        if (pauseLatched)
        {
            State = Ur5TeleopControllerState.Paused;
            FaultReason = Ur5Continuous6DofFaultReason.None;
            return CreateInactiveResult(Ur5TeleopControllerState.Paused, Ur5Continuous6DofFaultReason.None);
        }

        if (faultLatched)
        {
            State = Ur5TeleopControllerState.Fault;
            return CreateInactiveResult(Ur5TeleopControllerState.Fault, FaultReason);
        }

        Ur5Continuous6DofFaultReason inputFault = ValidateInput(input);
        if (inputFault != Ur5Continuous6DofFaultReason.None)
        {
            return EnterFault(inputFault);
        }

        if (!hasAnchor)
        {
            CaptureAnchor(input);
            return CreateActiveResult(
                wasAnchoredThisStep: true,
                controllerDistanceMeters: 0.0f,
                controllerAngleDegrees: 0.0f,
                translationGain: config.EvaluateTranslationGain(0.0f),
                rotationGain: config.EvaluateRotationGain(0.0f),
                mappedAngleDegrees: 0.0f,
                targetPosition: lastTargetPosition,
                targetRotation: lastTargetRotation);
        }

        Vector3 controllerDelta = input.ControllerPosition - controllerAnchorPosition;
        float controllerDistance = controllerDelta.magnitude;
        float translationGain = EvaluateControllerTranslationGain(controllerDistance);
        Vector3 mappedDelta = Vector3.ClampMagnitude(
            controllerDelta * translationGain,
            config.MaxClutchTranslationMeters);
        Vector3 targetPosition = tcpAnchorPosition + mappedDelta;

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

        float rotationGain = EvaluateControllerRotationGain(Mathf.Abs(angleDegrees));
        float mappedDegrees = Mathf.Clamp(
            angleDegrees * rotationGain,
            -config.MaxClutchRotationDegrees,
            config.MaxClutchRotationDegrees);
        Quaternion mappedDeltaRotation = axis.sqrMagnitude > 0.000001f
            ? Quaternion.AngleAxis(mappedDegrees, axis.normalized)
            : Quaternion.identity;

        // 相对旋转左乘 TCP 基准，表示把“手柄相对基准的世界旋转变化”
        // 施加到离合瞬间的工具姿态，而不是逐帧欧拉角累加。
        Quaternion targetRotation = Normalize(mappedDeltaRotation * tcpAnchorRotation);

        lastTargetPosition = targetPosition;
        lastTargetRotation = targetRotation;
        return CreateActiveResult(
            wasAnchoredThisStep: false,
            controllerDistanceMeters: controllerDistance,
            controllerAngleDegrees: Mathf.Abs(angleDegrees),
            translationGain: translationGain,
            rotationGain: rotationGain,
            mappedAngleDegrees: Mathf.Abs(mappedDegrees),
            targetPosition: targetPosition,
            targetRotation: targetRotation);
    }

    private float EvaluateControllerTranslationGain(float controllerDistanceMeters)
    {
        if (controllerDistanceMeters <= 0.001001f)
        {
            // 1 mm 以内保持近端增益，满足精细采集时“小动作就是细动作”的手感。
            return config.TranslationNearGain;
        }

        return config.EvaluateTranslationGain(controllerDistanceMeters);
    }

    private float EvaluateControllerRotationGain(float controllerAngleDegrees)
    {
        if (controllerAngleDegrees <= 1.001f)
        {
            // 1° 以内保持近端增益，避免极小旋转被过渡曲线提前放大。
            return config.RotationNearGain;
        }

        return config.EvaluateRotationGain(controllerAngleDegrees);
    }

    private void CaptureAnchor(Ur5Continuous6DofStepInput input)
    {
        controllerAnchorPosition = input.ControllerPosition;
        controllerAnchorRotation = Normalize(input.ControllerRotation);
        tcpAnchorPosition = input.ActualTcpPosition;
        tcpAnchorRotation = Normalize(input.ActualTcpRotation);
        lastTargetPosition = tcpAnchorPosition;
        lastTargetRotation = tcpAnchorRotation;
        hasAnchor = true;
        State = Ur5TeleopControllerState.Clutched;
        FaultReason = Ur5Continuous6DofFaultReason.None;
    }

    private Ur5Continuous6DofStepResult EnterFault(Ur5Continuous6DofFaultReason reason)
    {
        hasAnchor = false;
        faultLatched = true;
        State = Ur5TeleopControllerState.Fault;
        FaultReason = reason;
        return CreateInactiveResult(Ur5TeleopControllerState.Fault, reason);
    }

    private Ur5Continuous6DofStepResult CreateActiveResult(
        bool wasAnchoredThisStep,
        float controllerDistanceMeters,
        float controllerAngleDegrees,
        float translationGain,
        float rotationGain,
        float mappedAngleDegrees,
        Vector3 targetPosition,
        Quaternion targetRotation)
    {
        State = Ur5TeleopControllerState.Clutched;
        FaultReason = Ur5Continuous6DofFaultReason.None;
        return new Ur5Continuous6DofStepResult(
            state: State,
            faultReason: FaultReason,
            isMotionCommandActive: true,
            wasAnchoredThisStep: wasAnchoredThisStep,
            controllerDistanceMeters: controllerDistanceMeters,
            controllerAngleDegrees: controllerAngleDegrees,
            translationGain: translationGain,
            rotationGain: rotationGain,
            mappedAngleDegrees: mappedAngleDegrees,
            targetPosition: targetPosition,
            targetRotation: targetRotation);
    }

    private Ur5Continuous6DofStepResult CreateInactiveResult(
        Ur5TeleopControllerState state,
        Ur5Continuous6DofFaultReason reason)
    {
        return new Ur5Continuous6DofStepResult(
            state: state,
            faultReason: reason,
            isMotionCommandActive: false,
            wasAnchoredThisStep: false,
            controllerDistanceMeters: 0.0f,
            controllerAngleDegrees: 0.0f,
            translationGain: 0.0f,
            rotationGain: 0.0f,
            mappedAngleDegrees: 0.0f,
            targetPosition: lastTargetPosition,
            targetRotation: lastTargetRotation);
    }

    private static Ur5Continuous6DofFaultReason ValidateInput(Ur5Continuous6DofStepInput input)
    {
        if (!input.IsInputPoseValid
            || !IsFinite(input.ControllerPosition)
            || !IsValidRotation(input.ControllerRotation))
        {
            return Ur5Continuous6DofFaultReason.InvalidControllerPose;
        }

        if (!input.IsRobotStateValid
            || !IsFinite(input.ActualTcpPosition)
            || !IsValidRotation(input.ActualTcpRotation))
        {
            return Ur5Continuous6DofFaultReason.InvalidRobotPose;
        }

        if (!input.IsSafetyAccepted)
        {
            return Ur5Continuous6DofFaultReason.SafetyRejected;
        }

        return Ur5Continuous6DofFaultReason.None;
    }

    private static Quaternion Normalize(Quaternion value)
    {
        float magnitude = Mathf.Sqrt(
            value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w);
        return new Quaternion(
            value.x / magnitude,
            value.y / magnitude,
            value.z / magnitude,
            value.w / magnitude);
    }

    private static bool IsValidRotation(Quaternion value)
    {
        float sqrMagnitude =
            value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w;
        return IsFinite(value.x)
            && IsFinite(value.y)
            && IsFinite(value.z)
            && IsFinite(value.w)
            && sqrMagnitude > 0.000001f;
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

public enum Ur5LeftSafetyPoseState
{
    Idle,
    ButtonHeld,
    SnapDownActive,
    ReadyPoseActive
}

public readonly struct Ur5LeftSafetyPoseStepInput
{
    public readonly bool RightGripHeld;
    public readonly bool LeftPoseValid;
    public readonly bool LeftGripHeld;
    public readonly bool PrimaryPressed;
    public readonly bool SnapTargetReached;
    public readonly bool ReadyPoseActive;
    public readonly float DeltaTimeSeconds;

    public Ur5LeftSafetyPoseStepInput(
        bool rightGripHeld,
        bool leftPoseValid,
        bool leftGripHeld,
        bool primaryPressed,
        bool snapTargetReached,
        bool readyPoseActive,
        float deltaTimeSeconds)
    {
        RightGripHeld = rightGripHeld;
        LeftPoseValid = leftPoseValid;
        LeftGripHeld = leftGripHeld;
        PrimaryPressed = primaryPressed;
        SnapTargetReached = snapTargetReached;
        ReadyPoseActive = readyPoseActive;
        DeltaTimeSeconds = deltaTimeSeconds;
    }
}

public readonly struct Ur5LeftSafetyPoseStepResult
{
    public readonly Ur5LeftSafetyPoseState State;
    public readonly bool BlocksRightGrip;
    public readonly bool RequestSnapDown;
    public readonly bool RequestReadyPose;
    public readonly bool CancelReadyPose;
    public readonly bool SnapTimedOut;

    public Ur5LeftSafetyPoseStepResult(
        Ur5LeftSafetyPoseState state,
        bool blocksRightGrip,
        bool requestSnapDown,
        bool requestReadyPose,
        bool cancelReadyPose,
        bool snapTimedOut)
    {
        State = state;
        BlocksRightGrip = blocksRightGrip;
        RequestSnapDown = requestSnapDown;
        RequestReadyPose = requestReadyPose;
        CancelReadyPose = cancelReadyPose;
        SnapTimedOut = snapTimedOut;
    }
}

/// <summary>
/// 左手安全姿态的纯逻辑状态机。Unity 适配层负责读取 XR 按键和执行 IK，
/// 本类只区分短按朝下、长按 Ready Pose、超时和右手写入互斥。
/// </summary>
public sealed class Ur5LeftSafetyPoseController
{
    private readonly float readyPoseHoldSeconds;
    private readonly float snapTimeoutSeconds;
    private float buttonHeldSeconds;
    private float snapElapsedSeconds;

    public Ur5LeftSafetyPoseState State { get; private set; }
    public bool BlocksRightGrip => State != Ur5LeftSafetyPoseState.Idle;

    public Ur5LeftSafetyPoseController(float readyPoseHoldSeconds, float snapTimeoutSeconds)
    {
        this.readyPoseHoldSeconds = NonNegative(readyPoseHoldSeconds);
        this.snapTimeoutSeconds = NonNegative(snapTimeoutSeconds);
        Reset();
    }

    public void Reset()
    {
        State = Ur5LeftSafetyPoseState.Idle;
        buttonHeldSeconds = 0.0f;
        snapElapsedSeconds = 0.0f;
    }

    public Ur5LeftSafetyPoseStepResult Step(Ur5LeftSafetyPoseStepInput input)
    {
        bool requestSnapDown = false;
        bool requestReadyPose = false;
        bool cancelReadyPose = false;
        bool snapTimedOut = false;
        float deltaTime = NonNegative(input.DeltaTimeSeconds);

        switch (State)
        {
            case Ur5LeftSafetyPoseState.Idle:
                if (!input.RightGripHeld
                    && input.LeftPoseValid
                    && input.LeftGripHeld
                    && input.PrimaryPressed)
                {
                    State = Ur5LeftSafetyPoseState.ButtonHeld;
                    buttonHeldSeconds = deltaTime;
                    if (buttonHeldSeconds >= readyPoseHoldSeconds)
                    {
                        State = Ur5LeftSafetyPoseState.ReadyPoseActive;
                        requestReadyPose = true;
                    }
                }
                break;

            case Ur5LeftSafetyPoseState.ButtonHeld:
                if (!input.LeftPoseValid || !input.LeftGripHeld)
                {
                    Reset();
                    break;
                }

                if (!input.PrimaryPressed)
                {
                    // 短按必须等 X 释放后才发朝下请求，避免长按过程中先旋转再切 Ready Pose。
                    State = Ur5LeftSafetyPoseState.SnapDownActive;
                    snapElapsedSeconds = 0.0f;
                    requestSnapDown = true;
                    break;
                }

                buttonHeldSeconds += deltaTime;
                if (buttonHeldSeconds >= readyPoseHoldSeconds)
                {
                    State = Ur5LeftSafetyPoseState.ReadyPoseActive;
                    requestReadyPose = true;
                }
                break;

            case Ur5LeftSafetyPoseState.SnapDownActive:
                if (input.SnapTargetReached)
                {
                    Reset();
                    break;
                }

                snapElapsedSeconds += deltaTime;
                if (snapElapsedSeconds >= snapTimeoutSeconds)
                {
                    Reset();
                    snapTimedOut = true;
                }
                break;

            case Ur5LeftSafetyPoseState.ReadyPoseActive:
                if (!input.PrimaryPressed)
                {
                    cancelReadyPose = input.ReadyPoseActive;
                    Reset();
                }
                break;
        }

        return new Ur5LeftSafetyPoseStepResult(
            State,
            BlocksRightGrip,
            requestSnapDown,
            requestReadyPose,
            cancelReadyPose,
            snapTimedOut);
    }

    private static float NonNegative(float value)
    {
        return float.IsNaN(value) || float.IsInfinity(value) ? 0.0f : System.Math.Max(0.0f, value);
    }
}

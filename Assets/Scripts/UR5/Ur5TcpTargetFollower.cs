using UnityEngine;
using UnityEngine.Serialization;

[DefaultExecutionOrder(-50)]
public class Ur5TcpTargetFollower : MonoBehaviour
{
    public enum IkSolverMode
    {
        DampedLeastSquares,
        CcdFallback
    }

    [Header("References")]
    public Ur5ArticulationJointController jointController;
    public Transform robotRoot;
    public Transform tcpTarget;
    public Transform endEffector;
    public Ur5JointTrajectoryPlayer trajectoryPlayer;

    [Header("Gripper TCP")]
    [Tooltip("Use the midpoint between the two Robotiq pads as the task-space TCP.")]
    public bool useGripperPadCenter;
    public Transform leftGripperPad;
    public Transform rightGripperPad;
    public string leftGripperPadName = "left_pad_link";
    public string rightGripperPadName = "right_pad_link";
    [Tooltip("Robotiq base used with the two pads to derive the physical grasp approach axis at runtime.")]
    public Transform gripperBase;
    public string gripperBaseName = "robotiq_base_link";
    public bool usePadGeometryCenter = true;
    [Tooltip("Use the physical frame built from the two pads for all orientation constraints. This keeps the jaw direction fixed even when tool0 has a different URDF axis convention.")]
    public bool usePhysicalGraspFrameForOrientation = true;

    [Header("End Effector Search")]
    public string[] endEffectorNameHints =
    {
        "tool0",
        "ee_link",
        "tcp",
        "wrist_3_link",
        "wrist_3"
    };

    [Header("IK")]
    public bool followTarget = true;
    public IkSolverMode solverMode = IkSolverMode.DampedLeastSquares;
    public float positionTolerance = 0.008f;
    [Tooltip("Keep this at 1. Articulation poses update after FixedUpdate, so repeated CCD passes use stale geometry and cause oscillation.")]
    public int solverIterationsPerFixedUpdate = 1;
    public float angleBlend = 0.48f;
    [Tooltip("Legacy per-FixedUpdate value retained for serialized scenes. Runtime control uses maxJointSpeedDegreesPerSecond.")]
    public float maxJointStepDegrees = 1.45f;
    [Tooltip("Maximum IK command change in degrees per second. 252 preserves the previous 2.8 degree step at 90 Hz.")]
    public float maxJointSpeedDegreesPerSecond = 252.0f;
    [Tooltip("Ignores microscopic IK deltas that usually come from tracking noise rather than intentional motion.")]
    public float minimumJointDeltaDegrees = 0.015f;
    public bool adaptivePositionSpeed = true;
    public float fullSpeedPositionError = 0.12f;
    [Tooltip("Optional legacy IK-to-drive lead window in degrees. Set to 0 to rely on the Articulation measured-joint lead guard only.")]
    public float maximumCommandLeadDegrees = 2.00f;
    public float maxReachError = 1.5f;
    public bool clampToDriveLimits = true;

    [Header("Trajectory-Style Joint Assignment")]
    [Tooltip("Submit IK joint setpoints at a fixed cadence instead of every physics frame. This mirrors Unity Robotics Hub trajectory playback and avoids servo chatter.")]
    public bool useTimedJointAssignments = true;
    public float jointAssignmentIntervalSeconds = 0.016f;

    [Header("Damped Least Squares IK")]
    [Tooltip("Higher values trade responsiveness for stability near singular configurations.")]
    public float dlsDamping = 0.32f;
    [Tooltip("Treats one radian of orientation error as this many meters of task error.")]
    public float dlsOrientationWeight = 0.55f;
    public float dlsGain = 0.46f;
    [Tooltip("Bias orientation correction toward wrist joints to avoid shoulder/elbow solution jumps.")]
    public bool preferWristForOrientation = true;
    [Range(0.0f, 1.0f)] public float proximalOrientationWeight = 0.25f;
    [Tooltip("Right-hand translation keeps the current tool attitude as a high-priority task. Higher values prevent the gripper from tilting when the base/shoulder moves.")]
    public float translationOrientationHoldWeight = 8.00f;
    [Tooltip("0 = no smoothing, 1 = keep the previous IK delta. Use small values to reduce twitching.")]
    [Range(0.0f, 0.95f)] public float jointDeltaSmoothing = 0.45f;

    [Header("静止抗抖")]
    [Tooltip("TCP 目标保持不动一小段时间后，降低 IK 修正增益，抑制末端到位后的来回摆动。")]
    public bool enableStationaryDlsDamping = false;
    public float stationaryDampingStartSeconds = 0.06f;
    [Range(0.05f, 1.0f)] public float stationaryDlsGainMultiplier = 0.45f;

    [Header("End Effector Orientation")]
    public bool followTargetRotation = true;
    [Range(1, 3)] public int wristJointCount = 3;
    public float rotationToleranceDegrees = 1.20f;
    public float rotationBlend = 0.70f;
    [Tooltip("Legacy per-FixedUpdate value retained for serialized scenes. Runtime control uses maxWristSpeedDegreesPerSecond.")]
    public float maxWristStepDegrees = 2.00f;
    [Tooltip("Maximum wrist IK command change in degrees per second.")]
    public float maxWristSpeedDegreesPerSecond = 360.0f;
    [Tooltip("When only the right controller is translating, fully pause orientation IK so wrist joints do not twitch while chasing pose noise.")]
    public bool suppressRotationOnlyIkDuringPositionControl = true;

    [Header("Grasp Assist Precision")]
    [Tooltip("Only while grasp assist is active, continue IK down to this two-pad TCP position error before closing the gripper.")]
    public float graspAssistPositionTolerance = 0.0015f;
    [Tooltip("Only while grasp assist is active, continue IK down to this grasp-frame orientation error before closing the gripper.")]
    public float graspAssistRotationToleranceDegrees = 0.35f;

    [Header("Precision Assembly Tracking")]
    [Tooltip("Applies millimetre-level convergence whenever XR teleoperation owns the TCP. It is not a separate operator mode.")]
    [FormerlySerializedAs("enableFineAssemblyControl")]
    public bool enablePrecisionAssemblyTracking = true;
    [Tooltip("Two-pad TCP error required during manual XR assembly control. Keep this above the effective Articulation and model resolution.")]
    [FormerlySerializedAs("fineAssemblyPositionTolerance")]
    public float precisionAssemblyPositionTolerance = 0.0010f;
    [Tooltip("Physical grasp-frame rotation error required during manual XR assembly control.")]
    [FormerlySerializedAs("fineAssemblyRotationToleranceDegrees")]
    public float precisionAssemblyRotationToleranceDegrees = 0.25f;
    [Tooltip("Target must move farther than this to release the settled-pose hold during manual XR assembly control.")]
    [FormerlySerializedAs("fineAssemblyTargetChangeEpsilonMeters")]
    public float precisionAssemblyTargetChangeEpsilonMeters = 0.00015f;
    [FormerlySerializedAs("fineAssemblyTargetChangeEpsilonDegrees")]
    public float precisionAssemblyTargetChangeEpsilonDegrees = 0.04f;
    [Tooltip("Pose error at which a stationary manual XR target may lock its measured joint pose.")]
    [FormerlySerializedAs("fineAssemblySettledPositionError")]
    public float precisionAssemblySettledPositionError = 0.0012f;
    [FormerlySerializedAs("fineAssemblySettledRotationErrorDegrees")]
    public float precisionAssemblySettledRotationErrorDegrees = 0.30f;

    [Header("Quest Idle Hold")]
    public Quest3TcpTargetController questController;
    public Ur5CartesianVelocityTeleopController velocityTeleop;
    [Tooltip("Keeps IK active while the scripted PreGrasp -> Grasp -> Lift sequence is moving the TCP target.")]
    public Ur5GraspAssistController graspAssist;
    public bool pauseIkWhenQuestControllerIdle = true;
    public bool pauseIkWhenVelocityTeleopIdle = true;
    [Tooltip("Legacy behavior. Safe release is now handled once, with an explicit residual threshold.")]
    public bool snapTargetToActualPoseWhenQuestReleased = false;
    public bool holdJointPoseWhenQuestReleased = true;

    [Header("精细定位释放收敛")]
    [Tooltip("Legacy compatibility setting. Safe release below uses the explicit residual window instead.")]
    public bool finishVelocityTargetAfterRelease = false;
    public float velocityReleasePositionTolerance = 0.003f;
    public float velocityReleaseRotationToleranceDegrees = 0.50f;
    public float velocityReleaseSettleTimeoutSeconds = 2.0f;
    [Tooltip("A release may finish only this small residual. Larger residuals are stopped at the actual TCP immediately.")]
    public float safeReleaseMaximumResidualMeters = 0.003f;
    public float safeReleaseMaximumResidualDegrees = 0.50f;
    [Tooltip("Maximum time allowed to settle a residual already within the safe release window.")]
    public float safeReleaseSettleTimeoutSeconds = 0.20f;

    [Header("Settled Target Hold")]
    public bool holdJointPoseWhenTargetSettled = true;
    public float targetStationaryHoldSeconds = 0.12f;
    public float targetStationaryPositionEpsilon = 0.0015f;
    public float targetStationaryRotationEpsilonDegrees = 0.30f;
    public float settledPositionError = 0.010f;
    public float settledRotationErrorDegrees = 1.50f;

    [Header("Ready Pose")]
    [Tooltip("A safe, gripper-down starting configuration for repeated pick-and-place trials. It is reached with a rate-limited joint trajectory, not an instantaneous reset.")]
    public bool enableReadyPose = true;
    [Tooltip("UR5 joint targets in degrees: shoulder pan, shoulder lift, elbow, wrist 1, wrist 2, wrist 3.")]
    public float[] readyPoseJointDegrees = { 0.0f, -90.0f, 90.0f, -90.0f, -90.0f, 0.0f };
    [Tooltip("Maximum change of each ready-pose joint target per second.")]
    public float readyPoseMaxJointSpeedDegreesPerSecond = 80.0f;
    [Tooltip("The move completes only after each measured joint is within this tolerance of its ready target.")]
    public float readyPoseJointToleranceDegrees = 1.5f;

    [Header("Startup Alignment")]
    [Tooltip("Start the target at the current TCP so the robot only moves after user input.")]
    public bool snapTargetToEndEffectorOnStart = true;
    public bool snapTargetRotationOnStart = true;

    [Header("Joint Axes")]
    public Vector3 defaultJointLocalAxis = Vector3.right;
    public Vector3[] jointLocalAxes;

    [Header("Debug")]
    public bool drawDebug = true;
    public bool logStatus = true;
    public TcpTargetWriteMonitor targetWriteMonitor;

    private bool loggedReady;
    private bool loggedMissingReferences;
    private bool wasControllerCommandActive;
    private bool wasVelocityTeleopCommandActive;
    private bool isControllerIdleHoldActive;
    private bool isSettledTargetHoldActive;
    private bool hasLastTargetPose;
    private Vector3 lastTargetPosition;
    private Quaternion lastTargetRotation = Quaternion.identity;
    private float targetStationaryTime;
    private float nextJointAssignmentTime;
    private float[] smoothedJointDeltaDegrees = new float[0];
    private float[] workingJointTargetsDegrees = new float[0];
    private int workingJointCount;
    private bool workingJointWaypointChanged;
    private bool hasToolToGraspRotation;
    private Quaternion toolToGraspRotation = Quaternion.identity;
    private bool isReadyPoseActive;
    private bool safeReleaseSettlePending;
    private float safeReleaseSettleElapsedSeconds;

    public float PositionError { get; private set; }
    public float RotationErrorDegrees { get; private set; }
    public Vector3 ControlPointPosition => GetControlPointPosition();
    public Quaternion ActualGraspRotation => GetActualGraspRotation();
    /// <summary>
    /// Axis passing through the two-pad midpoint and the gripper body. Rotating
    /// around it changes the jaw alignment without tilting the grasp approach.
    /// </summary>
    public Vector3 GripperCenterSymmetryAxisWorld => GetActualGraspRotation() * Vector3.forward;
    public bool IsReadyPoseActive => isReadyPoseActive;
    public int IkFailureCount { get; private set; }
    public int ConsecutiveIkFailureCount { get; private set; }
    public string LastIkFailureReason { get; private set; } = string.Empty;
    public bool IsNearSingularity { get; private set; }
    public float LastDlsMinimumPivot { get; private set; }
    public bool WasIkCommandLeadLimited { get; private set; }

    /// <summary>
    /// 将命令 TCP 的姿态对齐到当前实际抓取坐标系，但不改变其位置。
    /// 抓取坐标系由夹爪基座、两指中心和两指连线定义；它与用户视觉上看到的
    /// 两个夹爪面完全一致，不依赖 tool0 在 URDF 中的局部轴约定。
    /// </summary>
    public void HoldTargetRotationAtCurrentGraspFrame(string writer = "Follower")
    {
        ResolveReferences();
        if (tcpTarget == null || endEffector == null)
        {
            return;
        }

        tcpTarget.rotation = GetToolRotationForGraspRotation(GetActualGraspRotation());
        if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite(writer);
        }
        RotationErrorDegrees = 0.0f;
        ResetJointDeltaSmoothing();
    }

    /// <summary>
    /// 兼容旧调用。新的控制逻辑应调用 <see cref="HoldTargetRotationAtCurrentGraspFrame"/>。
    /// </summary>
    public void HoldTargetRotationAtCurrentTool()
    {
        HoldTargetRotationAtCurrentGraspFrame();
    }

    private void Awake()
    {
        ResolveReferences();
    }

    private void Start()
    {
        ResolveReferences();
        SnapTargetToEndEffector();
        LogReadyState();
    }

    private void FixedUpdate()
    {
        if (!followTarget)
        {
            return;
        }

        ResolveReferences();

        if (!HasRequiredReferences())
        {
            LogMissingReferences();
            return;
        }

        LogReadyState();
        if (isReadyPoseActive)
        {
            StepTowardReadyPose();
            return;
        }

        if (ShouldPauseForControllerIdle())
        {
            EnterControllerIdleHold();
            return;
        }

        UpdateTargetStationaryState(Time.fixedDeltaTime);
        isControllerIdleHoldActive = false;
        wasControllerCommandActive = IsAnyControllerCommandActive();
        if (ShouldWaitForNextJointAssignment())
        {
            UpdateTrackingErrorsOnly();
            return;
        }

        StepTowardTarget();
    }

    private void OnValidate()
    {
        minimumJointDeltaDegrees = Mathf.Max(0.0f, minimumJointDeltaDegrees);
        dlsDamping = Mathf.Max(0.0f, dlsDamping);
        dlsOrientationWeight = Mathf.Max(0.0f, dlsOrientationWeight);
        translationOrientationHoldWeight = Mathf.Max(0.0f, translationOrientationHoldWeight);
        dlsGain = Mathf.Max(0.0f, dlsGain);
        stationaryDampingStartSeconds = Mathf.Max(0.0f, stationaryDampingStartSeconds);
        stationaryDlsGainMultiplier = Mathf.Clamp(stationaryDlsGainMultiplier, 0.05f, 1.0f);
        targetStationaryHoldSeconds = Mathf.Max(0.0f, targetStationaryHoldSeconds);
        targetStationaryPositionEpsilon = Mathf.Max(0.0f, targetStationaryPositionEpsilon);
        targetStationaryRotationEpsilonDegrees = Mathf.Max(0.0f, targetStationaryRotationEpsilonDegrees);
        settledPositionError = Mathf.Max(0.0f, settledPositionError);
        settledRotationErrorDegrees = Mathf.Max(0.0f, settledRotationErrorDegrees);
        jointAssignmentIntervalSeconds = Mathf.Max(0.0f, jointAssignmentIntervalSeconds);
        readyPoseMaxJointSpeedDegreesPerSecond = Mathf.Max(0.0f, readyPoseMaxJointSpeedDegreesPerSecond);
        readyPoseJointToleranceDegrees = Mathf.Max(0.01f, readyPoseJointToleranceDegrees);
        velocityReleasePositionTolerance = Mathf.Max(0.0f, velocityReleasePositionTolerance);
        velocityReleaseRotationToleranceDegrees = Mathf.Max(0.0f, velocityReleaseRotationToleranceDegrees);
        velocityReleaseSettleTimeoutSeconds = Mathf.Max(0.0f, velocityReleaseSettleTimeoutSeconds);
        maxJointSpeedDegreesPerSecond = Mathf.Max(0.0f, maxJointSpeedDegreesPerSecond);
        maxWristSpeedDegreesPerSecond = Mathf.Max(0.0f, maxWristSpeedDegreesPerSecond);
        safeReleaseMaximumResidualMeters = Mathf.Max(0.0f, safeReleaseMaximumResidualMeters);
        safeReleaseMaximumResidualDegrees = Mathf.Max(0.0f, safeReleaseMaximumResidualDegrees);
        safeReleaseSettleTimeoutSeconds = Mathf.Max(0.0f, safeReleaseSettleTimeoutSeconds);
        graspAssistPositionTolerance = Mathf.Max(0.0005f, graspAssistPositionTolerance);
        graspAssistRotationToleranceDegrees = Mathf.Max(0.05f, graspAssistRotationToleranceDegrees);
        precisionAssemblyPositionTolerance = Mathf.Max(0.0005f, precisionAssemblyPositionTolerance);
        precisionAssemblyRotationToleranceDegrees = Mathf.Max(0.05f, precisionAssemblyRotationToleranceDegrees);
        precisionAssemblyTargetChangeEpsilonMeters = Mathf.Max(0.00001f, precisionAssemblyTargetChangeEpsilonMeters);
        precisionAssemblyTargetChangeEpsilonDegrees = Mathf.Max(0.001f, precisionAssemblyTargetChangeEpsilonDegrees);
        precisionAssemblySettledPositionError = Mathf.Max(0.0005f, precisionAssemblySettledPositionError);
        precisionAssemblySettledRotationErrorDegrees = Mathf.Max(0.05f, precisionAssemblySettledRotationErrorDegrees);
    }

    private void OnDisable()
    {
        isReadyPoseActive = false;
        if (jointController != null)
        {
            jointController.SetReadyPoseDriveLeadProfile(false);
        }
        isSettledTargetHoldActive = false;
        ClearTrajectoryQueue();
    }

    /// <summary>
    /// Starts the rate-limited joint trajectory to the configured gripper-down
    /// ready pose. Manual TCP IK is suspended until it arrives or is cancelled.
    /// </summary>
    public bool BeginReadyPose()
    {
        ResolveReferences();
        int jointCount = jointController != null ? Mathf.Min(6, jointController.JointCount) : 0;
        if (!enableReadyPose
            || jointCount <= 0
            || readyPoseJointDegrees == null
            || readyPoseJointDegrees.Length < jointCount)
        {
            return false;
        }

        ClearTrajectoryQueue();
        ResetJointDeltaSmoothing();
        jointController.SetReadyPoseDriveLeadProfile(true);
        isReadyPoseActive = true;
        return true;
    }

    /// <summary>
    /// Stops an in-progress ready-pose move at the current measured joint pose.
    /// Used when the operator releases the hold-to-run X control early.
    /// </summary>
    public void CancelReadyPose()
    {
        if (!isReadyPoseActive)
        {
            return;
        }

        isReadyPoseActive = false;
        if (jointController != null)
        {
            jointController.SetReadyPoseDriveLeadProfile(false);
        }
        HoldCurrentJointsAndClearTrajectory();
        ResetJointDeltaSmoothing();
        SnapTargetToEndEffector();
    }

    /// <summary>
    /// 以当前测得关节角立即冻结机械臂，并将命令 TCP 对齐到实际两指中心。
    /// 用于速度式遥操作的“手停即停”：不能继续追赶此前尚未到达的 TCP 目标。
    /// </summary>
    public void FreezeAtCurrentPose()
    {
        ResolveReferences();
        if (jointController == null || tcpTarget == null || endEffector == null)
        {
            return;
        }

        HoldCurrentJointsAndClearTrajectory();
        tcpTarget.position = GetControlPointPosition();
        if (followTargetRotation)
        {
            HoldTargetRotationAtCurrentGraspFrame("FollowerFreeze");
        }
        else if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite("FollowerFreeze");
        }

        TcpTargetWorkspaceLimiter workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
        if (workspaceLimiter != null)
        {
            workspaceLimiter.PreserveCurrentTargetPose();
        }

        hasLastTargetPose = true;
        lastTargetPosition = tcpTarget.position;
        lastTargetRotation = tcpTarget.rotation;
        targetStationaryTime = 0.0f;
        ResetJointDeltaSmoothing();
        isSettledTargetHoldActive = true;
        nextJointAssignmentTime = Time.time;
    }

    private void OnDrawGizmos()
    {
        if (!drawDebug || tcpTarget == null || endEffector == null)
        {
            return;
        }

        Gizmos.color = Color.yellow;
        Vector3 controlPoint = GetControlPointPosition();
        Gizmos.DrawLine(controlPoint, tcpTarget.position);
        Gizmos.DrawWireSphere(controlPoint, 0.025f);
    }

    private void ResolveReferences()
    {
        if (jointController == null)
        {
            jointController = GetComponent<Ur5ArticulationJointController>();
        }

        if (jointController == null && robotRoot != null)
        {
            jointController = robotRoot.GetComponent<Ur5ArticulationJointController>();
        }

        if (trajectoryPlayer == null && jointController != null)
        {
            trajectoryPlayer = jointController.GetComponent<Ur5JointTrajectoryPlayer>();
        }

        if (trajectoryPlayer == null && robotRoot != null)
        {
            trajectoryPlayer = robotRoot.GetComponent<Ur5JointTrajectoryPlayer>();
        }

        if (robotRoot == null && jointController != null)
        {
            robotRoot = jointController.robotRoot != null ? jointController.robotRoot : jointController.transform;
        }

        if (tcpTarget == null)
        {
            GameObject foundTarget = GameObject.Find("TcpTarget");
            if (foundTarget != null)
            {
                tcpTarget = foundTarget.transform;
            }
        }

        if (targetWriteMonitor == null && tcpTarget != null)
        {
            targetWriteMonitor = tcpTarget.GetComponent<TcpTargetWriteMonitor>();
        }

        if (questController == null && tcpTarget != null)
        {
            questController = tcpTarget.GetComponent<Quest3TcpTargetController>();
        }

        if (velocityTeleop == null)
        {
            velocityTeleop = FindObjectOfType<Ur5CartesianVelocityTeleopController>();
        }

        if (graspAssist == null)
        {
            graspAssist = FindObjectOfType<Ur5GraspAssistController>();
        }

        if (endEffector == null && robotRoot != null)
        {
            endEffector = FindEndEffector(robotRoot);
        }

        if (useGripperPadCenter && robotRoot != null)
        {
            if (leftGripperPad == null)
            {
                leftGripperPad = FindByNameHint(robotRoot, leftGripperPadName);
            }

            if (rightGripperPad == null)
            {
                rightGripperPad = FindByNameHint(robotRoot, rightGripperPadName);
            }

            if (gripperBase == null)
            {
                gripperBase = FindByNameHint(robotRoot, gripperBaseName);
            }
        }
    }

    private bool ShouldPauseForControllerIdle()
    {
        // The autonomous grasp sequence owns TcpTarget while no hand grip is
        // necessarily held. Do not snap the target back to the current pose or
        // pause IK between its pre-grasp, vertical descent, and lift stages.
        if (graspAssist != null && graspAssist.IsAssistActive)
        {
            return false;
        }

        if (pauseIkWhenVelocityTeleopIdle
            && velocityTeleop != null
            && velocityTeleop.enabled)
        {
            if (velocityTeleop.IsCommandActive)
            {
                wasVelocityTeleopCommandActive = true;
                safeReleaseSettlePending = false;
                safeReleaseSettleElapsedSeconds = 0.0f;
                return false;
            }

            if (wasVelocityTeleopCommandActive)
            {
                wasVelocityTeleopCommandActive = false;
                BeginSafeRelease();
            }

            if (safeReleaseSettlePending)
            {
                UpdateTrackingErrorsOnly();
                safeReleaseSettleElapsedSeconds += Time.fixedDeltaTime;
                bool reachedTarget = PositionError <= GetActivePositionTolerance()
                    && RotationErrorDegrees <= GetActiveRotationToleranceDegrees();
                bool timedOut = safeReleaseSettleElapsedSeconds >= safeReleaseSettleTimeoutSeconds;
                if (!reachedTarget && !timedOut)
                {
                    return false;
                }

                safeReleaseSettlePending = false;
                if (!reachedTarget)
                {
                    SnapTargetToActualPose("FollowerSafeReleaseTimeout");
                }
            }

            return true;
        }

        return pauseIkWhenQuestControllerIdle
            && questController != null
            && questController.IsDeviceValid
            && !questController.IsClutched;
    }

    private bool IsAnyControllerCommandActive()
    {
        bool questActive = questController != null && questController.IsClutched;
        bool velocityActive = velocityTeleop != null && velocityTeleop.enabled && velocityTeleop.IsCommandActive;
        return questActive || velocityActive;
    }

    private void EnterControllerIdleHold()
    {
        if (isControllerIdleHoldActive && !wasControllerCommandActive)
        {
            return;
        }

        if (holdJointPoseWhenQuestReleased && jointController != null)
        {
            HoldCurrentJointsAndClearTrajectory();
        }
        else
        {
            ClearTrajectoryQueue();
        }

        ResetJointDeltaSmoothing();
        PositionError = 0.0f;
        RotationErrorDegrees = 0.0f;
        wasControllerCommandActive = false;
        isControllerIdleHoldActive = true;
        nextJointAssignmentTime = Time.time;
    }

    private void BeginSafeRelease()
    {
        UpdateTrackingErrorsOnly();
        bool residualIsSmall = PositionError <= safeReleaseMaximumResidualMeters
            && RotationErrorDegrees <= safeReleaseMaximumResidualDegrees;
        if (residualIsSmall)
        {
            safeReleaseSettlePending = true;
            safeReleaseSettleElapsedSeconds = 0.0f;
            return;
        }

        // The user released Grip while a visible target lead remained. Cancel
        // that lead once and explicitly, rather than letting multiple writers
        // repeatedly pull TcpTarget back to the physical arm.
        SnapTargetToActualPose("FollowerSafeRelease");
    }

    private void SnapTargetToActualPose(string writer)
    {
        if (tcpTarget == null)
        {
            return;
        }

        tcpTarget.position = GetControlPointPosition();
        if (followTargetRotation && endEffector != null)
        {
            HoldTargetRotationAtCurrentGraspFrame(writer);
        }

        TcpTargetWorkspaceLimiter workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
        if (workspaceLimiter != null)
        {
            workspaceLimiter.PreserveCurrentTargetPose();
        }

        if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite(writer);
        }
    }

    private bool HasRequiredReferences()
    {
        return jointController != null
            && jointController.JointCount > 0
            && tcpTarget != null
            && endEffector != null;
    }

    private void SnapTargetToEndEffector()
    {
        if (!snapTargetToEndEffectorOnStart || tcpTarget == null || endEffector == null)
        {
            return;
        }

        tcpTarget.position = GetControlPointPosition();
        TcpTargetWorkspaceLimiter workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
        if (workspaceLimiter != null)
        {
            workspaceLimiter.PreserveCurrentTargetPose();
        }

        if (snapTargetRotationOnStart)
        {
            HoldTargetRotationAtCurrentGraspFrame("FollowerStartup");
        }
        else if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite("FollowerStartup");
        }
    }

    private void StepTowardTarget()
    {
        WasIkCommandLeadLimited = false;
        Vector3 error = UpdateTrackingErrorsOnly();

        if (PositionError > maxReachError)
        {
            return;
        }

        if (ShouldHoldSettledTarget())
        {
            HoldCurrentPoseAtSettledTarget();
            return;
        }

        int jointCount = Mathf.Min(6, jointController.JointCount);
        BeginJointWaypoint(jointCount);
        if (IsTranslationOrientationHoldActive())
        {
            StepStrictTranslationWithLockedOrientation(jointCount);
            CommitJointWaypoint();
            return;
        }

        if (solverMode == IkSolverMode.DampedLeastSquares)
        {
            ApplyDampedLeastSquaresStep(jointCount, error);
            CommitJointWaypoint();
            return;
        }

        if (PositionError > GetActivePositionTolerance())
        {
            // Drives are applied by the physics simulation after this method returns.
            // More CCD passes here would repeatedly add corrections from the same pose.
            int iterationCount = 1;

            for (int iteration = 0; iteration < iterationCount; iteration++)
            {
                for (int i = jointCount - 1; i >= 0; i--)
                {
                    ApplyCcdStep(i);
                }

                error = tcpTarget.position - GetControlPointPosition();
                PositionError = error.magnitude;
                if (PositionError <= GetActivePositionTolerance())
                {
                    break;
                }
            }
        }

        StepOrientationTowardTarget(jointCount);
        CommitJointWaypoint();
    }

    private void StepStrictTranslationWithLockedOrientation(int jointCount)
    {
        if (PositionError > GetActivePositionTolerance())
        {
            // 严格模式先只用 CCD 满足位置。与普通 DLS 加权折中不同，
            // 这里不会让位置误差稀释后续的完整姿态约束。
            for (int jointIndex = jointCount - 1; jointIndex >= 0; jointIndex--)
            {
                ApplyCcdStep(jointIndex);
            }
        }

        // 立刻以腕关节抵消底座/肩部的转角；目标为右手 Grip 起始时
        // 捕获的完整世界四元数，因此夹爪不会跟随底座绕 Z 轴自转。
        StepOrientationTowardTarget(jointCount);
    }

    private void StepTowardReadyPose()
    {
        int jointCount = Mathf.Min(6, jointController.JointCount);
        if (readyPoseJointDegrees == null || readyPoseJointDegrees.Length < jointCount)
        {
            CancelReadyPose();
            return;
        }

        BeginJointWaypoint(jointCount);
        float targetStep = Mathf.Max(0.0f, readyPoseMaxJointSpeedDegreesPerSecond)
            * Mathf.Max(Time.fixedDeltaTime, 0.0001f);
        bool allTargetSetpointsReached = true;
        bool allMeasuredJointsReached = true;

        for (int i = 0; i < jointCount; i++)
        {
            float readyTarget = readyPoseJointDegrees[i];
            float currentTarget = jointController.GetJointTargetDegrees(i);
            float nextTarget = Mathf.MoveTowards(currentTarget, readyTarget, targetStep);
            if (Mathf.Abs(nextTarget - currentTarget) > 0.0001f)
            {
                workingJointTargetsDegrees[i] = nextTarget;
                workingJointWaypointChanged = true;
            }

            if (Mathf.Abs(currentTarget - readyTarget) > 0.05f)
            {
                allTargetSetpointsReached = false;
            }

            if (Mathf.Abs(jointController.GetMeasuredJointDegrees(i) - readyTarget)
                > Mathf.Max(0.01f, readyPoseJointToleranceDegrees))
            {
                allMeasuredJointsReached = false;
            }
        }

        CommitJointWaypoint();
        if (!allTargetSetpointsReached || !allMeasuredJointsReached)
        {
            return;
        }

        isReadyPoseActive = false;
        jointController.SetReadyPoseDriveLeadProfile(false);
        ClearTrajectoryQueue();
        ResetJointDeltaSmoothing();
        SnapTargetToEndEffector();
    }

    private bool ShouldWaitForNextJointAssignment()
    {
        if (!useTimedJointAssignments)
        {
            return false;
        }

        float interval = Mathf.Max(0.0f, jointAssignmentIntervalSeconds);
        if (interval <= 0.0f)
        {
            return false;
        }

        if (Time.time + 0.0001f < nextJointAssignmentTime)
        {
            return true;
        }

        nextJointAssignmentTime = Time.time + interval;
        return false;
    }

    private Vector3 UpdateTrackingErrorsOnly()
    {
        Vector3 error = tcpTarget.position - GetControlPointPosition();
        PositionError = error.magnitude;
        RotationErrorDegrees = ShouldSolveTargetRotation()
            ? Quaternion.Angle(GetCurrentOrientationFrame(), GetTargetOrientationFrame())
            : 0.0f;
        return error;
    }

    private void UpdateTargetStationaryState(float deltaTime)
    {
        if (tcpTarget == null)
        {
            targetStationaryTime = 0.0f;
            hasLastTargetPose = false;
            return;
        }

        if (!hasLastTargetPose)
        {
            lastTargetPosition = tcpTarget.position;
            lastTargetRotation = tcpTarget.rotation;
            targetStationaryTime = 0.0f;
            hasLastTargetPose = true;
            return;
        }

        bool targetMoved =
            Vector3.Distance(lastTargetPosition, tcpTarget.position) > GetTargetStationaryPositionEpsilon()
            || Quaternion.Angle(lastTargetRotation, tcpTarget.rotation) > GetTargetStationaryRotationEpsilonDegrees();

        if (targetMoved)
        {
            lastTargetPosition = tcpTarget.position;
            lastTargetRotation = tcpTarget.rotation;
            targetStationaryTime = 0.0f;
            // 是否按着 Grip 不能代表 TCP 仍在运动：操作者可以按住 Grip 暂停。
            // 只有 TCP 目标确实变化，才允许重新开始 IK 跟踪并解除静止锁存。
            isSettledTargetHoldActive = false;
            return;
        }

        targetStationaryTime += Mathf.Max(0.0f, deltaTime);
    }

    private bool ShouldHoldSettledTarget()
    {
        return holdJointPoseWhenTargetSettled
            && targetStationaryTime >= targetStationaryHoldSeconds
            && PositionError <= GetActiveSettledPositionError()
            && RotationErrorDegrees <= GetActiveSettledRotationErrorDegrees();
    }

    /// <summary>
    /// XR 遥操作始终采用毫米级收敛阈值；快速移动由速度、加速度和关节步长
    /// 决定，而不是通过放宽最终 TCP 误差来换取表观速度。
    /// </summary>
    private float GetActivePositionTolerance()
    {
        if (graspAssist != null && graspAssist.IsAssistActive)
        {
            return Mathf.Min(positionTolerance, graspAssistPositionTolerance);
        }

        if (IsPrecisionAssemblyTrackingActive())
        {
            return Mathf.Min(positionTolerance, precisionAssemblyPositionTolerance);
        }

        return UsesPrecisionTolerance()
            ? Mathf.Min(positionTolerance, velocityReleasePositionTolerance)
            : positionTolerance;
    }

    private float GetActiveRotationToleranceDegrees()
    {
        if (graspAssist != null && graspAssist.IsAssistActive)
        {
            return Mathf.Min(rotationToleranceDegrees, graspAssistRotationToleranceDegrees);
        }

        if (IsPrecisionAssemblyTrackingActive())
        {
            return Mathf.Min(rotationToleranceDegrees, precisionAssemblyRotationToleranceDegrees);
        }

        return UsesPrecisionTolerance()
            ? Mathf.Min(rotationToleranceDegrees, velocityReleaseRotationToleranceDegrees)
            : rotationToleranceDegrees;
    }

    private float GetActiveSettledPositionError()
    {
        if (graspAssist != null && graspAssist.IsAssistActive)
        {
            return Mathf.Min(settledPositionError, graspAssistPositionTolerance);
        }

        if (IsPrecisionAssemblyTrackingActive())
        {
            return Mathf.Min(settledPositionError, precisionAssemblySettledPositionError);
        }

        return UsesPrecisionTolerance()
            ? Mathf.Min(settledPositionError, velocityReleasePositionTolerance)
            : settledPositionError;
    }

    private float GetActiveSettledRotationErrorDegrees()
    {
        if (graspAssist != null && graspAssist.IsAssistActive)
        {
            return Mathf.Min(settledRotationErrorDegrees, graspAssistRotationToleranceDegrees);
        }

        if (IsPrecisionAssemblyTrackingActive())
        {
            return Mathf.Min(settledRotationErrorDegrees, precisionAssemblySettledRotationErrorDegrees);
        }

        return UsesPrecisionTolerance()
            ? Mathf.Min(settledRotationErrorDegrees, velocityReleaseRotationToleranceDegrees)
            : settledRotationErrorDegrees;
    }

    private bool UsesPrecisionTolerance()
    {
        return safeReleaseSettlePending
            || IsPrecisionAssemblyTrackingActive();
    }

    private bool IsPrecisionAssemblyTrackingActive()
    {
        return enablePrecisionAssemblyTracking
            && velocityTeleop != null
            && velocityTeleop.IsCommandActive;
    }

    private float GetTargetStationaryPositionEpsilon()
    {
        return IsPrecisionAssemblyTrackingActive()
            ? precisionAssemblyTargetChangeEpsilonMeters
            : targetStationaryPositionEpsilon;
    }

    private float GetTargetStationaryRotationEpsilonDegrees()
    {
        return IsPrecisionAssemblyTrackingActive()
            ? precisionAssemblyTargetChangeEpsilonDegrees
            : targetStationaryRotationEpsilonDegrees;
    }

    private void HoldCurrentPoseAtSettledTarget()
    {
        if (isSettledTargetHoldActive)
        {
            return;
        }

        if (jointController != null)
        {
            // 以测得的关节角作为最终保持值。这里必须只执行一次；若在每个
            // FixedUpdate 重设 ArticulationDrive，物理引擎会表现成停止后的摆动。
            HoldCurrentJointsAndClearTrajectory();
        }

        ResetJointDeltaSmoothing();
        isSettledTargetHoldActive = true;
    }

    private void BeginJointWaypoint(int jointCount)
    {
        workingJointCount = Mathf.Min(jointCount, jointController.JointCount);
        EnsureWorkingJointBuffer(workingJointCount);
        for (int i = 0; i < workingJointCount; i++)
        {
            // DLS is evaluated from measured geometry, so each correction must start
            // from the target that was actually assigned to the Articulation Drive.
            workingJointTargetsDegrees[i] = jointController.GetDriveTargetDegrees(i);
        }

        workingJointWaypointChanged = false;
    }

    private void QueueJointDelta(int jointIndex, float deltaDegrees)
    {
        if (jointIndex < 0 || jointIndex >= workingJointCount)
        {
            return;
        }

        workingJointTargetsDegrees[jointIndex] += deltaDegrees;
        workingJointWaypointChanged = true;
    }

    private void CommitJointWaypoint()
    {
        if (!workingJointWaypointChanged || workingJointCount <= 0)
        {
            return;
        }

        if (trajectoryPlayer != null && trajectoryPlayer.enabled)
        {
            trajectoryPlayer.EnqueueWaypointDegrees(workingJointTargetsDegrees, workingJointCount);
            return;
        }

        jointController.SetJointTargetsDegrees(
            workingJointTargetsDegrees,
            workingJointCount,
            clampToDriveLimits,
            false);
    }

    private void HoldCurrentJointsAndClearTrajectory()
    {
        if (trajectoryPlayer != null && trajectoryPlayer.enabled)
        {
            trajectoryPlayer.HoldCurrentJointPose();
            return;
        }

        if (jointController != null)
        {
            jointController.HoldCurrentJointPose();
        }
    }

    private void ClearTrajectoryQueue()
    {
        if (trajectoryPlayer != null)
        {
            trajectoryPlayer.ClearQueue();
        }
    }

    private void EnsureWorkingJointBuffer(int minLength)
    {
        if (workingJointTargetsDegrees.Length >= minLength)
        {
            return;
        }

        workingJointTargetsDegrees = new float[minLength];
    }

    private void ApplyDampedLeastSquaresStep(int jointCount, Vector3 positionError)
    {
        if (!IsFinite(positionError))
        {
            RegisterIkFailure("Non-finite TCP position error");
            return;
        }

        bool allowRotationSolve = ShouldSolveTargetRotation();
        Vector3 rotationErrorRadians = allowRotationSolve
            ? GetRotationErrorRadians()
            : Vector3.zero;
        RotationErrorDegrees = rotationErrorRadians.magnitude * Mathf.Rad2Deg;

        bool solvePosition = PositionError > GetActivePositionTolerance();
        bool isTranslationOrientationHold = IsTranslationOrientationHoldActive();
        bool solveRotation = allowRotationSolve
            && (isTranslationOrientationHold
                || RotationErrorDegrees > GetActiveRotationToleranceDegrees());
        if (!solvePosition && !solveRotation)
        {
            return;
        }

        const int taskDimensions = 6;
        float positionWeight = solvePosition ? 1.0f : 0.0f;
        float rotationWeight = solveRotation
            ? (isTranslationOrientationHold
                ? Mathf.Max(dlsOrientationWeight, translationOrientationHoldWeight)
                : Mathf.Max(0.0f, dlsOrientationWeight))
            : 0.0f;
        float[,] jacobian = new float[taskDimensions, jointCount];
        Vector3 controlPoint = GetControlPointPosition();
        int firstWristIndex = Mathf.Max(0, jointCount - wristJointCount);

        for (int i = 0; i < jointCount; i++)
        {
            ArticulationBody joint = jointController.Joints[i];
            Vector3 axis = GetJointAxisWorld(joint, i);
            Vector3 linearVelocity = Vector3.Cross(axis, controlPoint - joint.transform.position);
            float jointRotationWeight = preferWristForOrientation && i < firstWristIndex
                ? Mathf.Clamp01(proximalOrientationWeight)
                : 1.0f;
            jacobian[0, i] = linearVelocity.x * positionWeight;
            jacobian[1, i] = linearVelocity.y * positionWeight;
            jacobian[2, i] = linearVelocity.z * positionWeight;
            jacobian[3, i] = axis.x * rotationWeight * jointRotationWeight;
            jacobian[4, i] = axis.y * rotationWeight * jointRotationWeight;
            jacobian[5, i] = axis.z * rotationWeight * jointRotationWeight;
        }

        float[] taskError =
        {
            positionError.x * positionWeight,
            positionError.y * positionWeight,
            positionError.z * positionWeight,
            rotationErrorRadians.x * rotationWeight,
            rotationErrorRadians.y * rotationWeight,
            rotationErrorRadians.z * rotationWeight
        };
        float[,] normalMatrix = new float[taskDimensions, taskDimensions];
        float dampingSquared = dlsDamping * dlsDamping;

        for (int row = 0; row < taskDimensions; row++)
        {
            for (int column = 0; column < taskDimensions; column++)
            {
                float value = 0.0f;
                for (int jointIndex = 0; jointIndex < jointCount; jointIndex++)
                {
                    value += jacobian[row, jointIndex] * jacobian[column, jointIndex];
                }

                normalMatrix[row, column] = value + (row == column ? dampingSquared : 0.0f);
            }
        }

        float[] taskVelocity = new float[taskDimensions];
        if (!SolveLinearSystem(normalMatrix, taskError, taskVelocity))
        {
            RegisterIkFailure("DLS linear solve failed");
            return;
        }

        if (!IsFinite(taskVelocity))
        {
            RegisterIkFailure("Non-finite DLS solution");
            return;
        }

        for (int i = 0; i < jointCount; i++)
        {
            if (IsJointCommandLeadLimited(i))
            {
                continue;
            }

            float jointDeltaRadians = 0.0f;
            for (int row = 0; row < taskDimensions; row++)
            {
                jointDeltaRadians += jacobian[row, i] * taskVelocity[row];
            }

            float rawDeltaDegrees = Mathf.Clamp(
                jointDeltaRadians * Mathf.Rad2Deg * GetEffectiveDlsGain(),
                -GetMaxJointStepDegrees(Time.fixedDeltaTime),
                GetMaxJointStepDegrees(Time.fixedDeltaTime));
            if (!IsFinite(rawDeltaDegrees))
            {
                RegisterIkFailure("Non-finite DLS joint delta");
                return;
            }
            float deltaDegrees = SmoothJointDelta(i, rawDeltaDegrees);
            if (Mathf.Abs(deltaDegrees) > minimumJointDeltaDegrees)
            {
                QueueJointDelta(i, deltaDegrees);
            }
        }

        RegisterIkSuccess();
    }

    private Vector3 GetRotationErrorRadians()
    {
        if (!followTargetRotation || endEffector == null || tcpTarget == null)
        {
            return Vector3.zero;
        }

        Quaternion rotationError = GetTargetOrientationFrame()
            * Quaternion.Inverse(GetCurrentOrientationFrame());
        rotationError.ToAngleAxis(out float errorAngle, out Vector3 errorAxis);
        if (errorAngle > 180.0f)
        {
            errorAngle -= 360.0f;
        }

        return errorAxis.sqrMagnitude > 0.000001f
            ? errorAxis.normalized * errorAngle * Mathf.Deg2Rad
            : Vector3.zero;
    }

    private bool ShouldSolveTargetRotation()
    {
        if (!followTargetRotation || endEffector == null || tcpTarget == null)
        {
            return false;
        }

        if (suppressRotationOnlyIkDuringPositionControl)
        {
            // The grasp assist deliberately supplies a fixed world-down tool
            // attitude. It must override manual-input suppression even when
            // the operator has released the controller grip.
            if (graspAssist != null && graspAssist.IsAssistActive)
            {
                return true;
            }

            // 右手单独平移时，必须保持在 Grip 起始时捕获的完整抓取姿态。
            // 不能用“左手是否为 Locked 输入模式”判断；本项目左手采用摇杆
            // 控制，若在此返回 false，IK 会为了平移而把夹爪朝上偏转。
            if (velocityTeleop != null && velocityTeleop.IsPositionOrientationLocked)
            {
                return true;
            }

            if (questController != null
                && questController.IsPositionClutched
                && !questController.IsRotationClutched)
            {
                return false;
            }

            if (velocityTeleop != null
                && velocityTeleop.IsPositionClutched
                && !velocityTeleop.IsRotationClutched)
            {
                return false;
            }
        }

        return true;
    }

    private bool IsTranslationOrientationHoldActive()
    {
        return velocityTeleop != null
            && velocityTeleop.IsPositionOrientationLocked;
    }

    private bool SolveLinearSystem(float[,] matrix, float[] rightHandSide, float[] solution)
    {
        const int dimension = 6;
        float[,] augmented = new float[dimension, dimension + 1];
        LastDlsMinimumPivot = float.PositiveInfinity;
        IsNearSingularity = false;
        for (int row = 0; row < dimension; row++)
        {
            for (int column = 0; column < dimension; column++)
            {
                if (!IsFinite(matrix[row, column]))
                {
                    return false;
                }

                augmented[row, column] = matrix[row, column];
            }

            if (!IsFinite(rightHandSide[row]))
            {
                return false;
            }

            augmented[row, dimension] = rightHandSide[row];
        }

        for (int pivotColumn = 0; pivotColumn < dimension; pivotColumn++)
        {
            int pivotRow = pivotColumn;
            for (int row = pivotColumn + 1; row < dimension; row++)
            {
                if (Mathf.Abs(augmented[row, pivotColumn]) > Mathf.Abs(augmented[pivotRow, pivotColumn]))
                {
                    pivotRow = row;
                }
            }

            float pivot = augmented[pivotRow, pivotColumn];
            float absolutePivot = Mathf.Abs(pivot);
            LastDlsMinimumPivot = Mathf.Min(LastDlsMinimumPivot, absolutePivot);
            IsNearSingularity |= absolutePivot < 0.0001f;
            if (!IsFinite(pivot) || absolutePivot < 0.000001f)
            {
                return false;
            }

            if (pivotRow != pivotColumn)
            {
                for (int column = pivotColumn; column <= dimension; column++)
                {
                    float temporary = augmented[pivotColumn, column];
                    augmented[pivotColumn, column] = augmented[pivotRow, column];
                    augmented[pivotRow, column] = temporary;
                }
            }

            for (int column = pivotColumn; column <= dimension; column++)
            {
                augmented[pivotColumn, column] /= pivot;
            }

            for (int row = 0; row < dimension; row++)
            {
                if (row == pivotColumn)
                {
                    continue;
                }

                float factor = augmented[row, pivotColumn];
                for (int column = pivotColumn; column <= dimension; column++)
                {
                    augmented[row, column] -= factor * augmented[pivotColumn, column];
                }
            }
        }

        for (int row = 0; row < dimension; row++)
        {
            solution[row] = augmented[row, dimension];
            if (!IsFinite(solution[row]))
            {
                return false;
            }
        }

        return true;
    }

    private void ApplyCcdStep(int jointIndex)
    {
        ArticulationBody joint = jointController.Joints[jointIndex];
        if (IsJointCommandLeadLimited(jointIndex))
        {
            return;
        }

        Vector3 axis = GetJointAxisWorld(joint, jointIndex);
        Vector3 toEndEffector = GetControlPointPosition() - joint.transform.position;
        Vector3 toTarget = tcpTarget.position - joint.transform.position;

        Vector3 endProjected = Vector3.ProjectOnPlane(toEndEffector, axis);
        Vector3 targetProjected = Vector3.ProjectOnPlane(toTarget, axis);
        if (endProjected.sqrMagnitude < 0.000001f || targetProjected.sqrMagnitude < 0.000001f)
        {
            return;
        }

        float signedAngle = Vector3.SignedAngle(endProjected, targetProjected, axis);
        float response = adaptivePositionSpeed
            ? Mathf.InverseLerp(GetActivePositionTolerance(), fullSpeedPositionError, PositionError)
            : 1.0f;
        float effectiveAngleBlend = Mathf.Lerp(angleBlend * 0.55f, angleBlend, response);
        float maximumStep = GetMaxJointStepDegrees(Time.fixedDeltaTime);
        float effectiveMaxStep = Mathf.Lerp(maximumStep * 0.55f, maximumStep, response);
        float rawDeltaDegrees = Mathf.Clamp(
            signedAngle * effectiveAngleBlend,
            -effectiveMaxStep,
            effectiveMaxStep);
        float deltaDegrees = SmoothJointDelta(jointIndex, rawDeltaDegrees);

        if (Mathf.Abs(deltaDegrees) > minimumJointDeltaDegrees)
        {
            QueueJointDelta(jointIndex, deltaDegrees);
        }
    }

    private Vector3 GetJointAxisWorld(ArticulationBody joint, int index)
    {
        Vector3 localAxis = GetConfiguredLocalAxis(index);
        Vector3 jointFrameAxis = joint.anchorRotation * localAxis.normalized;
        return joint.transform.TransformDirection(jointFrameAxis).normalized;
    }

    private void StepOrientationTowardTarget(int jointCount)
    {
        if (!ShouldSolveTargetRotation())
        {
            return;
        }

        Quaternion currentOrientation = GetCurrentOrientationFrame();
        Quaternion targetOrientation = GetTargetOrientationFrame();
        RotationErrorDegrees = Quaternion.Angle(currentOrientation, targetOrientation);
        if (RotationErrorDegrees <= GetActiveRotationToleranceDegrees())
        {
            return;
        }

        Quaternion rotationError = targetOrientation * Quaternion.Inverse(currentOrientation);
        rotationError.ToAngleAxis(out float errorAngle, out Vector3 errorAxis);
        if (errorAngle > 180.0f)
        {
            errorAngle -= 360.0f;
        }

        Vector3 errorVectorDegrees = errorAxis.normalized * errorAngle;
        int firstWristIndex = Mathf.Max(0, jointCount - wristJointCount);
        for (int i = jointCount - 1; i >= firstWristIndex; i--)
        {
            ArticulationBody joint = jointController.Joints[i];
            if (IsJointCommandLeadLimited(i))
            {
                continue;
            }

            float axisError = Vector3.Dot(errorVectorDegrees, GetJointAxisWorld(joint, i));
            float rawDeltaDegrees = Mathf.Clamp(
                axisError * rotationBlend,
                -GetMaxWristStepDegrees(Time.fixedDeltaTime),
                GetMaxWristStepDegrees(Time.fixedDeltaTime));
            float deltaDegrees = SmoothJointDelta(i, rawDeltaDegrees);

            if (Mathf.Abs(deltaDegrees) > minimumJointDeltaDegrees)
            {
                QueueJointDelta(i, deltaDegrees);
            }
        }
    }

    private bool IsJointCommandLeadLimited(int jointIndex)
    {
        float maximumLead = Mathf.Max(0.0f, maximumCommandLeadDegrees);
        if (maximumLead <= 0.0f || jointController == null)
        {
            return false;
        }

        bool isLimited = Mathf.Abs(jointController.GetJointTargetDegrees(jointIndex)
                - jointController.GetAppliedJointTargetDegrees(jointIndex)) > maximumLead;
        WasIkCommandLeadLimited |= isLimited;
        return isLimited;
    }

    private float SmoothJointDelta(int jointIndex, float rawDeltaDegrees)
    {
        EnsureJointDeltaSmoothingBuffer(jointIndex + 1);
        if (IsStationaryDampingActive())
        {
            // 目标停止后绝不能沿用上一帧的关节修正，否则会形成“惯性尾巴”。
            // 静止阶段仅使用当帧的、已降低增益后的校正量。
            smoothedJointDeltaDegrees[jointIndex] = rawDeltaDegrees;
            return rawDeltaDegrees;
        }

        float smoothing = Mathf.Clamp01(jointDeltaSmoothing);
        float smoothed = Mathf.Lerp(rawDeltaDegrees, smoothedJointDeltaDegrees[jointIndex], smoothing);
        smoothedJointDeltaDegrees[jointIndex] = smoothed;
        return smoothed;
    }

    private float GetMaxJointStepDegrees(float deltaTime)
    {
        float speed = maxJointSpeedDegreesPerSecond > 0.0f
            ? maxJointSpeedDegreesPerSecond
            : Mathf.Max(0.0f, maxJointStepDegrees) * 90.0f;
        return speed * Mathf.Max(0.0001f, deltaTime);
    }

    private float GetMaxWristStepDegrees(float deltaTime)
    {
        float speed = maxWristSpeedDegreesPerSecond > 0.0f
            ? maxWristSpeedDegreesPerSecond
            : Mathf.Max(0.0f, maxWristStepDegrees) * 90.0f;
        return speed * Mathf.Max(0.0001f, deltaTime);
    }

    private void RegisterIkSuccess()
    {
        ConsecutiveIkFailureCount = 0;
        LastIkFailureReason = string.Empty;
    }

    private void RegisterIkFailure(string reason)
    {
        IkFailureCount++;
        ConsecutiveIkFailureCount++;
        LastIkFailureReason = reason;
        if (ConsecutiveIkFailureCount >= 3)
        {
            HoldCurrentJointsAndClearTrajectory();
            ResetJointDeltaSmoothing();
        }
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float[] values)
    {
        if (values == null)
        {
            return false;
        }

        for (int i = 0; i < values.Length; i++)
        {
            if (!IsFinite(values[i]))
            {
                return false;
            }
        }

        return true;
    }

    private float GetEffectiveDlsGain()
    {
        return IsStationaryDampingActive()
            ? dlsGain * stationaryDlsGainMultiplier
            : dlsGain;
    }

    private bool IsStationaryDampingActive()
    {
        // 该实验性补偿会改变 IK 的最终收敛路径。默认关闭并使用稳定基线的
        // 目标死区/保持逻辑，避免停止前出现长时间的低增益追赶。
        return enableStationaryDlsDamping
            && targetStationaryTime >= stationaryDampingStartSeconds;
    }

    private void EnsureJointDeltaSmoothingBuffer(int minLength)
    {
        if (smoothedJointDeltaDegrees.Length >= minLength)
        {
            return;
        }

        float[] resized = new float[minLength];
        for (int i = 0; i < smoothedJointDeltaDegrees.Length; i++)
        {
            resized[i] = smoothedJointDeltaDegrees[i];
        }

        smoothedJointDeltaDegrees = resized;
    }

    private void ResetJointDeltaSmoothing()
    {
        for (int i = 0; i < smoothedJointDeltaDegrees.Length; i++)
        {
            smoothedJointDeltaDegrees[i] = 0.0f;
        }
    }

    private Vector3 GetConfiguredLocalAxis(int index)
    {
        if (jointLocalAxes != null
            && index >= 0
            && index < jointLocalAxes.Length
            && jointLocalAxes[index].sqrMagnitude > 0.0001f)
        {
            return jointLocalAxes[index];
        }

        return defaultJointLocalAxis.sqrMagnitude > 0.0001f ? defaultJointLocalAxis : Vector3.right;
    }

    private Vector3 GetControlPointPosition()
    {
        if (useGripperPadCenter && leftGripperPad != null && rightGripperPad != null)
        {
            Vector3 leftCenter = GetPadCenter(leftGripperPad);
            Vector3 rightCenter = GetPadCenter(rightGripperPad);
            return (leftCenter + rightCenter) * 0.5f;
        }

        return endEffector != null ? endEffector.position : transform.position;
    }

    /// <summary>
    /// 返回用于 IK 姿态误差的当前任务坐标系。
    /// 启用时该坐标系的前向轴是“夹爪基座 -> 两指中心”的抓取方向，
    /// 上向轴由两指连线确定，因此会同时约束朝下方向和两指的朝向。
    /// </summary>
    private Quaternion GetCurrentOrientationFrame()
    {
        if (usePhysicalGraspFrameForOrientation)
        {
            return GetActualGraspRotation();
        }

        return endEffector != null ? endEffector.rotation : Quaternion.identity;
    }

    /// <summary>
    /// 将 TcpTarget 中保存的 tool0 旋转转换为相同语义下的抓取坐标系旋转。
    /// 这样位置和姿态都以两指中心任务帧为准，避免混用 TCP 位置与 tool0 轴。
    /// </summary>
    private Quaternion GetTargetOrientationFrame()
    {
        if (tcpTarget == null)
        {
            return Quaternion.identity;
        }

        if (!usePhysicalGraspFrameForOrientation)
        {
            return tcpTarget.rotation;
        }

        // 首次调用时缓存刚性安装变换 tool0 -> grasp frame。
        // 该变换由真实模型几何得出，不需要在 Inspector 中猜测或手填轴向。
        GetActualGraspRotation();
        return hasToolToGraspRotation
            ? tcpTarget.rotation * toolToGraspRotation
            : tcpTarget.rotation;
    }

    /// <summary>
    /// Builds the task frame from the physical gripper: forward is Robotiq
    /// base -> two-pad midpoint and up is perpendicular to the jaw axis.
    /// This avoids assuming a particular URDF/Unity tool-axis conversion.
    /// </summary>
    private Quaternion GetActualGraspRotation()
    {
        ResolveReferences();
        if (!TryGetPhysicalGraspRotation(out Quaternion graspRotation))
        {
            return endEffector != null ? endEffector.rotation : Quaternion.identity;
        }

        EnsureToolToGraspRotation(graspRotation);
        return graspRotation;
    }

    public Quaternion GetToolRotationForGraspRotation(Quaternion desiredGraspRotation)
    {
        Quaternion currentGraspRotation = GetActualGraspRotation();
        EnsureToolToGraspRotation(currentGraspRotation);
        return hasToolToGraspRotation
            ? desiredGraspRotation * Quaternion.Inverse(toolToGraspRotation)
            : desiredGraspRotation;
    }

    /// <summary>
    /// 将一个命令 tool0 姿态转换为对应的两指中心抓取坐标系姿态。
    /// 遥操作层用它从当前 TCP 命令中提取夹爪偏航参考，避免把 URDF 的 tool0
    /// 局部轴误当成物理夹爪的轴向。
    /// </summary>
    public Quaternion GetGraspRotationForToolRotation(Quaternion toolRotation)
    {
        Quaternion currentGraspRotation = GetActualGraspRotation();
        EnsureToolToGraspRotation(currentGraspRotation);
        return hasToolToGraspRotation
            ? toolRotation * toolToGraspRotation
            : toolRotation;
    }

    public Quaternion GetToolRotationForGraspApproach(
        Vector3 desiredApproachWorld,
        Vector3 yawReferenceWorld)
    {
        Vector3 forward = desiredApproachWorld.sqrMagnitude > 0.0001f
            ? desiredApproachWorld.normalized
            : Vector3.down;
        Vector3 up = Vector3.ProjectOnPlane(yawReferenceWorld, forward);
        if (up.sqrMagnitude < 0.0001f)
        {
            up = Vector3.ProjectOnPlane(Vector3.right, forward);
        }

        return GetToolRotationForGraspRotation(Quaternion.LookRotation(forward, up.normalized));
    }

    private bool TryGetPhysicalGraspRotation(out Quaternion graspRotation)
    {
        graspRotation = Quaternion.identity;
        if (!useGripperPadCenter
            || leftGripperPad == null
            || rightGripperPad == null
            || gripperBase == null)
        {
            return false;
        }

        Vector3 approach = GetControlPointPosition() - gripperBase.position;
        Vector3 jawAxis = rightGripperPad.position - leftGripperPad.position;
        if (approach.sqrMagnitude < 0.000001f || jawAxis.sqrMagnitude < 0.000001f)
        {
            return false;
        }

        Vector3 up = Vector3.Cross(jawAxis.normalized, approach.normalized);
        if (up.sqrMagnitude < 0.000001f)
        {
            return false;
        }

        graspRotation = Quaternion.LookRotation(approach.normalized, up.normalized);
        return true;
    }

    private void EnsureToolToGraspRotation(Quaternion graspRotation)
    {
        if (hasToolToGraspRotation || endEffector == null)
        {
            return;
        }

        toolToGraspRotation = Quaternion.Inverse(endEffector.rotation) * graspRotation;
        hasToolToGraspRotation = true;
    }

    private Vector3 GetPadCenter(Transform pad)
    {
        if (!usePadGeometryCenter)
        {
            return pad.position;
        }

        Renderer[] renderers = pad.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0)
        {
            return pad.position;
        }

        Bounds bounds = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++)
        {
            bounds.Encapsulate(renderers[i].bounds);
        }

        return bounds.center;
    }

    private Transform FindEndEffector(Transform root)
    {
        foreach (string hint in endEffectorNameHints)
        {
            Transform match = FindByNameHint(root, hint);
            if (match != null)
            {
                return match;
            }
        }

        ArticulationBody[] bodies = root.GetComponentsInChildren<ArticulationBody>(true);
        if (bodies.Length > 0)
        {
            return bodies[bodies.Length - 1].transform;
        }

        return null;
    }

    private Transform FindByNameHint(Transform root, string hint)
    {
        if (string.IsNullOrEmpty(hint))
        {
            return null;
        }

        string lowerHint = hint.ToLowerInvariant();
        Transform[] transforms = root.GetComponentsInChildren<Transform>(true);
        foreach (Transform child in transforms)
        {
            if (child.name.ToLowerInvariant().Contains(lowerHint))
            {
                return child;
            }
        }

        return null;
    }

    private void LogReadyState()
    {
        if (!logStatus || loggedReady || !HasRequiredReferences())
        {
            return;
        }

        loggedReady = true;
        Debug.Log("UR5 TCP target follower ready. Target=" + tcpTarget.name + ", EndEffector=" + endEffector.name);
    }

    private void LogMissingReferences()
    {
        if (!logStatus || loggedMissingReferences)
        {
            return;
        }

        loggedMissingReferences = true;
        Debug.LogWarning("UR5 TCP target follower is waiting for robot joints, TcpTarget, or end effector.");
    }
}

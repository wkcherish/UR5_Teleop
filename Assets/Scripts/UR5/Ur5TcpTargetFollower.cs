using UnityEngine;

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
    public bool usePadGeometryCenter = true;

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
    public float maxJointStepDegrees = 1.45f;
    [Tooltip("Ignores microscopic IK deltas that usually come from tracking noise rather than intentional motion.")]
    public float minimumJointDeltaDegrees = 0.015f;
    public bool adaptivePositionSpeed = true;
    public float fullSpeedPositionError = 0.12f;
    [Tooltip("Prevents CCD from queueing large target jumps before the drive has applied the prior correction.")]
    public float maximumCommandLeadDegrees = 6.00f;
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
    [Tooltip("0 = no smoothing, 1 = keep the previous IK delta. Use small values to reduce twitching.")]
    [Range(0.0f, 0.95f)] public float jointDeltaSmoothing = 0.45f;

    [Header("End Effector Orientation")]
    public bool followTargetRotation = true;
    [Range(1, 3)] public int wristJointCount = 3;
    public float rotationToleranceDegrees = 1.20f;
    public float rotationBlend = 0.70f;
    public float maxWristStepDegrees = 2.00f;
    [Tooltip("When only the right controller is translating, fully pause orientation IK so wrist joints do not twitch while chasing pose noise.")]
    public bool suppressRotationOnlyIkDuringPositionControl = true;

    [Header("Quest Idle Hold")]
    public Quest3TcpTargetController questController;
    public Ur5CartesianVelocityTeleopController velocityTeleop;
    [Tooltip("Keeps IK active while the scripted PreGrasp -> Grasp -> Lift sequence is moving the TCP target.")]
    public Ur5GraspAssistController graspAssist;
    public bool pauseIkWhenQuestControllerIdle = true;
    public bool pauseIkWhenVelocityTeleopIdle = true;
    public bool snapTargetToActualPoseWhenQuestReleased = true;
    public bool holdJointPoseWhenQuestReleased = true;

    [Header("Settled Target Hold")]
    public bool holdJointPoseWhenTargetSettled = true;
    public float targetStationaryHoldSeconds = 0.12f;
    public float targetStationaryPositionEpsilon = 0.0015f;
    public float targetStationaryRotationEpsilonDegrees = 0.30f;
    public float settledPositionError = 0.010f;
    public float settledRotationErrorDegrees = 1.50f;

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

    private bool loggedReady;
    private bool loggedMissingReferences;
    private bool wasControllerCommandActive;
    private bool isControllerIdleHoldActive;
    private bool hasLastTargetPose;
    private Vector3 lastTargetPosition;
    private Quaternion lastTargetRotation = Quaternion.identity;
    private float targetStationaryTime;
    private float nextJointAssignmentTime;
    private float[] smoothedJointDeltaDegrees = new float[0];
    private float[] workingJointTargetsDegrees = new float[0];
    private int workingJointCount;
    private bool workingJointWaypointChanged;

    public float PositionError { get; private set; }
    public float RotationErrorDegrees { get; private set; }
    public Vector3 ControlPointPosition => GetControlPointPosition();

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
        dlsGain = Mathf.Max(0.0f, dlsGain);
        targetStationaryHoldSeconds = Mathf.Max(0.0f, targetStationaryHoldSeconds);
        targetStationaryPositionEpsilon = Mathf.Max(0.0f, targetStationaryPositionEpsilon);
        targetStationaryRotationEpsilonDegrees = Mathf.Max(0.0f, targetStationaryRotationEpsilonDegrees);
        settledPositionError = Mathf.Max(0.0f, settledPositionError);
        settledRotationErrorDegrees = Mathf.Max(0.0f, settledRotationErrorDegrees);
        jointAssignmentIntervalSeconds = Mathf.Max(0.0f, jointAssignmentIntervalSeconds);
    }

    private void OnDisable()
    {
        ClearTrajectoryQueue();
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
            return !velocityTeleop.IsCommandActive;
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

        if (snapTargetToActualPoseWhenQuestReleased && tcpTarget != null)
        {
            tcpTarget.position = GetControlPointPosition();
            if (followTargetRotation && endEffector != null)
            {
                tcpTarget.rotation = endEffector.rotation;
            }

            TcpTargetWorkspaceLimiter workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
            if (workspaceLimiter != null)
            {
                workspaceLimiter.PreserveCurrentTargetPose();
            }
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
            tcpTarget.rotation = endEffector.rotation;
        }
    }

    private void StepTowardTarget()
    {
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
        if (solverMode == IkSolverMode.DampedLeastSquares)
        {
            ApplyDampedLeastSquaresStep(jointCount, error);
            CommitJointWaypoint();
            return;
        }

        if (PositionError > positionTolerance)
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
                if (PositionError <= positionTolerance)
                {
                    break;
                }
            }
        }

        StepOrientationTowardTarget(jointCount);
        CommitJointWaypoint();
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
            ? Quaternion.Angle(endEffector.rotation, tcpTarget.rotation)
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
            Vector3.Distance(lastTargetPosition, tcpTarget.position) > targetStationaryPositionEpsilon
            || Quaternion.Angle(lastTargetRotation, tcpTarget.rotation) > targetStationaryRotationEpsilonDegrees;

        if (targetMoved)
        {
            lastTargetPosition = tcpTarget.position;
            lastTargetRotation = tcpTarget.rotation;
            targetStationaryTime = 0.0f;
            return;
        }

        targetStationaryTime += Mathf.Max(0.0f, deltaTime);
    }

    private bool ShouldHoldSettledTarget()
    {
        return holdJointPoseWhenTargetSettled
            && targetStationaryTime >= targetStationaryHoldSeconds
            && PositionError <= settledPositionError
            && RotationErrorDegrees <= settledRotationErrorDegrees;
    }

    private void HoldCurrentPoseAtSettledTarget()
    {
        if (jointController != null)
        {
            HoldCurrentJointsAndClearTrajectory();
        }

        ResetJointDeltaSmoothing();
    }

    private void BeginJointWaypoint(int jointCount)
    {
        workingJointCount = Mathf.Min(jointCount, jointController.JointCount);
        EnsureWorkingJointBuffer(workingJointCount);
        for (int i = 0; i < workingJointCount; i++)
        {
            workingJointTargetsDegrees[i] = jointController.GetJointTargetDegrees(i);
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
            true);
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
        bool allowRotationSolve = ShouldSolveTargetRotation();
        Vector3 rotationErrorRadians = allowRotationSolve
            ? GetRotationErrorRadians()
            : Vector3.zero;
        RotationErrorDegrees = rotationErrorRadians.magnitude * Mathf.Rad2Deg;

        bool solvePosition = PositionError > positionTolerance;
        bool solveRotation = allowRotationSolve && RotationErrorDegrees > rotationToleranceDegrees;
        if (!solvePosition && !solveRotation)
        {
            return;
        }

        const int taskDimensions = 6;
        float positionWeight = solvePosition ? 1.0f : 0.0f;
        float rotationWeight = solveRotation ? Mathf.Max(0.0f, dlsOrientationWeight) : 0.0f;
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
            return;
        }

        for (int i = 0; i < jointCount; i++)
        {
            if (Mathf.Abs(jointController.GetJointTargetDegrees(i)
                    - jointController.GetAppliedJointTargetDegrees(i)) > maximumCommandLeadDegrees)
            {
                continue;
            }

            float jointDeltaRadians = 0.0f;
            for (int row = 0; row < taskDimensions; row++)
            {
                jointDeltaRadians += jacobian[row, i] * taskVelocity[row];
            }

            float rawDeltaDegrees = Mathf.Clamp(
                jointDeltaRadians * Mathf.Rad2Deg * dlsGain,
                -maxJointStepDegrees,
                maxJointStepDegrees);
            float deltaDegrees = SmoothJointDelta(i, rawDeltaDegrees);
            if (Mathf.Abs(deltaDegrees) > minimumJointDeltaDegrees)
            {
                QueueJointDelta(i, deltaDegrees);
            }
        }
    }

    private Vector3 GetRotationErrorRadians()
    {
        if (!followTargetRotation || endEffector == null || tcpTarget == null)
        {
            return Vector3.zero;
        }

        Quaternion rotationError = tcpTarget.rotation * Quaternion.Inverse(endEffector.rotation);
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

            // In locked-orientation teleoperation, the controller deliberately
            // supplies no angular command, but the IK still must preserve the
            // existing TCP attitude while the user translates it.
            if (velocityTeleop != null && velocityTeleop.IsOrientationLocked)
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

    private bool SolveLinearSystem(float[,] matrix, float[] rightHandSide, float[] solution)
    {
        const int dimension = 6;
        float[,] augmented = new float[dimension, dimension + 1];
        for (int row = 0; row < dimension; row++)
        {
            for (int column = 0; column < dimension; column++)
            {
                augmented[row, column] = matrix[row, column];
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
            if (Mathf.Abs(pivot) < 0.000001f)
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
        }

        return true;
    }

    private void ApplyCcdStep(int jointIndex)
    {
        ArticulationBody joint = jointController.Joints[jointIndex];
        if (Mathf.Abs(jointController.GetJointTargetDegrees(jointIndex)
                - jointController.GetAppliedJointTargetDegrees(jointIndex)) > maximumCommandLeadDegrees)
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
            ? Mathf.InverseLerp(positionTolerance, fullSpeedPositionError, PositionError)
            : 1.0f;
        float effectiveAngleBlend = Mathf.Lerp(angleBlend * 0.55f, angleBlend, response);
        float effectiveMaxStep = Mathf.Lerp(maxJointStepDegrees * 0.55f, maxJointStepDegrees, response);
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

        RotationErrorDegrees = Quaternion.Angle(endEffector.rotation, tcpTarget.rotation);
        if (RotationErrorDegrees <= rotationToleranceDegrees)
        {
            return;
        }

        Quaternion rotationError = tcpTarget.rotation * Quaternion.Inverse(endEffector.rotation);
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
            if (Mathf.Abs(jointController.GetJointTargetDegrees(i)
                    - jointController.GetAppliedJointTargetDegrees(i)) > maximumCommandLeadDegrees)
            {
                continue;
            }

            float axisError = Vector3.Dot(errorVectorDegrees, GetJointAxisWorld(joint, i));
            float rawDeltaDegrees = Mathf.Clamp(
                axisError * rotationBlend,
                -maxWristStepDegrees,
                maxWristStepDegrees);
            float deltaDegrees = SmoothJointDelta(i, rawDeltaDegrees);

            if (Mathf.Abs(deltaDegrees) > minimumJointDeltaDegrees)
            {
                QueueJointDelta(i, deltaDegrees);
            }
        }
    }

    private float SmoothJointDelta(int jointIndex, float rawDeltaDegrees)
    {
        EnsureJointDeltaSmoothingBuffer(jointIndex + 1);
        float smoothing = Mathf.Clamp01(jointDeltaSmoothing);
        float smoothed = Mathf.Lerp(rawDeltaDegrees, smoothedJointDeltaDegrees[jointIndex], smoothing);
        smoothedJointDeltaDegrees[jointIndex] = smoothed;
        return smoothed;
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

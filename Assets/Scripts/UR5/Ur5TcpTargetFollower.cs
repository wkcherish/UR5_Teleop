using UnityEngine;

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

    [Header("Gripper TCP")]
    [Tooltip("Use the midpoint between the two Robotiq pads as the task-space TCP.")]
    public bool useGripperPadCenter = true;
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
    public float positionTolerance = 0.015f;
    [Tooltip("Keep this at 1. Articulation poses update after FixedUpdate, so repeated CCD passes use stale geometry and cause oscillation.")]
    public int solverIterationsPerFixedUpdate = 1;
    public float angleBlend = 0.34f;
    public float maxJointStepDegrees = 1.25f;
    public bool adaptivePositionSpeed = true;
    public float fullSpeedPositionError = 0.12f;
    [Tooltip("Prevents CCD from queueing large target jumps before the drive has applied the prior correction.")]
    public float maximumCommandLeadDegrees = 1.2f;
    public float maxReachError = 1.5f;
    public bool clampToDriveLimits = true;

    [Header("Damped Least Squares IK")]
    [Tooltip("Higher values trade responsiveness for stability near singular configurations.")]
    public float dlsDamping = 0.10f;
    [Tooltip("Treats one radian of orientation error as this many meters of task error.")]
    public float dlsOrientationWeight = 0.34f;
    public float dlsGain = 0.58f;

    [Header("End Effector Orientation")]
    public bool followTargetRotation = true;
    [Range(1, 3)] public int wristJointCount = 3;
    public float rotationToleranceDegrees = 0.85f;
    public float rotationBlend = 0.34f;
    public float maxWristStepDegrees = 1.20f;

    [Header("Quest Idle Hold")]
    public Quest3TcpTargetController questController;
    public bool pauseIkWhenQuestControllerIdle = true;
    public bool snapTargetToActualPoseWhenQuestReleased = true;
    public bool holdJointPoseWhenQuestReleased = true;

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
    private bool wasQuestClutched;
    private bool isQuestIdleHoldActive;

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
        if (ShouldPauseForQuestIdle())
        {
            EnterQuestIdleHold();
            return;
        }

        isQuestIdleHoldActive = false;
        wasQuestClutched = questController != null && questController.IsClutched;
        StepTowardTarget();
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

    private bool ShouldPauseForQuestIdle()
    {
        return pauseIkWhenQuestControllerIdle
            && questController != null
            && questController.IsDeviceValid
            && !questController.IsClutched;
    }

    private void EnterQuestIdleHold()
    {
        if (isQuestIdleHoldActive && !wasQuestClutched)
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
            jointController.HoldCurrentJointPose();
        }

        PositionError = 0.0f;
        RotationErrorDegrees = 0.0f;
        wasQuestClutched = false;
        isQuestIdleHoldActive = true;
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
        Vector3 error = tcpTarget.position - GetControlPointPosition();
        PositionError = error.magnitude;

        if (PositionError > maxReachError)
        {
            return;
        }

        int jointCount = Mathf.Min(6, jointController.JointCount);
        if (solverMode == IkSolverMode.DampedLeastSquares)
        {
            ApplyDampedLeastSquaresStep(jointCount, error);
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
    }

    private void ApplyDampedLeastSquaresStep(int jointCount, Vector3 positionError)
    {
        Vector3 rotationErrorRadians = GetRotationErrorRadians();
        RotationErrorDegrees = rotationErrorRadians.magnitude * Mathf.Rad2Deg;

        bool solvePosition = PositionError > positionTolerance;
        bool solveRotation = followTargetRotation && RotationErrorDegrees > rotationToleranceDegrees;
        if (!solvePosition && !solveRotation)
        {
            return;
        }

        const int taskDimensions = 6;
        float positionWeight = solvePosition ? 1.0f : 0.0f;
        float rotationWeight = solveRotation ? Mathf.Max(0.0f, dlsOrientationWeight) : 0.0f;
        float[,] jacobian = new float[taskDimensions, jointCount];
        Vector3 controlPoint = GetControlPointPosition();

        for (int i = 0; i < jointCount; i++)
        {
            ArticulationBody joint = jointController.Joints[i];
            Vector3 axis = GetJointAxisWorld(joint, i);
            Vector3 linearVelocity = Vector3.Cross(axis, controlPoint - joint.transform.position);
            jacobian[0, i] = linearVelocity.x * positionWeight;
            jacobian[1, i] = linearVelocity.y * positionWeight;
            jacobian[2, i] = linearVelocity.z * positionWeight;
            jacobian[3, i] = axis.x * rotationWeight;
            jacobian[4, i] = axis.y * rotationWeight;
            jacobian[5, i] = axis.z * rotationWeight;
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

            float deltaDegrees = Mathf.Clamp(
                jointDeltaRadians * Mathf.Rad2Deg * dlsGain,
                -maxJointStepDegrees,
                maxJointStepDegrees);
            if (Mathf.Abs(deltaDegrees) > 0.0001f)
            {
                jointController.AddJointTargetDegrees(i, deltaDegrees, clampToDriveLimits);
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
        float deltaDegrees = Mathf.Clamp(
            signedAngle * effectiveAngleBlend,
            -effectiveMaxStep,
            effectiveMaxStep);

        if (Mathf.Abs(deltaDegrees) > 0.0001f)
        {
            jointController.AddJointTargetDegrees(jointIndex, deltaDegrees, clampToDriveLimits);
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
        if (!followTargetRotation || endEffector == null || tcpTarget == null)
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
            float deltaDegrees = Mathf.Clamp(
                axisError * rotationBlend,
                -maxWristStepDegrees,
                maxWristStepDegrees);

            if (Mathf.Abs(deltaDegrees) > 0.0001f)
            {
                jointController.AddJointTargetDegrees(i, deltaDegrees, clampToDriveLimits);
            }
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

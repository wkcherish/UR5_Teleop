using UnityEngine;
using UnityEngine.XR;

[DefaultExecutionOrder(-60)]
public class Ur5GraspAssistController : MonoBehaviour
{
    private enum GraspState
    {
        Idle,
        MoveToPreGrasp,
        MoveToGrasp,
        SettleAtGrasp,
        CloseGripper,
        Lift,
        Complete
    }

    [Header("References")]
    public Transform robotRoot;
    public Transform tcpTarget;
    public Ur5TcpTargetFollower tcpFollower;
    public Ur5CartesianVelocityTeleopController velocityTeleop;
    public TcpTargetWorkspaceLimiter workspaceLimiter;
    public TcpTargetWriteMonitor targetWriteMonitor;
    public Quest3RobotiqGripperController gripperController;
    public Transform graspTarget;

    [Header("Input")]
    public XRNode controllerNode = XRNode.RightHand;
    public bool startWithSecondaryButton = true;
    [Tooltip("Also accept the primary button so A/B mapping differences on Quest builds cannot block the grasp sequence.")]
    public bool startWithPrimaryButton = true;
    public KeyCode keyboardStartKey = KeyCode.G;
    public KeyCode keyboardAbortKey = KeyCode.X;

    [Header("Target Selection")]
    public bool autoSelectNearestTarget = true;
    public float targetSearchRadius = 0.35f;
    public float maxAutoTargetSize = 0.35f;
    public LayerMask graspableLayers = ~0;
    public string requiredTargetTag;

    [Header("Pick Sequence")]
    public Vector3 approachDirectionWorld = Vector3.up;
    public float preGraspHeight = 0.12f;
    [Tooltip("Place the two-pad TCP at the selected object's bounds center for a parallel-jaw grasp. Disable only when a known grasp point is above the center.")]
    public bool alignPadCenterToObjectCenter = true;
    [Tooltip("Calibration offset from the selected object's center along the retreat/approach direction. Positive values move the pad center above the object center.")]
    public float padCenterOffsetAlongApproach = 0.0f;
    public float graspClearance = 0.015f;
    public float liftHeight = 0.16f;
    public float assistMoveSpeed = 0.16f;
    [Tooltip("Final descent speed only. Pre-grasp and lift continue to use assistMoveSpeed.")]
    public float finalApproachSpeed = 0.035f;
    [Tooltip("Maximum Cartesian acceleration for each planned segment. This removes abrupt starts/stops without changing the straight-line waypoint path.")]
    public float assistMoveAcceleration = 0.45f;
    public float assistRotationSpeedDegreesPerSecond = 180.0f;
    public float waypointTolerance = 0.012f;
    [Tooltip("Actual two-pad TCP tolerance for the pre-grasp and lift stages.")]
    public float transitActualPositionTolerance = 0.008f;
    [Tooltip("Actual two-pad TCP tolerance required before closing the gripper.")]
    public float finalGraspActualPositionTolerance = 0.0015f;
    public float actualPositionTolerance = 0.025f;
    [Tooltip("Actual grasp-frame orientation tolerance required before closing the gripper.")]
    public float finalGraspActualRotationToleranceDegrees = 0.35f;
    [Tooltip("The final target must remain within the precision tolerances for this period before the gripper closes.")]
    public float finalGraspSettleSeconds = 0.12f;
    public float actualRotationToleranceDegrees = 8.0f;
    public float closeGripperSeconds = 0.45f;
    public bool openGripperOnStart = true;
    public bool preserveCurrentTcpRotation = false;
    [Tooltip("For the UR5 tool0 convention, align target.forward against the approach direction. This produces a true world-down grasp instead of relying on a guessed Euler angle.")]
    public bool alignToolForwardAgainstApproachDirection = true;
    [Tooltip("Used only to choose the yaw about the vertical grasp axis. Leave zero to use the robot-root forward direction.")]
    public Vector3 graspYawReferenceWorld = Vector3.zero;
    [Tooltip("Fallback only when automatic tool-axis alignment is disabled.")]
    public Vector3 fixedTcpRotationEuler = new Vector3(180.0f, 0.0f, 0.0f);

    [Header("Safety")]
    public bool pauseVelocityPreviewDuringAssist = true;
    public bool ignoreManualGripperTriggerDuringAssist = true;
    public bool preventRobotBodyTargetCollision = true;
    public float robotBodyClearanceRadius = 0.075f;
    public LayerMask robotBodyLayers = ~0;
    public string[] allowedRobotNearTcpNameHints =
    {
        "tool",
        "tcp",
        "ee_link",
        "wrist_3",
        "gripper",
        "finger",
        "pad",
        "driver",
        "spring",
        "follower"
    };

    [Header("Debug")]
    public bool drawDebug = true;
    public bool logStatus = true;

    private GraspState state = GraspState.Idle;
    private InputDevice inputDevice;
    private bool buttonWasPressed;
    private bool savedVelocityPreviewEnabled;
    private bool savedGripperUseTrigger;
    private bool hasSavedVelocityPreviewState;
    private bool hasSavedGripperState;
    private float stateTimer;
    private Vector3 preGraspPosition;
    private Vector3 graspPosition;
    private Vector3 liftPosition;
    private Quaternion sequenceRotation = Quaternion.identity;
    private Vector3 assistWorldVelocity;
    private Bounds selectedTargetBounds;
    private bool hasSelectedTargetBounds;

    public bool IsAssistActive => state != GraspState.Idle;
    public bool HasSelectedTarget => graspTarget != null || hasSelectedTargetBounds;
    public Vector3 PreGraspPosition => preGraspPosition;
    public Vector3 GraspPosition => graspPosition;
    public Vector3 LiftPosition => liftPosition;

    private void Awake()
    {
        ResolveReferences();
    }

    private void Update()
    {
        ResolveReferences();

        bool pressedThisFrame = ReadStartButtonPressedThisFrame();
        if (Input.GetKeyDown(keyboardAbortKey) || (IsAssistActive && pressedThisFrame))
        {
            AbortAssist("UR5 grasp assist aborted.");
            return;
        }

        if (Input.GetKeyDown(keyboardStartKey) || (!IsAssistActive && pressedThisFrame))
        {
            StartGraspAssist();
        }
    }

    private void FixedUpdate()
    {
        if (!IsAssistActive || tcpTarget == null)
        {
            return;
        }

        switch (state)
        {
            case GraspState.MoveToPreGrasp:
                MoveTowardWaypoint(preGraspPosition, sequenceRotation, GraspState.MoveToGrasp);
                break;
            case GraspState.MoveToGrasp:
                MoveTowardWaypoint(graspPosition, sequenceRotation, GraspState.SettleAtGrasp);
                break;
            case GraspState.SettleAtGrasp:
                HoldTargetPose(graspPosition, sequenceRotation);
                stateTimer = HasActualTcpReachedTarget(true)
                    ? stateTimer + Time.fixedDeltaTime
                    : 0.0f;
                if (stateTimer >= finalGraspSettleSeconds)
                {
                    EnterState(GraspState.CloseGripper);
                }
                break;
            case GraspState.CloseGripper:
                HoldTargetPose(graspPosition, sequenceRotation);
                stateTimer += Time.fixedDeltaTime;
                if (gripperController != null)
                {
                    gripperController.CloseGripper();
                }

                if (stateTimer >= closeGripperSeconds)
                {
                    EnterState(GraspState.Lift);
                }
                break;
            case GraspState.Lift:
                MoveTowardWaypoint(liftPosition, sequenceRotation, GraspState.Complete);
                break;
            case GraspState.Complete:
                FinishAssist();
                break;
        }
    }

    private void OnDisable()
    {
        RestoreManualControl();
        state = GraspState.Idle;
    }

    private void OnValidate()
    {
        targetSearchRadius = Mathf.Max(0.01f, targetSearchRadius);
        maxAutoTargetSize = Mathf.Max(0.0f, maxAutoTargetSize);
        preGraspHeight = Mathf.Max(0.0f, preGraspHeight);
        graspClearance = Mathf.Max(0.0f, graspClearance);
        liftHeight = Mathf.Max(0.0f, liftHeight);
        assistMoveSpeed = Mathf.Max(0.0f, assistMoveSpeed);
        finalApproachSpeed = Mathf.Max(0.0f, finalApproachSpeed);
        assistMoveAcceleration = Mathf.Max(0.0f, assistMoveAcceleration);
        assistRotationSpeedDegreesPerSecond = Mathf.Max(0.0f, assistRotationSpeedDegreesPerSecond);
        waypointTolerance = Mathf.Max(0.001f, waypointTolerance);
        transitActualPositionTolerance = Mathf.Max(0.001f, transitActualPositionTolerance);
        finalGraspActualPositionTolerance = Mathf.Max(0.0005f, finalGraspActualPositionTolerance);
        actualPositionTolerance = Mathf.Max(0.001f, actualPositionTolerance);
        actualRotationToleranceDegrees = Mathf.Max(0.0f, actualRotationToleranceDegrees);
        finalGraspActualRotationToleranceDegrees = Mathf.Max(0.05f, finalGraspActualRotationToleranceDegrees);
        finalGraspSettleSeconds = Mathf.Max(0.0f, finalGraspSettleSeconds);
        closeGripperSeconds = Mathf.Max(0.0f, closeGripperSeconds);
        robotBodyClearanceRadius = Mathf.Max(0.0f, robotBodyClearanceRadius);
    }

    private void OnDrawGizmos()
    {
        if (!drawDebug || state == GraspState.Idle)
        {
            return;
        }

        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(preGraspPosition, 0.025f);
        Gizmos.DrawLine(preGraspPosition, graspPosition);
        // Blue arrow is tool0 +Z. During a top-down grasp it must point from
        // the pre-grasp point toward the object, i.e. world-down.
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(preGraspPosition, sequenceRotation * Vector3.forward * 0.10f);
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(graspPosition, 0.020f);
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(liftPosition, 0.025f);
        Gizmos.DrawLine(graspPosition, liftPosition);
    }

    public bool StartGraspAssist()
    {
        ResolveReferences();
        if (tcpTarget == null)
        {
            LogWarning("UR5 grasp assist needs a TcpTarget.");
            return false;
        }

        if (!ResolveSelectedTargetBounds())
        {
            LogWarning("UR5 grasp assist did not find a nearby grasp target.");
            return false;
        }

        BuildSequenceWaypoints();
        if (!IsSafeTargetPosition(preGraspPosition)
            || !IsSafeTargetPosition(graspPosition)
            || !IsSafeTargetPosition(liftPosition))
        {
            LogWarning("UR5 grasp assist blocked: planned target pose is too close to robot body.");
            return false;
        }

        SaveAndPauseManualControl();
        if (openGripperOnStart && gripperController != null)
        {
            gripperController.OpenGripper();
        }

        EnterState(GraspState.MoveToPreGrasp);
        Log("UR5 grasp assist started: PreGrasp -> Grasp -> Close -> Lift.");
        return true;
    }

    public void AbortAssist(string message)
    {
        if (!IsAssistActive)
        {
            return;
        }

        RestoreManualControl();
        state = GraspState.Idle;
        Log(message);
    }

    private void MoveTowardWaypoint(Vector3 waypointPosition, Quaternion waypointRotation, GraspState nextState)
    {
        float deltaTime = Mathf.Max(Time.fixedDeltaTime, 0.0001f);
        Vector3 toWaypoint = waypointPosition - tcpTarget.position;
        float remainingDistance = toWaypoint.magnitude;
        float acceleration = Mathf.Max(0.0f, assistMoveAcceleration);
        float stoppingSpeed = acceleration > 0.0f
            ? Mathf.Sqrt(2.0f * acceleration * remainingDistance)
            : Mathf.Max(0.0f, assistMoveSpeed);
        float stageMaximumSpeed = state == GraspState.MoveToGrasp
            ? Mathf.Max(0.0f, finalApproachSpeed)
            : Mathf.Max(0.0f, assistMoveSpeed);
        float desiredSpeed = Mathf.Min(stageMaximumSpeed, stoppingSpeed);
        Vector3 desiredVelocity = remainingDistance > 0.000001f
            ? toWaypoint / remainingDistance * desiredSpeed
            : Vector3.zero;

        assistWorldVelocity = acceleration > 0.0f
            ? Vector3.MoveTowards(assistWorldVelocity, desiredVelocity, acceleration * deltaTime)
            : desiredVelocity;
        Vector3 nextPosition = tcpTarget.position + assistWorldVelocity * deltaTime;
        if (assistWorldVelocity.magnitude * deltaTime >= remainingDistance)
        {
            nextPosition = waypointPosition;
            assistWorldVelocity = Vector3.zero;
        }

        if (workspaceLimiter != null)
        {
            nextPosition = workspaceLimiter.ClampWorldPosition(nextPosition);
        }

        if (!IsSafeTargetPosition(nextPosition))
        {
            AbortAssist("UR5 grasp assist stopped: target would enter robot body clearance zone.");
            return;
        }

        Quaternion nextRotation = Quaternion.RotateTowards(
            tcpTarget.rotation,
            waypointRotation,
            Mathf.Max(0.0f, assistRotationSpeedDegreesPerSecond) * Time.fixedDeltaTime);
        tcpTarget.SetPositionAndRotation(nextPosition, nextRotation);
        if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite("GraspAssist");
        }

        if (Vector3.Distance(tcpTarget.position, waypointPosition) <= waypointTolerance
            && HasActualTcpReachedTarget(state == GraspState.MoveToGrasp))
        {
            EnterState(nextState);
        }
    }

    private void HoldTargetPose(Vector3 position, Quaternion rotation)
    {
        Vector3 clampedPosition = workspaceLimiter != null
            ? workspaceLimiter.ClampWorldPosition(position)
            : position;
        tcpTarget.SetPositionAndRotation(clampedPosition, rotation);
        if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite("GraspAssist");
        }
    }

    private bool HasActualTcpReachedTarget(bool requireFinalGraspPrecision = false)
    {
        if (tcpFollower == null)
        {
            return true;
        }

        float positionTolerance = requireFinalGraspPrecision
            ? finalGraspActualPositionTolerance
            : transitActualPositionTolerance;
        float rotationTolerance = requireFinalGraspPrecision
            ? finalGraspActualRotationToleranceDegrees
            : actualRotationToleranceDegrees;
        return tcpFollower.PositionError <= positionTolerance
            && tcpFollower.RotationErrorDegrees <= rotationTolerance;
    }

    private void EnterState(GraspState nextState)
    {
        state = nextState;
        stateTimer = 0.0f;
        // Each stage starts from rest. In particular, this prevents the
        // horizontal PreGrasp velocity from leaking into the vertical descent.
        assistWorldVelocity = Vector3.zero;
        Log("UR5 grasp assist state: " + state);
    }

    private void FinishAssist()
    {
        RestoreManualControl();
        state = GraspState.Idle;
        Log("UR5 grasp assist completed.");
    }

    private void SaveAndPauseManualControl()
    {
        if (pauseVelocityPreviewDuringAssist && velocityTeleop != null && !hasSavedVelocityPreviewState)
        {
            savedVelocityPreviewEnabled = velocityTeleop.enableUnityPreview;
            velocityTeleop.enableUnityPreview = false;
            hasSavedVelocityPreviewState = true;
        }

        if (ignoreManualGripperTriggerDuringAssist && gripperController != null && !hasSavedGripperState)
        {
            savedGripperUseTrigger = gripperController.useTrigger;
            gripperController.useTrigger = false;
            hasSavedGripperState = true;
        }
    }

    private void RestoreManualControl()
    {
        if (velocityTeleop != null && hasSavedVelocityPreviewState)
        {
            velocityTeleop.enableUnityPreview = savedVelocityPreviewEnabled;
        }

        if (gripperController != null && hasSavedGripperState)
        {
            gripperController.useTrigger = savedGripperUseTrigger;
        }

        hasSavedVelocityPreviewState = false;
        hasSavedGripperState = false;
    }

    private bool ResolveSelectedTargetBounds()
    {
        hasSelectedTargetBounds = false;

        if (graspTarget != null && TryGetTargetBounds(graspTarget, out selectedTargetBounds))
        {
            hasSelectedTargetBounds = true;
            return true;
        }

        if (!autoSelectNearestTarget || tcpTarget == null)
        {
            return false;
        }

        Collider bestCollider = null;
        float bestDistanceSquared = float.PositiveInfinity;
        Collider[] colliders = Physics.OverlapSphere(
            tcpTarget.position,
            targetSearchRadius,
            graspableLayers,
            QueryTriggerInteraction.Ignore);

        foreach (Collider candidate in colliders)
        {
            if (!IsValidGraspTargetCollider(candidate))
            {
                continue;
            }

            float distanceSquared = (candidate.bounds.center - tcpTarget.position).sqrMagnitude;
            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestCollider = candidate;
            }
        }

        if (bestCollider == null)
        {
            return TryResolveNearestRendererBounds();
        }

        graspTarget = bestCollider.transform;
        selectedTargetBounds = bestCollider.bounds;
        hasSelectedTargetBounds = true;
        return true;
    }

    private bool TryResolveNearestRendererBounds()
    {
        Renderer bestRenderer = null;
        float bestDistanceSquared = float.PositiveInfinity;
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        foreach (Renderer candidate in renderers)
        {
            if (candidate == null || !candidate.enabled || !IsValidGraspTargetTransform(candidate.transform))
            {
                continue;
            }

            Bounds bounds = candidate.bounds;
            if (maxAutoTargetSize > 0.0f && bounds.size.magnitude > maxAutoTargetSize)
            {
                continue;
            }

            float distanceSquared = (bounds.center - tcpTarget.position).sqrMagnitude;
            if (distanceSquared <= targetSearchRadius * targetSearchRadius
                && distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestRenderer = candidate;
            }
        }

        if (bestRenderer == null)
        {
            return false;
        }

        graspTarget = bestRenderer.transform;
        selectedTargetBounds = bestRenderer.bounds;
        hasSelectedTargetBounds = true;
        return true;
    }

    private bool TryGetTargetBounds(Transform target, out Bounds bounds)
    {
        Collider[] colliders = target.GetComponentsInChildren<Collider>();
        bool foundBounds = false;
        bounds = new Bounds(target.position, Vector3.zero);
        foreach (Collider collider in colliders)
        {
            if (collider == null || collider.isTrigger)
            {
                continue;
            }

            if (!foundBounds)
            {
                bounds = collider.bounds;
                foundBounds = true;
            }
            else
            {
                bounds.Encapsulate(collider.bounds);
            }
        }

        if (foundBounds)
        {
            return true;
        }

        Renderer[] renderers = target.GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            if (renderer == null)
            {
                continue;
            }

            if (!foundBounds)
            {
                bounds = renderer.bounds;
                foundBounds = true;
            }
            else
            {
                bounds.Encapsulate(renderer.bounds);
            }
        }

        return foundBounds;
    }

    private bool IsValidGraspTargetCollider(Collider collider)
    {
        if (collider == null || collider.isTrigger)
        {
            return false;
        }

        return IsValidGraspTargetTransform(collider.transform)
            && (maxAutoTargetSize <= 0.0f || collider.bounds.size.magnitude <= maxAutoTargetSize);
    }

    private bool IsValidGraspTargetTransform(Transform candidateTransform)
    {
        if (candidateTransform == null
            || candidateTransform == tcpTarget
            || candidateTransform.IsChildOf(tcpTarget)
            || candidateTransform.name.StartsWith("ActualTcp"))
        {
            return false;
        }

        if (robotRoot != null && candidateTransform.IsChildOf(robotRoot))
        {
            return false;
        }

        return string.IsNullOrEmpty(requiredTargetTag)
            || candidateTransform.gameObject.tag == requiredTargetTag;
    }

    private void BuildSequenceWaypoints()
    {
        Vector3 approachDirection = GetApproachDirection();
        Vector3 topPoint = GetBoundsSurfacePoint(selectedTargetBounds, approachDirection);
        graspPosition = alignPadCenterToObjectCenter
            ? selectedTargetBounds.center + approachDirection * padCenterOffsetAlongApproach
            : topPoint + approachDirection * graspClearance;
        preGraspPosition = graspPosition + approachDirection * preGraspHeight;
        liftPosition = graspPosition + approachDirection * liftHeight;

        if (workspaceLimiter != null)
        {
            preGraspPosition = workspaceLimiter.ClampWorldPosition(preGraspPosition);
            graspPosition = workspaceLimiter.ClampWorldPosition(graspPosition);
            liftPosition = workspaceLimiter.ClampWorldPosition(liftPosition);
        }

        sequenceRotation = GetSequenceRotation(approachDirection);
    }

    private Quaternion GetSequenceRotation(Vector3 approachDirection)
    {
        if (preserveCurrentTcpRotation && tcpTarget != null)
        {
            return tcpTarget.rotation;
        }

        if (!alignToolForwardAgainstApproachDirection)
        {
            return Quaternion.Euler(fixedTcpRotationEuler);
        }

        // Derive the target in the physical grasp frame (base -> pad midpoint)
        // and then convert it to the imported tool frame. This avoids relying
        // on a guessed tool0 axis convention.
        Vector3 yawReference = graspYawReferenceWorld.sqrMagnitude > 0.0001f
            ? graspYawReferenceWorld
            : robotRoot != null ? robotRoot.forward : Vector3.forward;
        if (tcpFollower != null)
        {
            return tcpFollower.GetToolRotationForGraspApproach(-approachDirection, yawReference);
        }

        Vector3 toolForwardWorld = -approachDirection.normalized;
        Vector3 toolUpWorld = Vector3.ProjectOnPlane(yawReference, toolForwardWorld);
        if (toolUpWorld.sqrMagnitude < 0.0001f)
        {
            toolUpWorld = Vector3.ProjectOnPlane(Vector3.right, toolForwardWorld);
        }

        return Quaternion.LookRotation(toolForwardWorld, toolUpWorld.normalized);
    }

    private Vector3 GetApproachDirection()
    {
        return approachDirectionWorld.sqrMagnitude > 0.0001f
            ? approachDirectionWorld.normalized
            : Vector3.up;
    }

    private Vector3 GetBoundsSurfacePoint(Bounds bounds, Vector3 direction)
    {
        Vector3 normalizedDirection = direction.normalized;
        float projectedExtent = Mathf.Abs(normalizedDirection.x) * bounds.extents.x
            + Mathf.Abs(normalizedDirection.y) * bounds.extents.y
            + Mathf.Abs(normalizedDirection.z) * bounds.extents.z;
        return bounds.center + normalizedDirection * projectedExtent;
    }

    private bool IsSafeTargetPosition(Vector3 candidatePosition)
    {
        if (!preventRobotBodyTargetCollision || robotRoot == null || robotBodyClearanceRadius <= 0.0f)
        {
            return true;
        }

        Collider[] colliders = Physics.OverlapSphere(
            candidatePosition,
            robotBodyClearanceRadius,
            robotBodyLayers,
            QueryTriggerInteraction.Ignore);

        foreach (Collider collider in colliders)
        {
            if (collider == null)
            {
                continue;
            }

            Transform colliderTransform = collider.transform;
            if (!colliderTransform.IsChildOf(robotRoot))
            {
                continue;
            }

            if (IsAllowedNearTcpRobotPart(colliderTransform))
            {
                continue;
            }

            return false;
        }

        return true;
    }

    private bool IsAllowedNearTcpRobotPart(Transform robotPart)
    {
        string lowerName = robotPart.name.ToLowerInvariant();
        foreach (string hint in allowedRobotNearTcpNameHints)
        {
            if (!string.IsNullOrEmpty(hint) && lowerName.Contains(hint.ToLowerInvariant()))
            {
                return true;
            }
        }

        return false;
    }

    private bool ReadStartButtonPressedThisFrame()
    {
        if (!startWithSecondaryButton && !startWithPrimaryButton)
        {
            return false;
        }

        if (!inputDevice.isValid)
        {
            inputDevice = InputDevices.GetDeviceAtXRNode(controllerNode);
        }

        bool secondaryPressed = startWithSecondaryButton
            && inputDevice.isValid
            && inputDevice.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryValue)
            && secondaryValue;
        bool primaryPressed = startWithPrimaryButton
            && inputDevice.isValid
            && inputDevice.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryValue)
            && primaryValue;
        bool isPressed = secondaryPressed || primaryPressed;
        bool pressedThisFrame = isPressed && !buttonWasPressed;
        buttonWasPressed = isPressed;
        return pressedThisFrame;
    }

    private void ResolveReferences()
    {
        if (robotRoot == null)
        {
            GameObject robot = GameObject.Find("ur5_robot");
            if (robot == null)
            {
                robot = GameObject.Find("base_link");
            }

            robotRoot = robot != null ? robot.transform : null;
        }

        if (tcpTarget == null)
        {
            GameObject target = GameObject.Find("TcpTarget");
            tcpTarget = target != null ? target.transform : null;
        }

        if (tcpFollower == null && robotRoot != null)
        {
            tcpFollower = robotRoot.GetComponent<Ur5TcpTargetFollower>();
        }

        if (velocityTeleop == null)
        {
            velocityTeleop = FindObjectOfType<Ur5CartesianVelocityTeleopController>();
        }

        if (workspaceLimiter == null && tcpTarget != null)
        {
            workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
        }

        if (targetWriteMonitor == null && tcpTarget != null)
        {
            targetWriteMonitor = tcpTarget.GetComponent<TcpTargetWriteMonitor>();
        }

        if (gripperController == null && robotRoot != null)
        {
            gripperController = robotRoot.GetComponent<Quest3RobotiqGripperController>();
        }
    }

    private void Log(string message)
    {
        if (logStatus)
        {
            Debug.Log(message);
        }
    }

    private void LogWarning(string message)
    {
        if (logStatus)
        {
            Debug.LogWarning(message);
        }
    }
}

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
    public Quest3RobotiqGripperController gripperController;
    public Transform graspTarget;

    [Header("Input")]
    public XRNode controllerNode = XRNode.RightHand;
    public bool startWithSecondaryButton = true;
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
    public float graspClearance = 0.015f;
    public float liftHeight = 0.16f;
    public float assistMoveSpeed = 0.16f;
    public float assistRotationSpeedDegreesPerSecond = 180.0f;
    public float waypointTolerance = 0.012f;
    public float actualPositionTolerance = 0.025f;
    public float actualRotationToleranceDegrees = 8.0f;
    public float closeGripperSeconds = 0.45f;
    public bool openGripperOnStart = true;
    public bool preserveCurrentTcpRotation = true;
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
                MoveTowardWaypoint(graspPosition, sequenceRotation, GraspState.CloseGripper);
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
        assistRotationSpeedDegreesPerSecond = Mathf.Max(0.0f, assistRotationSpeedDegreesPerSecond);
        waypointTolerance = Mathf.Max(0.001f, waypointTolerance);
        actualPositionTolerance = Mathf.Max(0.001f, actualPositionTolerance);
        actualRotationToleranceDegrees = Mathf.Max(0.0f, actualRotationToleranceDegrees);
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
        if (!IsSafeTargetPosition(preGraspPosition) || !IsSafeTargetPosition(graspPosition))
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
        Vector3 nextPosition = Vector3.MoveTowards(
            tcpTarget.position,
            waypointPosition,
            Mathf.Max(0.0f, assistMoveSpeed) * Time.fixedDeltaTime);

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

        if (Vector3.Distance(tcpTarget.position, waypointPosition) <= waypointTolerance
            && HasActualTcpReachedTarget())
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
    }

    private bool HasActualTcpReachedTarget()
    {
        if (tcpFollower == null)
        {
            return true;
        }

        return tcpFollower.PositionError <= actualPositionTolerance
            && tcpFollower.RotationErrorDegrees <= actualRotationToleranceDegrees;
    }

    private void EnterState(GraspState nextState)
    {
        state = nextState;
        stateTimer = 0.0f;
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
            return false;
        }

        graspTarget = bestCollider.transform;
        selectedTargetBounds = bestCollider.bounds;
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

        Transform colliderTransform = collider.transform;
        if (colliderTransform == tcpTarget || colliderTransform.IsChildOf(tcpTarget))
        {
            return false;
        }

        if (robotRoot != null && colliderTransform.IsChildOf(robotRoot))
        {
            return false;
        }

        if (!string.IsNullOrEmpty(requiredTargetTag) && collider.gameObject.tag != requiredTargetTag)
        {
            return false;
        }

        if (maxAutoTargetSize > 0.0f && collider.bounds.size.magnitude > maxAutoTargetSize)
        {
            return false;
        }

        return true;
    }

    private void BuildSequenceWaypoints()
    {
        Vector3 approachDirection = GetApproachDirection();
        Vector3 topPoint = GetBoundsSurfacePoint(selectedTargetBounds, approachDirection);
        graspPosition = topPoint + approachDirection * graspClearance;
        preGraspPosition = topPoint + approachDirection * preGraspHeight;
        liftPosition = graspPosition + approachDirection * liftHeight;

        if (workspaceLimiter != null)
        {
            preGraspPosition = workspaceLimiter.ClampWorldPosition(preGraspPosition);
            graspPosition = workspaceLimiter.ClampWorldPosition(graspPosition);
            liftPosition = workspaceLimiter.ClampWorldPosition(liftPosition);
        }

        sequenceRotation = preserveCurrentTcpRotation && tcpTarget != null
            ? tcpTarget.rotation
            : Quaternion.Euler(fixedTcpRotationEuler);
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
        if (!startWithSecondaryButton)
        {
            return false;
        }

        if (!inputDevice.isValid)
        {
            inputDevice = InputDevices.GetDeviceAtXRNode(controllerNode);
        }

        bool isPressed = inputDevice.isValid
            && inputDevice.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryPressed)
            && secondaryPressed;
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

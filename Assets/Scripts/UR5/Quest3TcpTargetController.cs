using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.XR;

[DefaultExecutionOrder(-100)]
public class Quest3TcpTargetController : MonoBehaviour
{
    private enum ControlChannel
    {
        None,
        Position,
        Rotation
    }

    [Header("Timing")]
    [Tooltip("Apply the smoothed TCP target on the physics tick so IK and target motion stay in the same control loop.")]
    public bool applyTargetInFixedUpdate = true;

    [Header("Controllers")]
    [FormerlySerializedAs("controllerNode")]
    public XRNode positionControllerNode = XRNode.RightHand;
    public XRNode rotationControllerNode = XRNode.RightHand;
    [FormerlySerializedAs("useGripAsClutch")]
    public bool usePositionGripAsClutch = true;
    public bool useRotationGripAsClutch = true;
    public Transform xrOrigin;
    public bool convertControllerPoseThroughXrOrigin = true;

    [Header("Mapping")]
    public float translationScale = 0.35f;
    [Range(0.0f, 3.0f)] public float rotationScale = 0.75f;
    public bool followControllerRotation = true;
    public float positionSmoothing = 14.0f;
    public float rotationSmoothing = 18.0f;
    [Tooltip("Caps target motion so the robot can track a fast controller movement smoothly.")]
    public float maximumTargetSpeed = 0.10f;
    public float maximumTargetAcceleration = 0.45f;
    public float maximumTargetAngularSpeed = 120.0f;
    public float translationDeadbandMeters = 0.0020f;
    public float rotationDeadbandDegrees = 0.50f;

    [Header("Input Filtering")]
    public bool filterControllerPose = true;
    public float controllerPositionJitterDeadbandMeters = 0.0015f;
    public float controllerRotationJitterDeadbandDegrees = 0.25f;
    public float controllerPositionFilterSharpness = 18.0f;
    public float controllerRotationFilterSharpness = 20.0f;
    [Tooltip("Analog grip value required to enter clutch. Higher values avoid accidental activation.")]
    [Range(0.0f, 1.0f)] public float gripPressThreshold = 0.65f;
    [Tooltip("Analog grip value below which clutch releases. Lower than press threshold to prevent chatter.")]
    [Range(0.0f, 1.0f)] public float gripReleaseThreshold = 0.40f;
    public bool holdTargetWhenClutchReleased = true;

    [Header("Controller Coordination")]
    [Tooltip("Only one controller edits the TCP target at a time. This avoids target jumps when both grips are held.")]
    public bool lockToOneControllerAtATime = false;
    [Tooltip("Short handoff delay after releasing one grip before the other controller can take over.")]
    public float clutchSwitchCooldownSeconds = 0.08f;
    public bool preferPositionWhenBothGripsPressed = true;

    [Header("Fine Control")]
    [Tooltip("Hold A on the right controller or X on the left controller while gripping for slower target motion.")]
    public bool enableFineControlButton = true;
    [Range(0.1f, 1.0f)] public float fineTargetSpeedMultiplier = 0.35f;
    [Range(0.1f, 1.0f)] public float fineTargetAccelerationMultiplier = 0.45f;
    [Range(0.1f, 1.0f)] public float fineTargetAngularSpeedMultiplier = 0.35f;

    [Header("Rotation Hold")]
    [Tooltip("Position-only right-hand control keeps the existing target rotation instead of reapplying stale rotation commands.")]
    public bool applyRotationOnlyWhileRotationClutched = true;

    [Header("Workspace Limit")]
    public bool clampWorkspace = true;
    public Vector3 minPosition = new Vector3(-0.8f, 0.0f, -0.8f);
    public Vector3 maxPosition = new Vector3(0.8f, 1.2f, 0.8f);

    [Header("Debug")]
    public bool logDeviceStatus = true;

    private InputDevice positionDevice;
    private InputDevice rotationDevice;
    private TcpTargetWorkspaceLimiter workspaceLimiter;
    private bool wasPositionClutched;
    private bool wasRotationClutched;
    private bool hasLoggedMissingPositionDevice;
    private bool hasLoggedMissingRotationDevice;
    private bool positionGripLatched;
    private bool rotationGripLatched;
    private ControlChannel activeControlChannel;
    private float blockNewClutchUntilTime;

    private Vector3 positionClutchStartControllerPosition;
    private Vector3 positionClutchStartTargetPosition;
    private Quaternion rotationClutchStartControllerRotation;
    private Quaternion rotationClutchStartTargetRotation;

    private Vector3 desiredPosition;
    private Quaternion desiredRotation;
    private Vector3 targetVelocity;
    private Vector3 filteredPositionControllerPosition;
    private Quaternion filteredRotationControllerRotation;
    private bool hasFilteredPositionPose;
    private bool hasFilteredRotationPose;

    public bool IsDeviceValid => positionDevice.isValid || rotationDevice.isValid;
    public bool IsClutched => IsPositionClutched || IsRotationClutched;
    public bool IsPositionClutched { get; private set; }
    public bool IsRotationClutched { get; private set; }
    public bool IsFineControlActive { get; private set; }

    private void Start()
    {
        desiredPosition = transform.position;
        desiredRotation = transform.rotation;
        workspaceLimiter = GetComponent<TcpTargetWorkspaceLimiter>();
        ResolveXrOrigin();
        TryRefreshPositionDevice();
        TryRefreshRotationDevice();
    }

    private void Update()
    {
        if (!positionDevice.isValid)
        {
            TryRefreshPositionDevice();
        }

        if (!rotationDevice.isValid)
        {
            TryRefreshRotationDevice();
        }

        bool hasPosition = TryReadControllerPosition(positionDevice, out Vector3 controllerPosition);
        bool hasRotation = TryReadControllerRotation(rotationDevice, out Quaternion controllerRotation);

        bool wantsPositionClutch = hasPosition
            && (!usePositionGripAsClutch || ReadGripClutch(positionDevice, ref positionGripLatched));
        bool wantsRotationClutch = hasRotation
            && followControllerRotation
            && (!useRotationGripAsClutch || ReadGripClutch(rotationDevice, ref rotationGripLatched));
        ResolveActiveClutches(wantsPositionClutch, wantsRotationClutch);
        IsFineControlActive =
            (IsPositionClutched && ReadFineControl(positionDevice))
            || (IsRotationClutched && ReadFineControl(rotationDevice));

        if (IsPositionClutched && !wasPositionClutched)
        {
            CapturePositionClutchStart(controllerPosition);
        }

        if (IsRotationClutched && !wasRotationClutched)
        {
            CaptureRotationClutchStart(controllerRotation);
        }

        if (IsPositionClutched)
        {
            UpdateDesiredPosition(controllerPosition);
        }
        else if (wasPositionClutched && holdTargetWhenClutchReleased)
        {
            HoldCurrentTargetPosition();
        }

        if (IsRotationClutched)
        {
            UpdateDesiredRotation(controllerRotation);
        }
        else if (wasRotationClutched && holdTargetWhenClutchReleased)
        {
            HoldCurrentTargetRotation();
        }

        if (IsClutched && !applyTargetInFixedUpdate)
        {
            ApplyTarget(Time.deltaTime);
        }

        wasPositionClutched = IsPositionClutched;
        wasRotationClutched = IsRotationClutched;
    }

    private void FixedUpdate()
    {
        if (applyTargetInFixedUpdate && IsClutched)
        {
            ApplyTarget(Time.fixedDeltaTime);
        }
    }

    private void TryRefreshPositionDevice()
    {
        positionDevice = InputDevices.GetDeviceAtXRNode(positionControllerNode);
        LogDeviceStatus(positionDevice, "position", ref hasLoggedMissingPositionDevice);
    }

    private void TryRefreshRotationDevice()
    {
        rotationDevice = InputDevices.GetDeviceAtXRNode(rotationControllerNode);
        LogDeviceStatus(rotationDevice, "rotation", ref hasLoggedMissingRotationDevice);
    }

    private void LogDeviceStatus(InputDevice device, string role, ref bool hasLoggedMissingDevice)
    {
        if (device.isValid)
        {
            hasLoggedMissingDevice = false;
            if (logDeviceStatus)
            {
                Debug.Log("Quest " + role + " controller connected: " + device.name);
            }
        }
        else if (logDeviceStatus && !hasLoggedMissingDevice)
        {
            hasLoggedMissingDevice = true;
            Debug.LogWarning("Quest " + role + " controller not found yet. Start Play Mode with Quest Link/Air Link or build to Quest.");
        }
    }

    private bool TryReadControllerPosition(InputDevice device, out Vector3 position)
    {
        position = Vector3.zero;
        if (!device.isValid || !device.TryGetFeatureValue(CommonUsages.devicePosition, out position))
        {
            return false;
        }

        if (convertControllerPoseThroughXrOrigin)
        {
            ResolveXrOrigin();
            if (xrOrigin != null)
            {
                position = xrOrigin.TransformPoint(position);
            }
        }

        return true;
    }

    private bool TryReadControllerRotation(InputDevice device, out Quaternion rotation)
    {
        rotation = Quaternion.identity;
        if (!device.isValid || !device.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation))
        {
            return false;
        }

        if (convertControllerPoseThroughXrOrigin)
        {
            ResolveXrOrigin();
            if (xrOrigin != null)
            {
                rotation = xrOrigin.rotation * rotation;
            }
        }

        return true;
    }

    private bool ReadGripClutch(InputDevice device, ref bool gripLatched)
    {
        if (!device.isValid)
        {
            gripLatched = false;
            return false;
        }

        bool hasAnalogGrip = device.TryGetFeatureValue(CommonUsages.grip, out float gripAmount);
        bool hasGripButton = device.TryGetFeatureValue(CommonUsages.gripButton, out bool gripPressed);

        if (hasAnalogGrip)
        {
            float pressThreshold = Mathf.Clamp01(gripPressThreshold);
            float releaseThreshold = Mathf.Min(pressThreshold, Mathf.Clamp01(gripReleaseThreshold));
            gripLatched = gripLatched
                ? gripAmount > releaseThreshold || (hasGripButton && gripPressed)
                : gripAmount >= pressThreshold || (hasGripButton && gripPressed);
            return gripLatched;
        }

        if (hasGripButton)
        {
            gripLatched = gripPressed;
            return gripLatched;
        }

        gripLatched = false;
        return false;
    }

    private bool ReadFineControl(InputDevice device)
    {
        return enableFineControlButton
            && device.isValid
            && device.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryPressed)
            && primaryPressed;
    }

    private void ResolveActiveClutches(bool wantsPositionClutch, bool wantsRotationClutch)
    {
        if (!lockToOneControllerAtATime)
        {
            IsPositionClutched = wantsPositionClutch;
            IsRotationClutched = wantsRotationClutch;
            if (IsPositionClutched)
            {
                activeControlChannel = ControlChannel.Position;
            }
            else if (IsRotationClutched)
            {
                activeControlChannel = ControlChannel.Rotation;
            }
            else
            {
                activeControlChannel = ControlChannel.None;
            }

            return;
        }

        if (activeControlChannel == ControlChannel.Position && wantsPositionClutch)
        {
            SetActiveClutchChannel(ControlChannel.Position);
            return;
        }

        if (activeControlChannel == ControlChannel.Rotation && wantsRotationClutch)
        {
            SetActiveClutchChannel(ControlChannel.Rotation);
            return;
        }

        if (activeControlChannel != ControlChannel.None)
        {
            SetActiveClutchChannel(ControlChannel.None);
            blockNewClutchUntilTime = Time.time + Mathf.Max(0.0f, clutchSwitchCooldownSeconds);
            return;
        }

        if (Time.time < blockNewClutchUntilTime)
        {
            SetActiveClutchChannel(ControlChannel.None);
            return;
        }

        if (wantsPositionClutch && wantsRotationClutch)
        {
            SetActiveClutchChannel(preferPositionWhenBothGripsPressed
                ? ControlChannel.Position
                : ControlChannel.Rotation);
            return;
        }

        if (wantsPositionClutch)
        {
            SetActiveClutchChannel(ControlChannel.Position);
            return;
        }

        if (wantsRotationClutch)
        {
            SetActiveClutchChannel(ControlChannel.Rotation);
            return;
        }

        SetActiveClutchChannel(ControlChannel.None);
    }

    private void SetActiveClutchChannel(ControlChannel channel)
    {
        activeControlChannel = channel;
        IsPositionClutched = channel == ControlChannel.Position;
        IsRotationClutched = channel == ControlChannel.Rotation;
    }

    private void CapturePositionClutchStart(Vector3 controllerPosition)
    {
        positionClutchStartControllerPosition = controllerPosition;
        positionClutchStartTargetPosition = transform.position;
        desiredPosition = transform.position;
        desiredRotation = transform.rotation;
        targetVelocity = Vector3.zero;
        filteredPositionControllerPosition = controllerPosition;
        hasFilteredPositionPose = true;
    }

    private void CaptureRotationClutchStart(Quaternion controllerRotation)
    {
        rotationClutchStartControllerRotation = controllerRotation;
        rotationClutchStartTargetRotation = transform.rotation;
        desiredRotation = transform.rotation;
        filteredRotationControllerRotation = controllerRotation;
        hasFilteredRotationPose = true;
    }

    private void UpdateDesiredPosition(Vector3 controllerPosition)
    {
        FilterControllerPosition(ref controllerPosition);

        Vector3 controllerDelta = controllerPosition - positionClutchStartControllerPosition;
        if (controllerDelta.magnitude < translationDeadbandMeters)
        {
            controllerDelta = Vector3.zero;
        }

        desiredPosition = positionClutchStartTargetPosition + controllerDelta * translationScale;

        if (clampWorkspace)
        {
            desiredPosition = new Vector3(
                Mathf.Clamp(desiredPosition.x, minPosition.x, maxPosition.x),
                Mathf.Clamp(desiredPosition.y, minPosition.y, maxPosition.y),
                Mathf.Clamp(desiredPosition.z, minPosition.z, maxPosition.z));
        }

        desiredPosition = ClampWithWorkspaceLimiter(desiredPosition);
    }

    private void UpdateDesiredRotation(Quaternion controllerRotation)
    {
        FilterControllerRotation(ref controllerRotation);

        Quaternion controllerDeltaRotation = controllerRotation * Quaternion.Inverse(rotationClutchStartControllerRotation);
        controllerDeltaRotation.ToAngleAxis(out float deltaAngle, out Vector3 deltaAxis);
        if (deltaAngle > 180.0f)
        {
            deltaAngle -= 360.0f;
        }

        if (Mathf.Abs(deltaAngle) < rotationDeadbandDegrees)
        {
            deltaAngle = 0.0f;
        }

        Quaternion scaledRotation = Quaternion.AngleAxis(deltaAngle * rotationScale, deltaAxis);
        desiredRotation = scaledRotation * rotationClutchStartTargetRotation;
    }

    private void FilterControllerPosition(ref Vector3 controllerPosition)
    {
        if (!filterControllerPose)
        {
            return;
        }

        if (!hasFilteredPositionPose)
        {
            filteredPositionControllerPosition = controllerPosition;
            hasFilteredPositionPose = true;
            return;
        }

        float deltaTime = Mathf.Max(Time.deltaTime, 0.0001f);
        if (Vector3.Distance(filteredPositionControllerPosition, controllerPosition) > controllerPositionJitterDeadbandMeters)
        {
            float positionT = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, controllerPositionFilterSharpness) * deltaTime);
            filteredPositionControllerPosition = Vector3.Lerp(filteredPositionControllerPosition, controllerPosition, positionT);
        }

        controllerPosition = filteredPositionControllerPosition;
    }

    private void FilterControllerRotation(ref Quaternion controllerRotation)
    {
        if (!filterControllerPose)
        {
            return;
        }

        if (!hasFilteredRotationPose)
        {
            filteredRotationControllerRotation = controllerRotation;
            hasFilteredRotationPose = true;
            return;
        }

        float deltaTime = Mathf.Max(Time.deltaTime, 0.0001f);
        if (Quaternion.Angle(filteredRotationControllerRotation, controllerRotation) > controllerRotationJitterDeadbandDegrees)
        {
            float rotationT = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, controllerRotationFilterSharpness) * deltaTime);
            filteredRotationControllerRotation = Quaternion.Slerp(filteredRotationControllerRotation, controllerRotation, rotationT);
        }

        controllerRotation = filteredRotationControllerRotation;
    }

    private void HoldCurrentTargetPosition()
    {
        desiredPosition = transform.position;
        targetVelocity = Vector3.zero;
        hasFilteredPositionPose = false;
    }

    private void HoldCurrentTargetRotation()
    {
        desiredRotation = transform.rotation;
        hasFilteredRotationPose = false;
    }

    private void ApplyTarget(float deltaTime)
    {
        deltaTime = Mathf.Max(deltaTime, 0.0001f);
        Vector3 positionError = desiredPosition - transform.position;
        Vector3 desiredVelocity = positionError * positionSmoothing;
        float effectiveMaximumTargetSpeed = maximumTargetSpeed;
        float effectiveMaximumTargetAcceleration = maximumTargetAcceleration;
        if (IsFineControlActive)
        {
            effectiveMaximumTargetSpeed *= Mathf.Clamp01(fineTargetSpeedMultiplier);
            effectiveMaximumTargetAcceleration *= Mathf.Clamp01(fineTargetAccelerationMultiplier);
        }

        if (effectiveMaximumTargetSpeed > 0.0f)
        {
            desiredVelocity = Vector3.ClampMagnitude(desiredVelocity, effectiveMaximumTargetSpeed);
        }

        targetVelocity = Vector3.MoveTowards(
            targetVelocity,
            desiredVelocity,
            Mathf.Max(0.0f, effectiveMaximumTargetAcceleration) * deltaTime);
        Vector3 nextPosition = transform.position + targetVelocity * deltaTime;
        if (Vector3.Dot(desiredPosition - transform.position, desiredPosition - nextPosition) <= 0.0f)
        {
            nextPosition = desiredPosition;
            targetVelocity = Vector3.zero;
        }

        transform.position = ClampWithWorkspaceLimiter(nextPosition);

        if (followControllerRotation
            && (!applyRotationOnlyWhileRotationClutched || IsRotationClutched))
        {
            float exponentialStep = Quaternion.Angle(transform.rotation, desiredRotation)
                * (1.0f - Mathf.Exp(-rotationSmoothing * deltaTime));
            float effectiveMaximumTargetAngularSpeed = IsFineControlActive
                ? maximumTargetAngularSpeed * Mathf.Clamp01(fineTargetAngularSpeedMultiplier)
                : maximumTargetAngularSpeed;
            float maxStep = effectiveMaximumTargetAngularSpeed > 0.0f
                ? effectiveMaximumTargetAngularSpeed * deltaTime
                : float.PositiveInfinity;
            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                desiredRotation,
                Mathf.Min(exponentialStep, maxStep));
        }
    }

    private Vector3 ClampWithWorkspaceLimiter(Vector3 worldPosition)
    {
        if (workspaceLimiter == null)
        {
            workspaceLimiter = GetComponent<TcpTargetWorkspaceLimiter>();
        }

        return workspaceLimiter != null && workspaceLimiter.constrainTarget
            ? workspaceLimiter.ClampWorldPosition(worldPosition)
            : worldPosition;
    }

    private void OnValidate()
    {
        gripPressThreshold = Mathf.Clamp01(gripPressThreshold);
        gripReleaseThreshold = Mathf.Clamp(gripReleaseThreshold, 0.0f, gripPressThreshold);
        maximumTargetSpeed = Mathf.Max(0.0f, maximumTargetSpeed);
        maximumTargetAcceleration = Mathf.Max(0.0f, maximumTargetAcceleration);
        maximumTargetAngularSpeed = Mathf.Max(0.0f, maximumTargetAngularSpeed);
    }

    private void ResolveXrOrigin()
    {
        if (xrOrigin != null)
        {
            return;
        }

        GameObject foundOrigin = GameObject.Find("XR Origin (VR)");
        if (foundOrigin != null)
        {
            xrOrigin = foundOrigin.transform;
        }
    }
}

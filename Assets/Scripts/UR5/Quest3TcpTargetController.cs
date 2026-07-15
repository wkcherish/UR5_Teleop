using UnityEngine;
using UnityEngine.XR;

public class Quest3TcpTargetController : MonoBehaviour
{
    [Header("Controller")]
    public XRNode controllerNode = XRNode.RightHand;
    public bool useGripAsClutch = true;
    public bool useTriggerAsClutch = false;
    public Transform xrOrigin;
    public bool convertControllerPoseThroughXrOrigin = true;

    [Header("Mapping")]
    public float translationScale = 0.6f;
    [Range(0.0f, 2.0f)] public float rotationScale = 1.0f;
    public bool followControllerRotation = true;
    public float positionSmoothing = 18.0f;
    public float rotationSmoothing = 18.0f;
    [Tooltip("Caps target motion so the robot can track a fast controller movement smoothly.")]
    public float maximumTargetSpeed = 0.30f;
    public float maximumTargetAcceleration = 1.2f;
    public float maximumTargetAngularSpeed = 240.0f;
    public float translationDeadbandMeters = 0.0015f;
    public float rotationDeadbandDegrees = 0.35f;

    [Header("Input Filtering")]
    public bool filterControllerPose = true;
    public float controllerPositionJitterDeadbandMeters = 0.0010f;
    public float controllerRotationJitterDeadbandDegrees = 0.35f;
    public float controllerPositionFilterSharpness = 28.0f;
    public float controllerRotationFilterSharpness = 24.0f;
    public bool holdTargetWhenClutchReleased = true;

    [Header("Workspace Limit")]
    public bool clampWorkspace = true;
    public Vector3 minPosition = new Vector3(-0.8f, 0.0f, -0.8f);
    public Vector3 maxPosition = new Vector3(0.8f, 1.2f, 0.8f);

    [Header("Debug")]
    public bool logDeviceStatus = true;

    private InputDevice controllerDevice;
    private bool wasClutched;
    private bool hasLoggedMissingDevice;

    private Vector3 clutchStartControllerPosition;
    private Quaternion clutchStartControllerRotation;
    private Vector3 clutchStartTargetPosition;
    private Quaternion clutchStartTargetRotation;

    private Vector3 desiredPosition;
    private Quaternion desiredRotation;
    private Vector3 targetVelocity;
    private Vector3 filteredControllerPosition;
    private Quaternion filteredControllerRotation;
    private bool hasFilteredControllerPose;

    public bool IsDeviceValid => controllerDevice.isValid;
    public bool IsClutched { get; private set; }

    private void Start()
    {
        desiredPosition = transform.position;
        desiredRotation = transform.rotation;
        ResolveXrOrigin();
        TryRefreshDevice();
    }

    private void Update()
    {
        if (!controllerDevice.isValid)
        {
            TryRefreshDevice();
            return;
        }

        if (!TryReadControllerPose(out Vector3 controllerPosition, out Quaternion controllerRotation))
        {
            return;
        }

        IsClutched = ReadClutch();

        if (IsClutched && !wasClutched)
        {
            CaptureClutchStart(controllerPosition, controllerRotation);
        }

        if (IsClutched)
        {
            UpdateDesiredTarget(controllerPosition, controllerRotation);
            ApplyTarget();
        }
        else if (wasClutched && holdTargetWhenClutchReleased)
        {
            HoldCurrentTargetPose();
        }

        wasClutched = IsClutched;
    }

    private void TryRefreshDevice()
    {
        controllerDevice = InputDevices.GetDeviceAtXRNode(controllerNode);

        if (controllerDevice.isValid)
        {
            hasLoggedMissingDevice = false;
            if (logDeviceStatus)
            {
                Debug.Log("Quest controller connected: " + controllerDevice.name);
            }
        }
        else if (logDeviceStatus && !hasLoggedMissingDevice)
        {
            hasLoggedMissingDevice = true;
            Debug.LogWarning("Quest controller not found yet. Start Play Mode with Quest Link/Air Link or build to Quest.");
        }
    }

    private bool TryReadControllerPose(out Vector3 position, out Quaternion rotation)
    {
        bool hasPosition = controllerDevice.TryGetFeatureValue(CommonUsages.devicePosition, out position);
        bool hasRotation = controllerDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation);

        if (hasPosition && hasRotation && convertControllerPoseThroughXrOrigin)
        {
            ResolveXrOrigin();
            if (xrOrigin != null)
            {
                position = xrOrigin.TransformPoint(position);
                rotation = xrOrigin.rotation * rotation;
            }
        }

        return hasPosition && hasRotation;
    }

    private bool ReadClutch()
    {
        bool gripPressed = false;
        bool triggerPressed = false;

        if (useGripAsClutch)
        {
            controllerDevice.TryGetFeatureValue(CommonUsages.gripButton, out gripPressed);
        }

        if (useTriggerAsClutch)
        {
            controllerDevice.TryGetFeatureValue(CommonUsages.triggerButton, out triggerPressed);
        }

        return gripPressed || triggerPressed;
    }

    private void CaptureClutchStart(Vector3 controllerPosition, Quaternion controllerRotation)
    {
        clutchStartControllerPosition = controllerPosition;
        clutchStartControllerRotation = controllerRotation;
        clutchStartTargetPosition = transform.position;
        clutchStartTargetRotation = transform.rotation;
        desiredPosition = transform.position;
        desiredRotation = transform.rotation;
        targetVelocity = Vector3.zero;
        filteredControllerPosition = controllerPosition;
        filteredControllerRotation = controllerRotation;
        hasFilteredControllerPose = true;
    }

    private void UpdateDesiredTarget(Vector3 controllerPosition, Quaternion controllerRotation)
    {
        FilterControllerPose(ref controllerPosition, ref controllerRotation);

        Vector3 controllerDelta = controllerPosition - clutchStartControllerPosition;
        if (controllerDelta.magnitude < translationDeadbandMeters)
        {
            controllerDelta = Vector3.zero;
        }

        desiredPosition = clutchStartTargetPosition + controllerDelta * translationScale;

        if (clampWorkspace)
        {
            desiredPosition = new Vector3(
                Mathf.Clamp(desiredPosition.x, minPosition.x, maxPosition.x),
                Mathf.Clamp(desiredPosition.y, minPosition.y, maxPosition.y),
                Mathf.Clamp(desiredPosition.z, minPosition.z, maxPosition.z));
        }

        if (followControllerRotation)
        {
            Quaternion controllerDeltaRotation = controllerRotation * Quaternion.Inverse(clutchStartControllerRotation);
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
            desiredRotation = scaledRotation * clutchStartTargetRotation;
        }
    }

    private void FilterControllerPose(ref Vector3 controllerPosition, ref Quaternion controllerRotation)
    {
        if (!filterControllerPose)
        {
            return;
        }

        if (!hasFilteredControllerPose)
        {
            filteredControllerPosition = controllerPosition;
            filteredControllerRotation = controllerRotation;
            hasFilteredControllerPose = true;
            return;
        }

        float deltaTime = Mathf.Max(Time.deltaTime, 0.0001f);
        if (Vector3.Distance(filteredControllerPosition, controllerPosition) > controllerPositionJitterDeadbandMeters)
        {
            float positionT = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, controllerPositionFilterSharpness) * deltaTime);
            filteredControllerPosition = Vector3.Lerp(filteredControllerPosition, controllerPosition, positionT);
        }

        if (Quaternion.Angle(filteredControllerRotation, controllerRotation) > controllerRotationJitterDeadbandDegrees)
        {
            float rotationT = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, controllerRotationFilterSharpness) * deltaTime);
            filteredControllerRotation = Quaternion.Slerp(filteredControllerRotation, controllerRotation, rotationT);
        }

        controllerPosition = filteredControllerPosition;
        controllerRotation = filteredControllerRotation;
    }

    private void HoldCurrentTargetPose()
    {
        desiredPosition = transform.position;
        desiredRotation = transform.rotation;
        targetVelocity = Vector3.zero;
        hasFilteredControllerPose = false;
    }

    private void ApplyTarget()
    {
        float deltaTime = Mathf.Max(Time.deltaTime, 0.0001f);
        Vector3 positionError = desiredPosition - transform.position;
        Vector3 desiredVelocity = positionError * positionSmoothing;
        if (maximumTargetSpeed > 0.0f)
        {
            desiredVelocity = Vector3.ClampMagnitude(desiredVelocity, maximumTargetSpeed);
        }

        targetVelocity = Vector3.MoveTowards(
            targetVelocity,
            desiredVelocity,
            Mathf.Max(0.0f, maximumTargetAcceleration) * deltaTime);
        Vector3 nextPosition = transform.position + targetVelocity * deltaTime;
        if (Vector3.Dot(desiredPosition - transform.position, desiredPosition - nextPosition) <= 0.0f)
        {
            nextPosition = desiredPosition;
            targetVelocity = Vector3.zero;
        }

        transform.position = nextPosition;

        if (followControllerRotation)
        {
            float exponentialStep = Quaternion.Angle(transform.rotation, desiredRotation)
                * (1.0f - Mathf.Exp(-rotationSmoothing * deltaTime));
            float maxStep = maximumTargetAngularSpeed > 0.0f
                ? maximumTargetAngularSpeed * deltaTime
                : float.PositiveInfinity;
            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                desiredRotation,
                Mathf.Min(exponentialStep, maxStep));
        }
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

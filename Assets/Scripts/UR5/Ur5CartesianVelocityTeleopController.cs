using UnityEngine;
using UnityEngine.XR;

[DefaultExecutionOrder(-90)]
public class Ur5CartesianVelocityTeleopController : MonoBehaviour
{
    public enum UnityPreviewMode
    {
        RelativePoseTarget,
        VelocityIntegration
    }

    public enum RotationInputMode
    {
        Joystick,
        ControllerPoseDelta,
        Locked
    }

    [Header("References")]
    public Transform robotBaseFrame;
    public Transform xrOrigin;
    public Transform tcpPreviewTarget;
    public TcpTargetWorkspaceLimiter workspaceLimiter;
    public Ur5UrScriptSpeedlClient speedlClient;

    [Header("Controllers")]
    public XRNode positionControllerNode = XRNode.RightHand;
    public XRNode rotationControllerNode = XRNode.RightHand;
    public bool usePositionGripAsDeadman = true;
    public bool useRotationGripAsDeadman = true;
    [Tooltip("Locked is the pick-and-place default: translation is tracked while the TCP attitude is held. Joystick and controller-pose modes are optional Inspector-only modes for special tasks.")]
    public RotationInputMode rotationInputMode = RotationInputMode.Locked;

    [Header("Right Controller Position Stabilization")]
    [Tooltip("Filters millimetre-level Quest controller jitter before it can move the TCP target. This affects only the right-hand Cartesian command, never joint-state recording.")]
    public bool filterControllerPosition = true;
    [Tooltip("Small controller-position changes below this radius are treated as tracking noise.")]
    public float controllerPositionJitterDeadbandMeters = 0.0025f;
    [Tooltip("Higher values respond faster to intentional hand motion; 16 keeps normal movement responsive while removing idle jitter.")]
    public float controllerPositionFilterSharpness = 16.0f;

    [Header("Velocity Mapping")]
    [Tooltip("Meters/second generated per meter of right-hand displacement from the clutch origin.")]
    public float linearSpeedGain = 1.20f;
    public float maxLinearSpeed = 0.14f;
    public float linearDeadbandMeters = 0.005f;

    [Tooltip("Radians/second generated per radian of controller rotation when ControllerPoseDelta mode is explicitly selected.")]
    public float angularSpeedGain = 1.30f;
    public float maxAngularSpeedRadiansPerSecond = 4.00f;
    public float angularDeadbandDegrees = 1.2f;

    [Header("Joystick Rotation")]
    [Tooltip("Deadband for the rotation-controller thumbstick.")]
    public float rotationJoystickDeadband = 0.12f;
    [Tooltip("Rotation-controller thumbstick X. Positive turns the TCP around the robot base Y axis.")]
    public float joystickYawSpeedDegreesPerSecond = 220.0f;
    [Tooltip("Rotation-controller thumbstick Y. Positive pitches the TCP around the robot base X axis.")]
    public float joystickPitchSpeedDegreesPerSecond = 180.0f;
    [Tooltip("Hold the rotation-controller secondary button and use stick X for TCP roll around the robot base Z axis.")]
    public bool useSecondaryButtonForJoystickRoll = false;
    public float joystickRollSpeedDegreesPerSecond = 200.0f;
    public bool invertJoystickPitch = false;
    [Tooltip("When the joystick returns to deadband, stop angular preview immediately instead of coasting through the velocity filter.")]
    public bool snapJoystickRotationToZeroInDeadband = true;

    [Header("Axis Locks")]
    public bool allowBaseX = true;
    public bool allowBaseY = true;
    public bool allowBaseZ = true;
    public bool allowAngularX = true;
    public bool allowAngularY = true;
    public bool allowAngularZ = true;

    [Header("Filtering And Limits")]
    public float commandSmoothingSharpness = 22.0f;
    public float maxLinearAcceleration = 0.40f;
    public float maxAngularAcceleration = 10.00f;
    public bool snapToZeroOnRelease = true;

    [Header("Workspace Guard")]
    public bool enableWorkspaceGuard = true;
    public float workspacePredictionHorizonSeconds = 0.35f;
    public bool stopAtWorkspaceBoundary = true;

    [Header("Unity Preview")]
    [Tooltip("RelativePoseTarget is the stable Quest preview mode. VelocityIntegration is useful only for joystick-like speedl tuning.")]
    public UnityPreviewMode unityPreviewMode = UnityPreviewMode.RelativePoseTarget;
    [Tooltip("Allows Unity preview to be tuned before connecting the real robot.")]
    public bool enableUnityPreview = true;
    public bool clampPreviewWithWorkspaceLimiter = true;
    public float relativePreviewPositionScale = 2.40f;
    public float relativePreviewRotationScale = 1.80f;
    public float previewPositionSmoothingSharpness = 26.0f;
    public float previewRotationSmoothingSharpness = 28.0f;
    public float previewMaxLinearSpeed = 0.35f;
    public float previewMaxAngularSpeedDegreesPerSecond = 420.0f;

    [Header("Grasp Axis Assist")]
    [Tooltip("When the physical grasp axis is close to world up/down, snap it exactly vertical while preserving the jaw yaw. This removes small controller-roll errors during top-down grasps.")]
    public bool snapGraspApproachToVertical = true;
    [Range(1.0f, 89.0f)] public float verticalApproachSnapDegrees = 32.0f;

    [Header("Left Controller Safety Pose")]
    [Tooltip("Tap X on the left controller to align the physical grasp axis with world down. Hold it to run the configured ready-pose trajectory.")]
    public bool enableLeftPrimarySnapDown = true;
    [Tooltip("Holding X for this duration starts the full gripper-down ready pose. Releasing X before arrival safely holds the current joint pose.")]
    public bool enableLeftPrimaryReadyPose = true;
    public float leftPrimaryReadyPoseHoldSeconds = 0.45f;
    [Tooltip("Hold Y on the left controller to freeze the current TCP orientation while the right hand translates.")]
    public bool enableLeftSecondaryOrientationHold = true;

    [Header("Actual TCP Lead Limit")]
    [Tooltip("Prevents the IK command target from running far ahead of the real two-pad TCP when the hand moves faster than the arm can track.")]
    public bool limitPreviewLeadToActualTcp = true;
    public float maximumPreviewLeadMeters = 0.025f;

    [Header("Grip Hysteresis")]
    [Range(0.0f, 1.0f)] public float gripPressThreshold = 0.65f;
    [Range(0.0f, 1.0f)] public float gripReleaseThreshold = 0.40f;

    [Header("Fine Control")]
    public bool enableFineControlButton = true;
    public bool applyFineControlToRelativePreview = true;
    [Range(0.1f, 1.0f)] public float fineLinearSpeedMultiplier = 0.25f;
    [Range(0.1f, 1.0f)] public float fineAngularSpeedMultiplier = 0.25f;

    [Header("Output")]
    public bool sendToSpeedlClient = true;
    public bool logDeviceStatus = true;

    private InputDevice positionDevice;
    private InputDevice rotationDevice;
    private bool hasLoggedMissingPositionDevice;
    private bool hasLoggedMissingRotationDevice;
    private bool positionGripLatched;
    private bool rotationGripLatched;
    private bool wasPositionClutched;
    private bool wasRotationClutched;

    private Vector3 positionNeutralWorldPosition;
    private Quaternion rotationNeutralWorldRotation;
    private Vector3 positionClutchStartTargetWorldPosition;
    private Quaternion rotationClutchStartTargetWorldRotation;
    private Quaternion rotationClutchStartGraspWorldRotation;
    private Vector3 filteredControllerPositionWorld;
    private bool hasFilteredControllerPosition;
    private bool controllerPositionFilterIsSettling;
    private Vector3 latestPositionWorld;
    private Quaternion latestRotationWorld = Quaternion.identity;
    private bool latestPositionValid;
    private bool latestRotationValid;
    private Vector2 latestRotationJoystick;
    private bool latestRotationJoystickValid;
    private bool isJoystickRollModifierActive;
    private Vector3 rawBaseLinearVelocity;
    private Vector3 rawBaseAngularVelocity;
    private Vector3 limitedBaseLinearVelocity;
    private Vector3 limitedBaseAngularVelocity;
    private Vector3 filteredBaseLinearVelocity;
    private Vector3 filteredBaseAngularVelocity;
    private Ur5TcpTargetFollower tcpFollower;
    private bool leftPrimaryWasPressed;
    private float leftPrimaryHeldSeconds;
    private bool leftPrimaryReadyPoseWasRequested;
    private bool leftSecondaryWasPressed;
    private bool hasPersistentOrientationTarget;
    private Quaternion persistentOrientationTarget = Quaternion.identity;
    // 右手仅平移时锁定完整工具四元数，不仅锁定“朝下”轴线，
    // 也锁定夹爪两指开合方向，防止底座旋转时夹爪发生偏航自转。
    private bool hasPositionOrientationLock;
    private Quaternion positionOrientationLock = Quaternion.identity;
    // X 长按的 ready pose 会直接将 TCP 同步到机械臂末端。记录其状态边沿，
    // 以避免旧的“夹爪朝下”目标在右手首次平移时被重新应用而造成预转动。
    private bool readyPoseWasActive;

    public bool IsDeviceValid => positionDevice.isValid || rotationDevice.isValid;
    public bool IsPositionClutched { get; private set; }
    public bool IsRotationClutched { get; private set; }
    public bool IsCommandActive => IsPositionClutched || IsRotationClutched;
    public bool IsFineControlActive { get; private set; }
    public bool IsWorkspaceLimited { get; private set; }
    public bool IsInputPoseValid { get; private set; }
    public Vector3 RawBaseLinearVelocity => rawBaseLinearVelocity;
    public Vector3 RawBaseAngularVelocity => rawBaseAngularVelocity;
    public Vector3 BaseLinearVelocity => filteredBaseLinearVelocity;
    public Vector3 BaseAngularVelocity => filteredBaseAngularVelocity;
    public RotationInputMode CurrentRotationInputMode => rotationInputMode;
    public bool IsOrientationLocked => rotationInputMode == RotationInputMode.Locked;
    public bool IsSafetyOrientationHoldActive { get; private set; }
    public bool IsRotationJoystickValid => latestRotationJoystickValid;
    public Vector2 RotationJoystickInput => latestRotationJoystickValid ? latestRotationJoystick : Vector2.zero;
    public bool IsJoystickRollModifierActive => isJoystickRollModifierActive;

    private void Awake()
    {
        ResolveReferences();
    }

    private void Start()
    {
        TryRefreshPositionDevice();
        TryRefreshRotationDevice();
    }

    private void Update()
    {
        ResolveReferences();
        RefreshDevicesIfNeeded();

        bool hasPosition = TryReadControllerPosition(positionDevice, out Vector3 positionWorld);
        if (hasPosition)
        {
            positionWorld = FilterControllerPosition(positionWorld, Time.deltaTime);
        }
        else
        {
            hasFilteredControllerPosition = false;
            controllerPositionFilterIsSettling = false;
        }

        bool hasRotation = TryReadControllerRotation(rotationDevice, out Quaternion rotationWorld);
        bool hasRotationJoystick = TryReadRotationJoystick(rotationDevice, out Vector2 rotationJoystick);
        bool hasRotationInput = rotationInputMode == RotationInputMode.Joystick
            ? hasRotationJoystick
            : rotationInputMode == RotationInputMode.ControllerPoseDelta && hasRotation;
        IsInputPoseValid = hasPosition || hasRotationInput;
        latestPositionValid = hasPosition;
        latestRotationValid = hasRotation;
        latestRotationJoystickValid = hasRotationJoystick;
        if (hasPosition)
        {
            latestPositionWorld = positionWorld;
        }

        if (hasRotation)
        {
            latestRotationWorld = rotationWorld;
        }

        if (hasRotationJoystick)
        {
            latestRotationJoystick = rotationJoystick;
        }

        isJoystickRollModifierActive = IsJoystickRotationMode()
            && useSecondaryButtonForJoystickRoll
            && ReadSecondaryButton(rotationDevice);

        UpdateLeftControllerSafetyPose();
        SynchronizeOrientationAfterReadyPose();

        IsPositionClutched = hasPosition
            && (!usePositionGripAsDeadman || ReadGripDeadman(positionDevice, ref positionGripLatched));
        IsRotationClutched = hasRotationInput
            && (!useRotationGripAsDeadman || ReadGripDeadman(rotationDevice, ref rotationGripLatched));
        if (IsRotationClutched
            && !wasRotationClutched
            && !IsSafetyOrientationHoldActive
            && !leftPrimaryWasPressed)
        {
            hasPersistentOrientationTarget = false;
        }
        IsFineControlActive =
            (IsPositionClutched && ReadFineControl(positionDevice))
            || (IsRotationClutched && ReadFineControl(rotationDevice));

        CaptureClutchOrigins(positionWorld, rotationWorld);
        CalculateRawVelocity(positionWorld, rotationWorld);
        ApplySafetyLimitsAndFiltering(Time.deltaTime);

        wasPositionClutched = IsPositionClutched;
        wasRotationClutched = IsRotationClutched;
    }

    private void FixedUpdate()
    {
        float deltaTime = Mathf.Max(Time.fixedDeltaTime, 0.0001f);
        if (enableUnityPreview)
        {
            ApplyUnityPreview(deltaTime);
        }

        if (sendToSpeedlClient && speedlClient != null)
        {
            speedlClient.SetCommand(
                filteredBaseLinearVelocity,
                filteredBaseAngularVelocity,
                IsCommandActive && IsInputPoseValid);
        }
    }

    private void LateUpdate()
    {
        if (hasPositionOrientationLock
            && IsPositionClutched
            && !IsRotationClutched
            && tcpPreviewTarget != null)
        {
            // 真实 UR 的 speedl 纯平移命令角速度为零。将这一不变量放在
            // LateUpdate 再执行一次，防止场景中的辅助组件或父级变换覆盖 TCP 世界姿态。
            tcpPreviewTarget.rotation = positionOrientationLock;
        }
    }

    private void OnDisable()
    {
        rawBaseLinearVelocity = Vector3.zero;
        rawBaseAngularVelocity = Vector3.zero;
        limitedBaseLinearVelocity = Vector3.zero;
        limitedBaseAngularVelocity = Vector3.zero;
        filteredBaseLinearVelocity = Vector3.zero;
        filteredBaseAngularVelocity = Vector3.zero;
        hasFilteredControllerPosition = false;
        controllerPositionFilterIsSettling = false;

        if (speedlClient != null)
        {
            speedlClient.SetCommand(Vector3.zero, Vector3.zero, false);
        }
    }

    private void RefreshDevicesIfNeeded()
    {
        if (!positionDevice.isValid)
        {
            TryRefreshPositionDevice();
        }

        if (!rotationDevice.isValid)
        {
            TryRefreshRotationDevice();
        }
    }

    private void CaptureClutchOrigins(Vector3 positionWorld, Quaternion rotationWorld)
    {
        if (IsPositionClutched && !wasPositionClutched)
        {
            positionNeutralWorldPosition = positionWorld;
            if (tcpPreviewTarget != null)
            {
                positionClutchStartTargetWorldPosition = tcpPreviewTarget.position;
            }

            if (!IsRotationClutched && tcpFollower != null)
            {
                // 右手 Grip 只负责平移。开始平移的瞬间以实际末端姿态为基准，
                // 防止 X 初始位或上一轮 IK 残留的微小误差先驱动腕关节转动。
                tcpFollower.HoldTargetRotationAtCurrentGraspFrame();
                persistentOrientationTarget = tcpPreviewTarget.rotation;
                hasPersistentOrientationTarget = true;
                positionOrientationLock = tcpPreviewTarget.rotation;
                hasPositionOrientationLock = true;
            }
        }

        if (!IsPositionClutched)
        {
            hasPositionOrientationLock = false;
        }

        if (IsRotationClutched && !wasRotationClutched)
        {
            rotationNeutralWorldRotation = rotationWorld;
            if (tcpPreviewTarget != null)
            {
                rotationClutchStartTargetWorldRotation = tcpPreviewTarget.rotation;
                rotationClutchStartGraspWorldRotation = tcpFollower != null
                    ? tcpFollower.ActualGraspRotation
                    : tcpPreviewTarget.rotation;
            }
        }
    }

    private void CalculateRawVelocity(Vector3 positionWorld, Quaternion rotationWorld)
    {
        rawBaseLinearVelocity = IsPositionClutched
            ? CalculateBaseLinearVelocity(positionWorld)
            : Vector3.zero;
        rawBaseAngularVelocity = IsRotationClutched && !IsSafetyOrientationHoldActive
            ? CalculateBaseAngularVelocity(rotationWorld)
            : Vector3.zero;
    }

    private void ApplySafetyLimitsAndFiltering(float deltaTime)
    {
        limitedBaseLinearVelocity = ApplyAxisLocks(rawBaseLinearVelocity, allowBaseX, allowBaseY, allowBaseZ);
        limitedBaseAngularVelocity = ApplyAxisLocks(rawBaseAngularVelocity, allowAngularX, allowAngularY, allowAngularZ);

        IsWorkspaceLimited = false;
        if (enableWorkspaceGuard)
        {
            limitedBaseLinearVelocity = ApplyWorkspaceGuard(limitedBaseLinearVelocity);
        }

        bool releasedThisFrame = !IsCommandActive && (wasPositionClutched || wasRotationClutched);
        if (snapToZeroOnRelease && releasedThisFrame)
        {
            filteredBaseLinearVelocity = Vector3.zero;
            filteredBaseAngularVelocity = Vector3.zero;
            return;
        }

        bool shouldSnapJoystickRotation =
            IsJoystickRotationMode()
            && snapJoystickRotationToZeroInDeadband
            && rawBaseAngularVelocity.sqrMagnitude < 0.00000001f;

        filteredBaseLinearVelocity = FilterVelocity(
            filteredBaseLinearVelocity,
            limitedBaseLinearVelocity,
            maxLinearAcceleration,
            deltaTime);
        filteredBaseAngularVelocity = FilterVelocity(
            filteredBaseAngularVelocity,
            limitedBaseAngularVelocity,
            maxAngularAcceleration,
            deltaTime);

        if (shouldSnapJoystickRotation)
        {
            filteredBaseAngularVelocity = Vector3.zero;
        }
    }

    private Vector3 CalculateBaseLinearVelocity(Vector3 positionWorld)
    {
        Vector3 worldDelta = positionWorld - positionNeutralWorldPosition;
        Vector3 baseDelta = WorldDirectionToBase(worldDelta);
        Vector3 velocity = ApplyVectorDeadband(baseDelta, linearDeadbandMeters) * Mathf.Max(0.0f, linearSpeedGain);
        float speedLimit = IsFineControlActive
            ? maxLinearSpeed * Mathf.Clamp01(fineLinearSpeedMultiplier)
            : maxLinearSpeed;
        return Vector3.ClampMagnitude(velocity, Mathf.Max(0.0f, speedLimit));
    }

    private Vector3 CalculateBaseAngularVelocity(Quaternion rotationWorld)
    {
        if (IsOrientationLocked)
        {
            return Vector3.zero;
        }

        if (IsJoystickRotationMode())
        {
            return CalculateBaseAngularVelocityFromJoystick();
        }

        Quaternion rotationDelta = rotationWorld * Quaternion.Inverse(rotationNeutralWorldRotation);
        rotationDelta.ToAngleAxis(out float angleDegrees, out Vector3 worldAxis);
        if (angleDegrees > 180.0f)
        {
            angleDegrees -= 360.0f;
        }

        if (worldAxis.sqrMagnitude < 0.000001f || Mathf.Abs(angleDegrees) <= angularDeadbandDegrees)
        {
            return Vector3.zero;
        }

        float signedAngleRadians = Mathf.Sign(angleDegrees)
            * Mathf.Max(0.0f, Mathf.Abs(angleDegrees) - angularDeadbandDegrees)
            * Mathf.Deg2Rad;
        Vector3 baseAxis = WorldDirectionToBase(worldAxis.normalized);
        Vector3 angularVelocity = baseAxis.normalized * signedAngleRadians * Mathf.Max(0.0f, angularSpeedGain);
        float angularSpeedLimit = IsFineControlActive
            ? maxAngularSpeedRadiansPerSecond * Mathf.Clamp01(fineAngularSpeedMultiplier)
            : maxAngularSpeedRadiansPerSecond;
        return Vector3.ClampMagnitude(angularVelocity, Mathf.Max(0.0f, angularSpeedLimit));
    }

    private Vector3 CalculateBaseAngularVelocityFromJoystick()
    {
        Vector2 joystick = latestRotationJoystickValid
            ? ApplyJoystickDeadband(latestRotationJoystick, rotationJoystickDeadband)
            : Vector2.zero;
        if (joystick.sqrMagnitude < 0.000001f)
        {
            return Vector3.zero;
        }

        float fineMultiplier = IsFineControlActive
            ? Mathf.Clamp01(fineAngularSpeedMultiplier)
            : 1.0f;
        float yawRadiansPerSecond = joystick.x * joystickYawSpeedDegreesPerSecond * Mathf.Deg2Rad * fineMultiplier;
        float pitchInput = invertJoystickPitch ? -joystick.y : joystick.y;
        float pitchRadiansPerSecond = pitchInput * joystickPitchSpeedDegreesPerSecond * Mathf.Deg2Rad * fineMultiplier;
        float rollRadiansPerSecond = 0.0f;

        if (isJoystickRollModifierActive)
        {
            rollRadiansPerSecond = joystick.x * joystickRollSpeedDegreesPerSecond * Mathf.Deg2Rad * fineMultiplier;
            yawRadiansPerSecond = 0.0f;
        }

        Vector3 baseAngularVelocity = new Vector3(
            pitchRadiansPerSecond,
            yawRadiansPerSecond,
            rollRadiansPerSecond);
        return Vector3.ClampMagnitude(baseAngularVelocity, Mathf.Max(0.0f, maxAngularSpeedRadiansPerSecond));
    }

    private Vector3 ApplyWorkspaceGuard(Vector3 requestedBaseVelocity)
    {
        if (workspaceLimiter == null || tcpPreviewTarget == null || requestedBaseVelocity.sqrMagnitude < 0.00000001f)
        {
            return requestedBaseVelocity;
        }

        float horizon = Mathf.Max(0.02f, workspacePredictionHorizonSeconds);
        Vector3 currentWorldPosition = tcpPreviewTarget.position;
        Vector3 requestedWorldVelocity = BaseDirectionToWorld(requestedBaseVelocity);
        Vector3 predictedWorldPosition = currentWorldPosition + requestedWorldVelocity * horizon;
        Vector3 clampedWorldPosition = workspaceLimiter.ClampWorldPosition(predictedWorldPosition);

        if ((clampedWorldPosition - predictedWorldPosition).sqrMagnitude < 0.00000001f)
        {
            return requestedBaseVelocity;
        }

        IsWorkspaceLimited = true;
        if (stopAtWorkspaceBoundary)
        {
            Vector3 allowedWorldVelocity = (clampedWorldPosition - currentWorldPosition) / horizon;
            Vector3 allowedBaseVelocity = WorldDirectionToBase(allowedWorldVelocity);
            return LimitVelocityTowardBoundary(requestedBaseVelocity, allowedBaseVelocity);
        }

        return Vector3.zero;
    }

    private Vector3 LimitVelocityTowardBoundary(Vector3 requested, Vector3 allowed)
    {
        return new Vector3(
            LimitAxisTowardBoundary(requested.x, allowed.x),
            LimitAxisTowardBoundary(requested.y, allowed.y),
            LimitAxisTowardBoundary(requested.z, allowed.z));
    }

    private float LimitAxisTowardBoundary(float requested, float allowed)
    {
        if (Mathf.Abs(requested) < 0.000001f)
        {
            return 0.0f;
        }

        if (Mathf.Sign(requested) != Mathf.Sign(allowed))
        {
            return 0.0f;
        }

        return Mathf.Sign(requested) * Mathf.Min(Mathf.Abs(requested), Mathf.Abs(allowed));
    }

    private Vector3 FilterVelocity(
        Vector3 currentVelocity,
        Vector3 targetVelocity,
        float maxAcceleration,
        float deltaTime)
    {
        float t = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, commandSmoothingSharpness) * Mathf.Max(0.0001f, deltaTime));
        Vector3 smoothedTarget = Vector3.Lerp(currentVelocity, targetVelocity, t);
        float accelerationLimit = Mathf.Max(0.0f, maxAcceleration);
        if (accelerationLimit <= 0.0f)
        {
            return smoothedTarget;
        }

        return Vector3.MoveTowards(currentVelocity, smoothedTarget, accelerationLimit * Mathf.Max(0.0001f, deltaTime));
    }

    private void ApplyUnityPreview(float deltaTime)
    {
        if (tcpPreviewTarget == null)
        {
            return;
        }

        if (unityPreviewMode == UnityPreviewMode.RelativePoseTarget)
        {
            ApplyRelativePosePreview(deltaTime);
            return;
        }

        ApplyVelocityIntegrationPreview(deltaTime);
    }

    private void ApplyVelocityIntegrationPreview(float deltaTime)
    {
        Vector3 worldLinearVelocity = BaseDirectionToWorld(filteredBaseLinearVelocity);
        Vector3 nextWorldPosition = tcpPreviewTarget.position + worldLinearVelocity * deltaTime;
        if (clampPreviewWithWorkspaceLimiter && workspaceLimiter != null)
        {
            nextWorldPosition = workspaceLimiter.ClampWorldPosition(nextWorldPosition);
        }

        Quaternion nextWorldRotation = tcpPreviewTarget.rotation;
        Vector3 worldAngularVelocity = BaseDirectionToWorld(filteredBaseAngularVelocity);
        float angularSpeed = worldAngularVelocity.magnitude;
        if (angularSpeed > 0.000001f)
        {
            Quaternion deltaRotation = Quaternion.AngleAxis(
                angularSpeed * Mathf.Rad2Deg * deltaTime,
                worldAngularVelocity.normalized);
            nextWorldRotation = deltaRotation * nextWorldRotation;
        }

        tcpPreviewTarget.SetPositionAndRotation(nextWorldPosition, nextWorldRotation);
    }

    private void ApplyRelativePosePreview(float deltaTime)
    {
        Vector3 desiredPosition = tcpPreviewTarget.position;
        Quaternion desiredRotation = tcpPreviewTarget.rotation;

        if (IsPositionClutched && latestPositionValid)
        {
            Vector3 controllerDelta = latestPositionWorld - positionNeutralWorldPosition;
            Vector3 deadbandedDelta = ApplyVectorDeadband(controllerDelta, linearDeadbandMeters);
            desiredPosition = positionClutchStartTargetWorldPosition
                + deadbandedDelta * GetRelativePreviewPositionScale();

            if (clampPreviewWithWorkspaceLimiter && workspaceLimiter != null)
            {
                desiredPosition = workspaceLimiter.ClampWorldPosition(desiredPosition);
            }

            desiredPosition = LimitPreviewLeadToActualTcp(desiredPosition);
        }

        if (hasPositionOrientationLock && IsPositionClutched && !IsRotationClutched)
        {
            // 右手只允许改变位置；此处保持的是完整旋转，不允许任何偏航自转。
            desiredRotation = positionOrientationLock;
        }
        else if (hasPersistentOrientationTarget)
        {
            desiredRotation = persistentOrientationTarget;
        }
        else if (IsRotationClutched && IsJoystickRotationMode())
        {
            desiredRotation = CalculateJoystickPreviewRotation(tcpPreviewTarget.rotation, deltaTime);
        }
        else if (IsRotationClutched && latestRotationValid)
        {
            desiredRotation = CalculateRelativePreviewRotation(latestRotationWorld);
        }

        float positionBlend = 1.0f - Mathf.Exp(
            -Mathf.Max(0.0f, previewPositionSmoothingSharpness) * Mathf.Max(0.0001f, deltaTime));
        Vector3 blendedPosition = Vector3.Lerp(tcpPreviewTarget.position, desiredPosition, positionBlend);
        float previewSpeed = IsFineControlActive
            ? previewMaxLinearSpeed * Mathf.Clamp01(fineLinearSpeedMultiplier)
            : previewMaxLinearSpeed;
        Vector3 nextPosition = Vector3.MoveTowards(
            tcpPreviewTarget.position,
            blendedPosition,
            Mathf.Max(0.0f, previewSpeed) * Mathf.Max(0.0001f, deltaTime));

        float rotationBlend = 1.0f - Mathf.Exp(
            -Mathf.Max(0.0f, previewRotationSmoothingSharpness) * Mathf.Max(0.0001f, deltaTime));
        Quaternion blendedRotation = Quaternion.Slerp(tcpPreviewTarget.rotation, desiredRotation, rotationBlend);
        Quaternion nextRotation = Quaternion.RotateTowards(
            tcpPreviewTarget.rotation,
            blendedRotation,
            Mathf.Max(0.0f, previewMaxAngularSpeedDegreesPerSecond) * Mathf.Max(0.0001f, deltaTime));

        tcpPreviewTarget.SetPositionAndRotation(nextPosition, nextRotation);
    }

    private Quaternion CalculateRelativePreviewRotation(Quaternion rotationWorld)
    {
        Quaternion rotationDelta = rotationWorld * Quaternion.Inverse(rotationNeutralWorldRotation);
        rotationDelta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (angleDegrees > 180.0f)
        {
            angleDegrees -= 360.0f;
        }

        if (axis.sqrMagnitude < 0.000001f || Mathf.Abs(angleDegrees) <= angularDeadbandDegrees)
        {
            return rotationClutchStartTargetWorldRotation;
        }

        float scaledAngle = Mathf.Sign(angleDegrees)
            * Mathf.Max(0.0f, Mathf.Abs(angleDegrees) - angularDeadbandDegrees)
            * GetRelativePreviewRotationScale();
        Quaternion controllerRotationDelta = Quaternion.AngleAxis(scaledAngle, axis.normalized);
        Quaternion desiredGraspRotation = controllerRotationDelta * rotationClutchStartGraspWorldRotation;
        desiredGraspRotation = SnapGraspApproachToVertical(desiredGraspRotation);
        return tcpFollower != null
            ? tcpFollower.GetToolRotationForGraspRotation(desiredGraspRotation)
            : controllerRotationDelta * rotationClutchStartTargetWorldRotation;
    }

    private Quaternion SnapGraspApproachToVertical(Quaternion desiredGraspRotation)
    {
        if (!snapGraspApproachToVertical)
        {
            return desiredGraspRotation;
        }

        Vector3 approach = desiredGraspRotation * Vector3.forward;
        Vector3 snappedApproach;
        float snapAngle = Mathf.Clamp(verticalApproachSnapDegrees, 1.0f, 89.0f);
        if (Vector3.Angle(approach, Vector3.down) <= snapAngle)
        {
            snappedApproach = Vector3.down;
        }
        else if (Vector3.Angle(approach, Vector3.up) <= snapAngle)
        {
            snappedApproach = Vector3.up;
        }
        else
        {
            return desiredGraspRotation;
        }

        // Keep the current jaw yaw, while removing the unwanted roll/pitch
        // residual that makes a visually vertical gripper look skewed.
        Vector3 jawUp = Vector3.ProjectOnPlane(desiredGraspRotation * Vector3.up, snappedApproach);
        if (jawUp.sqrMagnitude < 0.0001f)
        {
            jawUp = Vector3.ProjectOnPlane(Vector3.forward, snappedApproach);
        }

        return Quaternion.LookRotation(snappedApproach, jawUp.normalized);
    }

    private void UpdateLeftControllerSafetyPose()
    {
        if (rotationControllerNode != XRNode.LeftHand
            || !rotationDevice.isValid
            || tcpPreviewTarget == null)
        {
            if (tcpFollower != null && tcpFollower.IsReadyPoseActive)
            {
                tcpFollower.CancelReadyPose();
            }

            leftPrimaryWasPressed = false;
            leftPrimaryHeldSeconds = 0.0f;
            leftPrimaryReadyPoseWasRequested = false;
            leftSecondaryWasPressed = false;
            IsSafetyOrientationHoldActive = false;
            return;
        }

        bool primaryPressed = enableLeftPrimarySnapDown
            && rotationDevice.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryValue)
            && primaryValue;
        bool secondaryPressed = enableLeftSecondaryOrientationHold
            && rotationDevice.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryValue)
            && secondaryValue;

        if (primaryPressed && !leftPrimaryWasPressed)
        {
            Vector3 yawReference = robotBaseFrame != null ? robotBaseFrame.forward : Vector3.forward;
            persistentOrientationTarget = tcpFollower != null
                ? tcpFollower.GetToolRotationForGraspApproach(Vector3.down, yawReference)
                : tcpPreviewTarget.rotation;
            hasPersistentOrientationTarget = true;
        }

        if (primaryPressed)
        {
            leftPrimaryHeldSeconds += Time.deltaTime;
            if (enableLeftPrimaryReadyPose
                && !leftPrimaryReadyPoseWasRequested
                && leftPrimaryHeldSeconds >= Mathf.Max(0.0f, leftPrimaryReadyPoseHoldSeconds))
            {
                leftPrimaryReadyPoseWasRequested = tcpFollower != null && tcpFollower.BeginReadyPose();
            }
        }
        else
        {
            if (leftPrimaryWasPressed && tcpFollower != null && tcpFollower.IsReadyPoseActive)
            {
                tcpFollower.CancelReadyPose();
            }

            leftPrimaryHeldSeconds = 0.0f;
            leftPrimaryReadyPoseWasRequested = false;
        }

        if (secondaryPressed && !leftSecondaryWasPressed)
        {
            persistentOrientationTarget = tcpPreviewTarget.rotation;
            hasPersistentOrientationTarget = true;
        }

        IsSafetyOrientationHoldActive = secondaryPressed;
        leftPrimaryWasPressed = primaryPressed;
        leftSecondaryWasPressed = secondaryPressed;
    }

    private void SynchronizeOrientationAfterReadyPose()
    {
        bool isReadyPoseActive = tcpFollower != null && tcpFollower.IsReadyPoseActive;
        if (readyPoseWasActive && !isReadyPoseActive && tcpPreviewTarget != null)
        {
            // ready pose 完成或被 X 松开取消时，follower 已把 TcpTarget 对齐到
            // 当前机械臂。同步持久姿态，保证右手 Grip 只产生平移命令。
            persistentOrientationTarget = tcpPreviewTarget.rotation;
            hasPersistentOrientationTarget = true;
        }

        readyPoseWasActive = isReadyPoseActive;
    }

    private Quaternion CalculateJoystickPreviewRotation(Quaternion currentRotation, float deltaTime)
    {
        Vector3 worldAngularVelocity = BaseDirectionToWorld(filteredBaseAngularVelocity);
        float angularSpeed = worldAngularVelocity.magnitude;
        if (angularSpeed < 0.000001f)
        {
            return currentRotation;
        }

        Quaternion deltaRotation = Quaternion.AngleAxis(
            angularSpeed * Mathf.Rad2Deg * Mathf.Max(0.0001f, deltaTime),
            worldAngularVelocity.normalized);
        return deltaRotation * currentRotation;
    }

    private float GetRelativePreviewPositionScale()
    {
        // Never change the position mapping while a grip clutch is held: that
        // would move the target even when the user keeps the hand still.
        return Mathf.Max(0.0f, relativePreviewPositionScale);
    }

    private float GetRelativePreviewRotationScale()
    {
        float scale = Mathf.Max(0.0f, relativePreviewRotationScale);
        return applyFineControlToRelativePreview && IsFineControlActive
            ? scale * Mathf.Clamp01(fineAngularSpeedMultiplier)
            : scale;
    }

    private Vector3 LimitPreviewLeadToActualTcp(Vector3 requestedPosition)
    {
        if (!limitPreviewLeadToActualTcp
            || maximumPreviewLeadMeters <= 0.0f
            || tcpFollower == null
            || !tcpFollower.enabled)
        {
            return requestedPosition;
        }

        Vector3 actualTcpPosition = tcpFollower.ControlPointPosition;
        return actualTcpPosition + Vector3.ClampMagnitude(
            requestedPosition - actualTcpPosition,
            maximumPreviewLeadMeters);
    }

    private Vector3 ApplyAxisLocks(Vector3 value, bool allowX, bool allowY, bool allowZ)
    {
        return new Vector3(
            allowX ? value.x : 0.0f,
            allowY ? value.y : 0.0f,
            allowZ ? value.z : 0.0f);
    }

    private Vector3 ApplyVectorDeadband(Vector3 value, float deadband)
    {
        float magnitude = value.magnitude;
        if (magnitude <= deadband || magnitude < 0.000001f)
        {
            return Vector3.zero;
        }

        return value.normalized * (magnitude - Mathf.Max(0.0f, deadband));
    }

    private Vector3 FilterControllerPosition(Vector3 rawPosition, float deltaTime)
    {
        if (!filterControllerPosition)
        {
            filteredControllerPositionWorld = rawPosition;
            hasFilteredControllerPosition = true;
            controllerPositionFilterIsSettling = false;
            return rawPosition;
        }

        if (!hasFilteredControllerPosition)
        {
            filteredControllerPositionWorld = rawPosition;
            hasFilteredControllerPosition = true;
            controllerPositionFilterIsSettling = false;
            return filteredControllerPositionWorld;
        }

        Vector3 inputDelta = rawPosition - filteredControllerPositionWorld;
        float jitterRadius = Mathf.Max(0.0f, controllerPositionJitterDeadbandMeters);
        if (inputDelta.sqrMagnitude > jitterRadius * jitterRadius)
        {
            controllerPositionFilterIsSettling = true;
        }

        if (!controllerPositionFilterIsSettling)
        {
            // Keep the last accepted pose completely still while the Quest
            // reports its normal idle-position noise.
            return filteredControllerPositionWorld;
        }

        float blend = 1.0f - Mathf.Exp(
            -Mathf.Max(0.0f, controllerPositionFilterSharpness) * Mathf.Max(0.0001f, deltaTime));
        filteredControllerPositionWorld = Vector3.Lerp(
            filteredControllerPositionWorld,
            rawPosition,
            blend);

        // Once a deliberate movement begins, let it converge all the way to
        // the new hand pose. Without this state, a small intentional motion
        // can get stranded part way through the jitter deadband.
        float settleDistance = Mathf.Min(0.00025f, jitterRadius * 0.1f);
        if (inputDelta.sqrMagnitude <= settleDistance * settleDistance)
        {
            filteredControllerPositionWorld = rawPosition;
            controllerPositionFilterIsSettling = false;
        }

        return filteredControllerPositionWorld;
    }

    private Vector2 ApplyJoystickDeadband(Vector2 value, float deadband)
    {
        float magnitude = value.magnitude;
        if (magnitude <= deadband || magnitude < 0.000001f)
        {
            return Vector2.zero;
        }

        float scaledMagnitude = Mathf.InverseLerp(Mathf.Max(0.0f, deadband), 1.0f, magnitude);
        return value.normalized * Mathf.Clamp01(scaledMagnitude);
    }

    private Vector3 WorldDirectionToBase(Vector3 worldDirection)
    {
        return robotBaseFrame != null
            ? robotBaseFrame.InverseTransformDirection(worldDirection)
            : worldDirection;
    }

    private Vector3 BaseDirectionToWorld(Vector3 baseDirection)
    {
        return robotBaseFrame != null
            ? robotBaseFrame.TransformDirection(baseDirection)
            : baseDirection;
    }

    private bool TryReadControllerPosition(InputDevice device, out Vector3 position)
    {
        position = Vector3.zero;
        if (!device.isValid || !device.TryGetFeatureValue(CommonUsages.devicePosition, out position))
        {
            return false;
        }

        if (xrOrigin != null)
        {
            position = xrOrigin.TransformPoint(position);
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

        if (xrOrigin != null)
        {
            rotation = xrOrigin.rotation * rotation;
        }

        return true;
    }

    private bool TryReadRotationJoystick(InputDevice device, out Vector2 joystick)
    {
        joystick = Vector2.zero;
        return device.isValid
            && device.TryGetFeatureValue(CommonUsages.primary2DAxis, out joystick);
    }

    private bool ReadGripDeadman(InputDevice device, ref bool gripLatched)
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

    private bool ReadSecondaryButton(InputDevice device)
    {
        return device.isValid
            && device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryPressed)
            && secondaryPressed;
    }

    private bool IsJoystickRotationMode()
    {
        return rotationInputMode == RotationInputMode.Joystick;
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
                Debug.Log("Quest " + role + " velocity controller connected: " + device.name);
            }
        }
        else if (logDeviceStatus && !hasLoggedMissingDevice)
        {
            hasLoggedMissingDevice = true;
            Debug.LogWarning("Quest " + role + " controller not found for velocity teleop.");
        }
    }

    private void ResolveReferences()
    {
        if (robotBaseFrame == null)
        {
            GameObject foundRobot = GameObject.Find("ur5_robot");
            if (foundRobot == null)
            {
                foundRobot = GameObject.Find("base_link");
            }

            robotBaseFrame = foundRobot != null ? foundRobot.transform : null;
        }

        if (xrOrigin == null)
        {
            GameObject foundOrigin = GameObject.Find("XR Origin (VR)");
            xrOrigin = foundOrigin != null ? foundOrigin.transform : null;
        }

        if (tcpPreviewTarget == null)
        {
            GameObject foundTarget = GameObject.Find("TcpTarget");
            tcpPreviewTarget = foundTarget != null ? foundTarget.transform : null;
        }

        if (workspaceLimiter == null && tcpPreviewTarget != null)
        {
            workspaceLimiter = tcpPreviewTarget.GetComponent<TcpTargetWorkspaceLimiter>();
        }

        if (speedlClient == null)
        {
            speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
        }

        if (tcpFollower == null)
        {
            tcpFollower = FindObjectOfType<Ur5TcpTargetFollower>();
        }
    }

    private void OnValidate()
    {
        linearSpeedGain = Mathf.Max(0.0f, linearSpeedGain);
        maxLinearSpeed = Mathf.Max(0.0f, maxLinearSpeed);
        linearDeadbandMeters = Mathf.Max(0.0f, linearDeadbandMeters);
        controllerPositionJitterDeadbandMeters = Mathf.Max(0.0f, controllerPositionJitterDeadbandMeters);
        controllerPositionFilterSharpness = Mathf.Max(0.0f, controllerPositionFilterSharpness);
        angularSpeedGain = Mathf.Max(0.0f, angularSpeedGain);
        maxAngularSpeedRadiansPerSecond = Mathf.Max(0.0f, maxAngularSpeedRadiansPerSecond);
        angularDeadbandDegrees = Mathf.Max(0.0f, angularDeadbandDegrees);
        rotationJoystickDeadband = Mathf.Clamp01(rotationJoystickDeadband);
        joystickYawSpeedDegreesPerSecond = Mathf.Max(0.0f, joystickYawSpeedDegreesPerSecond);
        joystickPitchSpeedDegreesPerSecond = Mathf.Max(0.0f, joystickPitchSpeedDegreesPerSecond);
        joystickRollSpeedDegreesPerSecond = Mathf.Max(0.0f, joystickRollSpeedDegreesPerSecond);
        commandSmoothingSharpness = Mathf.Max(0.0f, commandSmoothingSharpness);
        maxLinearAcceleration = Mathf.Max(0.0f, maxLinearAcceleration);
        maxAngularAcceleration = Mathf.Max(0.0f, maxAngularAcceleration);
        workspacePredictionHorizonSeconds = Mathf.Max(0.02f, workspacePredictionHorizonSeconds);
        relativePreviewPositionScale = Mathf.Max(0.0f, relativePreviewPositionScale);
        relativePreviewRotationScale = Mathf.Max(0.0f, relativePreviewRotationScale);
        previewPositionSmoothingSharpness = Mathf.Max(0.0f, previewPositionSmoothingSharpness);
        previewRotationSmoothingSharpness = Mathf.Max(0.0f, previewRotationSmoothingSharpness);
        previewMaxLinearSpeed = Mathf.Max(0.0f, previewMaxLinearSpeed);
        previewMaxAngularSpeedDegreesPerSecond = Mathf.Max(0.0f, previewMaxAngularSpeedDegreesPerSecond);
        verticalApproachSnapDegrees = Mathf.Clamp(verticalApproachSnapDegrees, 1.0f, 89.0f);
        leftPrimaryReadyPoseHoldSeconds = Mathf.Max(0.0f, leftPrimaryReadyPoseHoldSeconds);
        maximumPreviewLeadMeters = Mathf.Max(0.0f, maximumPreviewLeadMeters);
        fineLinearSpeedMultiplier = Mathf.Clamp(fineLinearSpeedMultiplier, 0.1f, 1.0f);
        fineAngularSpeedMultiplier = Mathf.Clamp(fineAngularSpeedMultiplier, 0.1f, 1.0f);
        gripPressThreshold = Mathf.Clamp01(gripPressThreshold);
        gripReleaseThreshold = Mathf.Clamp(gripReleaseThreshold, 0.0f, gripPressThreshold);
    }
}

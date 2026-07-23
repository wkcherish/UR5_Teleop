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
    [Tooltip("实验性自适应滤波。默认关闭：它会让手停后仍有滤波尾巴，不适合直接驱动 Unity IK 目标。")]
    public bool useAdaptiveControllerPositionFilter = false;
    [Tooltip("One Euro 静止截止频率（Hz）。较小更稳，较大更灵敏。")]
    public float controllerPositionMinimumCutoffHz = 1.50f;
    [Tooltip("One Euro 随手部速度提高截止频率的系数。")]
    public float controllerPositionFilterBeta = 3.00f;
    public float controllerPositionDerivativeCutoffHz = 1.00f;

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
    [Tooltip("左手摇杆优先使用 X 轴；仅推动 Y 轴时自动复用为同一旋转命令。旋转轴由两夹爪中心的几何对称轴实时计算。")]
    public float joystickYawSpeedDegreesPerSecond = 235.0f;
    [Tooltip("Rotation-controller thumbstick Y. Positive pitches the TCP around the robot base X axis.")]
    public float joystickPitchSpeedDegreesPerSecond = 180.0f;
    [Tooltip("Hold the rotation-controller secondary button and use stick X for TCP roll around the robot base Z axis.")]
    public bool useSecondaryButtonForJoystickRoll = false;
    public float joystickRollSpeedDegreesPerSecond = 200.0f;
    public bool invertJoystickPitch = false;
    [Tooltip("When the joystick returns to deadband, stop angular preview immediately instead of coasting through the velocity filter.")]
    public bool snapJoystickRotationToZeroInDeadband = true;

    [Header("左手 Y + 手柄相对姿态")]
    [Tooltip("左手 Grip + Y 时，以按下瞬间为零点，左右移动或水平转动左手柄可连续调整夹爪偏航。该输入优先于摇杆，松开后保持当前姿态。")]
    public bool enableLeftSecondaryPoseRotation = true;
    [Tooltip("左手 Grip + Y 时，左手相对左右移动一米对应的夹爪偏航角度。仅改变竖直下抓姿态下的一个自由度。")]
    public float leftSecondaryPoseYawDegreesPerMeter = 900.0f;
    [Tooltip("左手 Grip + Y 时，手柄绕竖直方向的相对转动映射到夹爪偏航的比例。")]
    public float leftSecondaryPoseTwistScale = 0.85f;

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
    [Header("连续精度响应")]
    [Tooltip("默认启用：小手部位移采用较低比例以便精密装配；大位移平滑过渡到常规比例。没有额外按键或控制模式。")]
    public bool useProgressivePositionResponse = true;
    [Tooltip("右手靠近 clutch 原点时使用的位置比例，降低目标点附近的过冲。")]
    public float precisionPositionScale = 1.70f;
    [Tooltip("从精密比例过渡到常规比例所需的右手累计位移。")]
    public float progressivePositionTransitionMeters = 0.030f;
    public float relativePreviewRotationScale = 1.80f;
    public float previewPositionSmoothingSharpness = 26.0f;
    public float previewRotationSmoothingSharpness = 28.0f;
    public float previewMaxLinearSpeed = 0.35f;
    public float previewMaxAngularSpeedDegreesPerSecond = 420.0f;
    [Header("Open-Teach 相对位姿命令")]
    [Tooltip("采用 Open-Teach 同类的单级相对位姿滤波。开启后不再叠加 TCP 的第二层指数平滑，减少 Quest 操作中的迟滞与卡顿。")]
    public bool useRelativePoseCommandFilter = true;
    [Tooltip("最终 TCP 命令的上一帧保留比例。0 为完全直通；0.55 在 Quest 追踪噪声与快速响应之间取得平衡。")]
    [Range(0.0f, 0.95f)] public float relativePoseCommandFilterRetention = 0.55f;
    [Tooltip("精细模式使用的 TCP 命令保留比例。略高于常规模式，用于抑制 Quest 微抖，但不会产生松手后的滤波尾巴。")]
    [Range(0.0f, 0.95f)] public float fineRelativePoseCommandFilterRetention = 0.40f;
    [Tooltip("实验性 TCP 加速度轨迹。默认关闭：Unity 预览使用直接限速目标，避免停手后继续追赶。")]
    public bool useAccelerationLimitedPreviewTrajectory = false;
    public float previewMaxLinearAcceleration = 1.60f;
    public float previewMaxAngularAccelerationDegreesPerSecondSquared = 1800.0f;
    [Tooltip("TCP 指令小于该距离时保持上一个目标，避免 Quest 微抖被位置比例放大。")]
    public float previewTargetDeadbandMeters = 0.00025f;
    [Tooltip("兼容旧 Inspector 配置；默认控制已使用相同的精细目标死区。")]
    public float finePreviewTargetDeadbandMeters = 0.00025f;

    [Header("右手停手即停")]
    [Tooltip("实验性手停冻结。默认关闭，避免与 IK 自身的到位保持发生目标切换。")]
    public bool freezeRobotWhenPositionHandStops = false;
    [Tooltip("手部累计位移超过该值才视为新的有意移动。")]
    public float controllerMotionEpsilonMeters = 0.00030f;
    [Tooltip("连续无有意手部移动多久后执行零速度保持。")]
    public float controllerStopHoldSeconds = 0.12f;
    [Tooltip("兼容旧 Inspector 配置；默认控制已使用相同的停止检测阈值。")]
    public float fineControllerMotionEpsilonMeters = 0.00030f;

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
    public float maximumPreviewLeadMeters = 0.060f;

    [Header("Grip Hysteresis")]
    [Range(0.0f, 1.0f)] public float gripPressThreshold = 0.65f;
    [Range(0.0f, 1.0f)] public float gripReleaseThreshold = 0.40f;

    [Header("Legacy Optional Fine Mode")]
    [Tooltip("Compatibility-only thumbstick-click scaling. The standard assembly profile keeps this disabled because precision tracking is always active.")]
    public bool enableFineControlButton = false;
    public bool applyFineControlToRelativePreview = false;
    [Range(0.1f, 1.0f)] public float fineLinearSpeedMultiplier = 1.00f;
    [Range(0.1f, 1.0f)] public float fineAngularSpeedMultiplier = 1.00f;

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
    private bool wasFinePositionControlActive;

    private Vector3 positionNeutralWorldPosition;
    private Quaternion rotationNeutralWorldRotation;
    private Vector3 positionClutchStartTargetWorldPosition;
    private Quaternion rotationClutchStartTargetWorldRotation;
    private Quaternion rotationClutchStartGraspWorldRotation;
    // Y 姿态通道有自己的零点，不能复用摇杆/普通姿态的 clutch，
    // 否则双手切换控制时会把旧的输入基准带入当前目标。
    private Vector3 secondaryPoseNeutralWorldPosition;
    private Quaternion secondaryPoseNeutralWorldRotation = Quaternion.identity;
    private Quaternion secondaryPoseStartGraspWorldRotation = Quaternion.identity;
    private Vector3 filteredControllerPositionWorld;
    private readonly Ur5OneEuroVectorFilter controllerPositionOneEuroFilter = new Ur5OneEuroVectorFilter();
    private readonly Ur5RelativePoseCommandFilter relativePoseCommandFilter = new Ur5RelativePoseCommandFilter();
    private bool hasFilteredControllerPosition;
    private bool controllerPositionFilterIsSettling;
    private Vector3 latestPositionWorld;
    private Quaternion latestRotationWorld = Quaternion.identity;
    private Vector3 latestRotationControllerPositionWorld;
    private bool latestPositionValid;
    private Vector3 latestRawPositionWorld;
    private bool latestRawPositionValid;
    private bool latestRotationValid;
    private bool latestRotationControllerPositionValid;
    private Vector2 latestRotationJoystick;
    private bool latestRotationJoystickValid;
    private bool isJoystickRollModifierActive;
    private Vector3 rawBaseLinearVelocity;
    private Vector3 rawBaseAngularVelocity;
    private Vector3 limitedBaseLinearVelocity;
    private Vector3 limitedBaseAngularVelocity;
    private Vector3 filteredBaseLinearVelocity;
    private Vector3 filteredBaseAngularVelocity;
    private Vector3 previewLinearVelocity;
    private Vector3 previewAngularVelocityDegrees;
    private bool hasStabilizedPreviewPosition;
    private Vector3 stabilizedPreviewPosition;
    private Vector3 positionMotionReferenceWorld;
    private bool hasPositionMotionReference;
    private float positionHandStillSeconds;
    private bool positionHandStopHoldActive;
    private Ur5TcpTargetFollower tcpFollower;
    private bool leftPrimaryWasPressed;
    private float leftPrimaryHeldSeconds;
    private bool leftPrimaryReadyPoseWasRequested;
    private bool leftSecondaryWasPressed;
    private bool isSecondaryPoseRotationActive;
    private bool wasSecondaryPoseRotationActive;
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
    // 左手仅握住 Grip、摇杆居中时不应让 IK 继续追赶旧姿态目标；只有摇杆
    // 真正离开死区后才视为旋转命令。右手平移始终保持实时控制。
    public bool IsCommandActive => IsPositionClutched || IsRotationCommandActive;
    public bool IsFineControlActive { get; private set; }
    /// <summary>右手 Grip + 右摇杆按下时的无跳变精细平移模式。</summary>
    public bool IsFinePositionControlActive { get; private set; }
    public bool IsWorkspaceLimited { get; private set; }
    public bool IsInputPoseValid { get; private set; }
    public Vector3 RawBaseLinearVelocity => rawBaseLinearVelocity;
    public Vector3 RawBaseAngularVelocity => rawBaseAngularVelocity;
    public Vector3 BaseLinearVelocity => filteredBaseLinearVelocity;
    public Vector3 BaseAngularVelocity => filteredBaseAngularVelocity;
    public RotationInputMode CurrentRotationInputMode => rotationInputMode;
    public bool IsOrientationLocked => rotationInputMode == RotationInputMode.Locked;
    /// <summary>右手 Grip 单独平移时是否已捕获完整 TCP 姿态锁。</summary>
    public bool IsPositionOrientationLocked => hasPositionOrientationLock
        && IsPositionClutched
        && !IsRotationClutched;
    public bool IsRotationCommandActive => IsRotationClutched
        && (isSecondaryPoseRotationActive
            || !IsJoystickRotationMode()
            || (latestRotationJoystickValid
                && ApplyJoystickDeadband(latestRotationJoystick, rotationJoystickDeadband).sqrMagnitude > 0.000001f));
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
            latestRawPositionWorld = positionWorld;
            latestRawPositionValid = true;
            positionWorld = FilterControllerPosition(positionWorld, Time.deltaTime);
        }
        else
        {
            hasFilteredControllerPosition = false;
            controllerPositionFilterIsSettling = false;
            latestRawPositionValid = false;
        }

        bool hasRotation = TryReadControllerRotation(rotationDevice, out Quaternion rotationWorld);
        // 左手 Y 姿态通道读取的是左手的相对位置和水平转角；右手位置仍然
        // 完全独立地负责 TCP 平移，因此双手同时使用不会相互覆盖。
        bool hasRotationControllerPosition = TryReadControllerPosition(
            rotationDevice,
            out Vector3 rotationControllerPositionWorld);
        bool hasRotationJoystick = TryReadRotationJoystick(rotationDevice, out Vector2 rotationJoystick);
        // 左手仅握住 Grip、摇杆居中时不接管姿态。这样右手仍可稳定地保持
        // 当前 TCP 姿态；只有明确推动摇杆后才进入绕基座 Y 轴的旋转控制。
        bool hasJoystickRotationCommand = hasRotationJoystick
            && ApplyJoystickDeadband(rotationJoystick, rotationJoystickDeadband).sqrMagnitude > 0.000001f;
        // Y + 左 Grip 的相对姿态输入与摇杆属于同一个姿态通道，但 Y 优先。
        // 它必须在抓取后才真正生效；这里先声明设备输入可用，随后由 Grip
        // deadman 决定是否进入 clutch。
        bool hasSecondaryPoseRotationCommand = enableLeftSecondaryPoseRotation
            && hasRotation
            && hasRotationControllerPosition
            && ReadSecondaryButton(rotationDevice);
        bool hasRotationInput = hasSecondaryPoseRotationCommand
            || (rotationInputMode == RotationInputMode.Joystick
                ? hasJoystickRotationCommand
                : rotationInputMode == RotationInputMode.ControllerPoseDelta && hasRotation);
        IsInputPoseValid = hasPosition || hasRotationInput;
        latestPositionValid = hasPosition;
        latestRotationValid = hasRotation;
        latestRotationControllerPositionValid = hasRotationControllerPosition;
        latestRotationJoystickValid = hasRotationJoystick;
        if (hasPosition)
        {
            latestPositionWorld = positionWorld;
        }

        if (hasRotation)
        {
            latestRotationWorld = rotationWorld;
        }

        if (hasRotationControllerPosition)
        {
            latestRotationControllerPositionWorld = rotationControllerPositionWorld;
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
        isSecondaryPoseRotationActive = IsRotationClutched && hasSecondaryPoseRotationCommand;

        // 输入源切换时始终以当前 TCP 重新建 clutch，避免摇杆与手柄相对姿态
        // 同时使用时把上一种输入的历史零点带入，造成姿态跳变或扰动。
        if (IsRotationClutched
            && isSecondaryPoseRotationActive != wasSecondaryPoseRotationActive
            && latestRotationValid
            && latestRotationControllerPositionValid
            && tcpPreviewTarget != null)
        {
            rotationNeutralWorldRotation = latestRotationWorld;
            rotationClutchStartTargetWorldRotation = tcpPreviewTarget.rotation;
            rotationClutchStartGraspWorldRotation = tcpFollower != null
                ? tcpFollower.ActualGraspRotation
                : tcpPreviewTarget.rotation;
            secondaryPoseNeutralWorldPosition = latestRotationControllerPositionWorld;
            secondaryPoseNeutralWorldRotation = latestRotationWorld;
            secondaryPoseStartGraspWorldRotation = GetPreviewGraspRotation(tcpPreviewTarget.rotation);
            relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
        }
        if (IsRotationClutched
            && !wasRotationClutched
            && !IsSafetyOrientationHoldActive
            && !leftPrimaryWasPressed)
        {
            hasPersistentOrientationTarget = false;
        }
        else if (!IsRotationClutched && wasRotationClutched && tcpPreviewTarget != null)
        {
            // 松开左摇杆时将刚刚达到的姿态保存为下一轮右手平移的保持姿态，
            // 避免因回到旧的 Grip 起始旋转而产生夹爪“自动复原”。
            persistentOrientationTarget = tcpPreviewTarget.rotation;
            hasPersistentOrientationTarget = true;
            if (IsPositionClutched)
            {
                positionOrientationLock = persistentOrientationTarget;
                hasPositionOrientationLock = true;
            }
        }
        IsFinePositionControlActive = IsPositionClutched && ReadFineControl(positionDevice);
        IsFineControlActive =
            IsFinePositionControlActive
            || (IsRotationClutched && ReadFineControl(rotationDevice));

        CaptureClutchOrigins(positionWorld, rotationWorld);
        RebasePositionClutchForFineControl(positionWorld);
        UpdatePositionHandStopHold(Time.deltaTime);
        CalculateRawVelocity(positionWorld, rotationWorld);
        ApplySafetyLimitsAndFiltering(Time.deltaTime);

        wasPositionClutched = IsPositionClutched;
        wasRotationClutched = IsRotationClutched;
        wasSecondaryPoseRotationActive = isSecondaryPoseRotationActive;
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
        previewLinearVelocity = Vector3.zero;
        previewAngularVelocityDegrees = Vector3.zero;
        hasStabilizedPreviewPosition = false;
        hasFilteredControllerPosition = false;
        controllerPositionFilterIsSettling = false;
        relativePoseCommandFilter.Clear();

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
                stabilizedPreviewPosition = tcpPreviewTarget.position;
                hasStabilizedPreviewPosition = true;
                // 离合按下时同时重置相对位姿命令滤波。与 Open-Teach 的
                // hand_init_H / robot_init_H 成对重建相同，避免从旧命令追赶。
                relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
            }
            positionMotionReferenceWorld = latestRawPositionWorld;
            hasPositionMotionReference = latestRawPositionValid;
            positionHandStillSeconds = 0.0f;
            positionHandStopHoldActive = false;

            if (!IsRotationClutched && tcpFollower != null)
            {
                // 右手 Grip 只负责平移。若 X 初始位或左手偏航已经保存了一个
                // “夹爪朝下”的目标，就必须继续使用它，不能用实际末端的微小
                // 机械误差覆盖它，否则右手向下抓取时会逐渐翘起。
                if (!hasPersistentOrientationTarget
                    || !IsDownwardGraspTarget(persistentOrientationTarget))
                {
                    tcpFollower.HoldTargetRotationAtCurrentGraspFrame();
                    persistentOrientationTarget = tcpPreviewTarget.rotation;
                    hasPersistentOrientationTarget = true;
                }

                positionOrientationLock = persistentOrientationTarget;
                hasPositionOrientationLock = true;
            }
        }

        if (!IsPositionClutched)
        {
            hasPositionOrientationLock = false;
            wasFinePositionControlActive = false;
            hasStabilizedPreviewPosition = false;
            hasPositionMotionReference = false;
            positionHandStillSeconds = 0.0f;
            positionHandStopHoldActive = false;
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
                relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
            }
        }
    }

    private void CalculateRawVelocity(Vector3 positionWorld, Quaternion rotationWorld)
    {
        rawBaseLinearVelocity = IsPositionClutched
            ? CalculateBaseLinearVelocity(positionWorld)
            : Vector3.zero;
        rawBaseAngularVelocity = IsRotationClutched
            && !IsSafetyOrientationHoldActive
            && !isSecondaryPoseRotationActive
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
        float yawRadiansPerSecond = GetJoystickYawInput(joystick)
            * joystickYawSpeedDegreesPerSecond * Mathf.Deg2Rad * fineMultiplier;
        float pitchInput = invertJoystickPitch ? -joystick.y : joystick.y;
        float pitchRadiansPerSecond = pitchInput * joystickPitchSpeedDegreesPerSecond * Mathf.Deg2Rad * fineMultiplier;
        float rollRadiansPerSecond = 0.0f;

        if (isJoystickRollModifierActive)
        {
            rollRadiansPerSecond = joystick.x * joystickRollSpeedDegreesPerSecond * Mathf.Deg2Rad * fineMultiplier;
            yawRadiansPerSecond = 0.0f;
        }

        Vector3 graspAxisWorld = GetGripperCenterSymmetryAxisWorld();
        Vector3 baseAngularVelocity = WorldDirectionToBase(
            graspAxisWorld * yawRadiansPerSecond
            + (robotBaseFrame != null ? robotBaseFrame.right : Vector3.right) * pitchRadiansPerSecond
            + (robotBaseFrame != null ? robotBaseFrame.forward : Vector3.forward) * rollRadiansPerSecond);
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
                + ApplyProgressivePositionResponse(deadbandedDelta);

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
        else if (IsRotationClutched
            && isSecondaryPoseRotationActive
            && latestRotationValid
            && latestRotationControllerPositionValid)
        {
            // 左手 Grip + Y：仅写竖直下抓姿态中的偏航；右手同时 Grip 时位置仍由右手写入。
            desiredRotation = CalculateSecondaryPoseYawRotation();
        }
        else if (IsRotationClutched && IsJoystickRotationMode())
        {
            // 双手叠加时，右手只写位置、左手只写姿态；两者最终在同一个
            // TCP 位姿中合成，不能让旧的持久姿态覆盖左摇杆的当帧指令。
            desiredRotation = CalculateJoystickPreviewRotation(tcpPreviewTarget.rotation, deltaTime);
        }
        else if (IsRotationClutched && latestRotationValid)
        {
            desiredRotation = CalculateRelativePreviewRotation(latestRotationWorld);
        }
        else if (hasPersistentOrientationTarget)
        {
            desiredRotation = persistentOrientationTarget;
        }

        Vector3 nextPosition;
        Quaternion nextRotation;
        if (useRelativePoseCommandFilter && IsCommandActive)
        {
            // Open-Teach 同类策略：先由相对离合位姿得到唯一 TCP 命令，再只在
            // 这一层做位置 Lerp / 姿态 Slerp。下方 MoveTowards/RotateTowards 仅是
            // 防止追踪丢帧时单帧跳跃过大，并非第二个低通滤波器。
            relativePoseCommandFilter.Filter(
                desiredPosition,
                desiredRotation,
                GetRelativePoseCommandFilterRetention(),
                out Vector3 filteredPosition,
                out Quaternion filteredRotation);
            nextPosition = Vector3.MoveTowards(
                tcpPreviewTarget.position,
                filteredPosition,
                Mathf.Max(0.0f, previewMaxLinearSpeed) * deltaTime);
            nextRotation = Quaternion.RotateTowards(
                tcpPreviewTarget.rotation,
                filteredRotation,
                Mathf.Max(0.0f, previewMaxAngularSpeedDegreesPerSecond) * deltaTime);
        }
        else
        {
            // 松开 Grip 后清空滤波状态；下一次按下会重新建立相对零点，而不会
            // 继续追赶上一次的滤波尾巴。
            relativePoseCommandFilter.Reset(desiredPosition, desiredRotation);
            nextPosition = desiredPosition;
            nextRotation = desiredRotation;
        }

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

        float rotationScale = GetRelativePreviewRotationScale();
        float scaledAngle = Mathf.Sign(angleDegrees)
            * Mathf.Max(0.0f, Mathf.Abs(angleDegrees) - angularDeadbandDegrees)
            * rotationScale;
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

        // X/Y 属于左手安全控制，必须同时满足左手 Grip，不能被偶然按键触发。
        bool leftGripHeld = !useRotationGripAsDeadman
            || ReadGripDeadman(rotationDevice, ref rotationGripLatched);
        bool primaryPressed = leftGripHeld
            && enableLeftPrimarySnapDown
            && rotationDevice.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryValue)
            && primaryValue;
        bool secondaryPressed = leftGripHeld
            && enableLeftSecondaryOrientationHold
            && !enableLeftSecondaryPoseRotation
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
            relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
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
            relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
        }

        readyPoseWasActive = isReadyPoseActive;
    }

    private Quaternion CalculateJoystickPreviewRotation(Quaternion currentRotation, float deltaTime)
    {
        Vector2 joystick = latestRotationJoystickValid
            ? ApplyJoystickDeadband(latestRotationJoystick, rotationJoystickDeadband)
            : Vector2.zero;
        float yawInput = GetJoystickYawInput(joystick);
        if (Mathf.Abs(yawInput) < 0.000001f)
        {
            return currentRotation;
        }

        // Use the physical symmetry axis through the two-pad midpoint rather
        // than an imported tool0 or controller-local axis.
        float localYDegrees = yawInput
            * Mathf.Max(0.0f, joystickYawSpeedDegreesPerSecond)
            * Mathf.Max(0.0001f, deltaTime);
        Quaternion currentGraspRotation = GetPreviewGraspRotation(currentRotation);
        Vector3 symmetryAxisWorld = GetGripperCenterSymmetryAxisWorld(currentGraspRotation);
        Quaternion localYDelta = Quaternion.AngleAxis(localYDegrees, symmetryAxisWorld);
        Quaternion desiredGraspRotation = localYDelta * currentGraspRotation;
        return tcpFollower != null
            ? tcpFollower.GetToolRotationForGraspRotation(desiredGraspRotation)
            : localYDelta * currentRotation;
    }

    /// <summary>
    /// 左手 Grip + Y 的连续偏航输入。
    /// 左右移动和手柄绕竖直轴的转动都会累加到同一个抓取偏航角，便于操作者
    /// 根据姿势选择更自然的操作方式；无论哪种输入，夹爪接近方向始终是世界向下。
    /// </summary>
    private Quaternion CalculateSecondaryPoseYawRotation()
    {
        Vector3 worldDisplacement = latestRotationControllerPositionWorld - secondaryPoseNeutralWorldPosition;
        Vector3 baseDisplacement = WorldDirectionToBase(worldDisplacement);
        float translationYawDegrees = baseDisplacement.x
            * Mathf.Max(0.0f, leftSecondaryPoseYawDegreesPerMeter);

        float twistYawDegrees = CalculateHorizontalControllerTwistDegrees(
            secondaryPoseNeutralWorldRotation,
            latestRotationWorld)
            * Mathf.Max(0.0f, leftSecondaryPoseTwistScale);
        return BuildDownwardGraspYawRotation(
            secondaryPoseStartGraspWorldRotation,
            translationYawDegrees + twistYawDegrees);
    }

    /// <summary>
    /// 只提取控制器在水平面内的转动，忽略手腕俯仰/翻转的追踪误差。
    /// 这使 Grip + Y 的手柄转动不会破坏“夹爪竖直向下”的不变量。
    /// </summary>
    private float CalculateHorizontalControllerTwistDegrees(Quaternion startRotation, Quaternion currentRotation)
    {
        Vector3 startForward = Vector3.ProjectOnPlane(startRotation * Vector3.forward, Vector3.down);
        Vector3 currentForward = Vector3.ProjectOnPlane(currentRotation * Vector3.forward, Vector3.down);
        if (startForward.sqrMagnitude < 0.0001f || currentForward.sqrMagnitude < 0.0001f)
        {
            return 0.0f;
        }

        return Vector3.SignedAngle(startForward, currentForward, Vector3.down);
    }

    /// <summary>
    /// 从参考抓取姿态取夹爪两指方向，并仅绕竖直下方的抓取轴旋转。
    /// 通过 follower 的抓取坐标系转换回 tool0，避免依赖 URDF 中不直观的局部轴定义。
    /// </summary>
    private Quaternion BuildDownwardGraspYawRotation(Quaternion referenceGraspRotation, float yawDegrees)
    {
        Vector3 yawReference = Vector3.ProjectOnPlane(referenceGraspRotation * Vector3.up, Vector3.down);
        if (yawReference.sqrMagnitude < 0.0001f)
        {
            yawReference = robotBaseFrame != null
                ? Vector3.ProjectOnPlane(robotBaseFrame.forward, Vector3.down)
                : Vector3.forward;
        }

        Vector3 rotatedYawReference = Quaternion.AngleAxis(yawDegrees, Vector3.down) * yawReference.normalized;
        return tcpFollower != null
            ? tcpFollower.GetToolRotationForGraspApproach(Vector3.down, rotatedYawReference)
            : Quaternion.LookRotation(Vector3.down, rotatedYawReference);
    }

    private Quaternion GetPreviewGraspRotation(Quaternion toolRotation)
    {
        return tcpFollower != null
            ? tcpFollower.GetGraspRotationForToolRotation(toolRotation)
            : toolRotation;
    }

    private bool IsDownwardGraspTarget(Quaternion toolRotation)
    {
        Vector3 approach = GetPreviewGraspRotation(toolRotation) * Vector3.forward;
        return Vector3.Angle(approach, Vector3.down)
            <= Mathf.Clamp(verticalApproachSnapDegrees, 1.0f, 89.0f);
    }

    private float GetRelativePreviewPositionScale()
    {
        float scale = Mathf.Max(0.0f, relativePreviewPositionScale);
        return applyFineControlToRelativePreview && IsFinePositionControlActive
            ? scale * Mathf.Clamp01(fineLinearSpeedMultiplier)
            : scale;
    }

    private float GetRelativePoseCommandFilterRetention()
    {
        return IsFineControlActive
            ? Mathf.Clamp(fineRelativePoseCommandFilterRetention, 0.0f, 0.95f)
            : Mathf.Clamp(relativePoseCommandFilterRetention, 0.0f, 0.95f);
    }

    /// <summary>
    /// 将同一个相对 clutch 输入连续映射为“近处精密、远处常规”的位移。
    /// 这是连续函数而非精细模式：不会重置原点、不需要额外按钮，也不会在
    /// 左右手同时操作时产生档位切换扰动。
    /// </summary>
    private Vector3 ApplyProgressivePositionResponse(Vector3 deadbandedDelta)
    {
        float normalScale = GetRelativePreviewPositionScale();
        if (!useProgressivePositionResponse || deadbandedDelta.sqrMagnitude < 0.0000000001f)
        {
            return deadbandedDelta * normalScale;
        }

        float transition = Mathf.Max(0.0001f, progressivePositionTransitionMeters);
        float progress = Mathf.Clamp01(deadbandedDelta.magnitude / transition);
        // SmoothStep 保证过渡端点的一阶导数为零，避免穿过临界距离时手感突变。
        progress = progress * progress * (3.0f - 2.0f * progress);
        float nearScale = Mathf.Min(Mathf.Max(0.0f, precisionPositionScale), normalScale);
        float scale = Mathf.Lerp(nearScale, normalScale, progress);
        return deadbandedDelta * scale;
    }

    /// <summary>
    /// 精细模式切换时重新捕获右手与 TCP 的相对基准。这样按下或松开右摇杆
    /// 不会改变当前目标点，只会改变之后手部位移的分辨率。
    /// </summary>
    private void RebasePositionClutchForFineControl(Vector3 positionWorld)
    {
        if (!IsPositionClutched || !latestPositionValid)
        {
            wasFinePositionControlActive = false;
            return;
        }

        if (IsFinePositionControlActive == wasFinePositionControlActive)
        {
            return;
        }

        positionNeutralWorldPosition = positionWorld;
        if (tcpPreviewTarget != null)
        {
            positionClutchStartTargetWorldPosition = tcpPreviewTarget.position;
            stabilizedPreviewPosition = tcpPreviewTarget.position;
            hasStabilizedPreviewPosition = true;
        }

        positionMotionReferenceWorld = latestRawPositionWorld;
        hasPositionMotionReference = latestRawPositionValid;
        positionHandStillSeconds = 0.0f;
        positionHandStopHoldActive = false;

        wasFinePositionControlActive = IsFinePositionControlActive;
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
            Mathf.Max(0.0f, maximumPreviewLeadMeters));
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
            controllerPositionOneEuroFilter.Reset(rawPosition);
            hasFilteredControllerPosition = true;
            controllerPositionFilterIsSettling = false;
            return rawPosition;
        }

        if (!hasFilteredControllerPosition)
        {
            filteredControllerPositionWorld = rawPosition;
            controllerPositionOneEuroFilter.Reset(rawPosition);
            hasFilteredControllerPosition = true;
            controllerPositionFilterIsSettling = false;
            return filteredControllerPositionWorld;
        }

        if (useAdaptiveControllerPositionFilter)
        {
            filteredControllerPositionWorld = controllerPositionOneEuroFilter.Filter(
                rawPosition,
                deltaTime,
                controllerPositionMinimumCutoffHz,
                controllerPositionFilterBeta,
                controllerPositionDerivativeCutoffHz);
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

    private void UpdatePositionHandStopHold(float deltaTime)
    {
        if (!freezeRobotWhenPositionHandStops
            || !IsPositionClutched
            || !latestRawPositionValid
            || tcpPreviewTarget == null
            || tcpFollower == null)
        {
            return;
        }

        if (!hasPositionMotionReference)
        {
            positionMotionReferenceWorld = latestRawPositionWorld;
            hasPositionMotionReference = true;
            positionHandStillSeconds = 0.0f;
            return;
        }

        float motionEpsilon = controllerMotionEpsilonMeters;
        if (Vector3.Distance(positionMotionReferenceWorld, latestRawPositionWorld)
            >= Mathf.Max(0.0f, motionEpsilon))
        {
            // 使用累计位移而非单帧速度判断，慢速精细移动同样能持续生效。
            positionMotionReferenceWorld = latestRawPositionWorld;
            positionHandStillSeconds = 0.0f;
            positionHandStopHoldActive = false;
            return;
        }

        positionHandStillSeconds += Mathf.Max(0.0f, deltaTime);
        if (positionHandStopHoldActive
            || positionHandStillSeconds < Mathf.Max(0.0f, controllerStopHoldSeconds))
        {
            return;
        }

        // 与真实 speedl 的零 Twist 命令相同：停止手部输入时不再完成旧的目标轨迹，
        // 而是保持当前实际 TCP。下一次有意移动会重新建立 clutch 原点。
        tcpFollower.FreezeAtCurrentPose();
        controllerPositionOneEuroFilter.Reset(latestRawPositionWorld);
        positionNeutralWorldPosition = latestRawPositionWorld;
        positionClutchStartTargetWorldPosition = tcpPreviewTarget.position;
        stabilizedPreviewPosition = tcpPreviewTarget.position;
        hasStabilizedPreviewPosition = true;
        previewLinearVelocity = Vector3.zero;
        previewAngularVelocityDegrees = Vector3.zero;
        positionMotionReferenceWorld = latestRawPositionWorld;
        positionHandStopHoldActive = true;
    }

    private Vector3 StabilizePreviewPositionTarget(Vector3 requestedPosition)
    {
        if (!hasStabilizedPreviewPosition)
        {
            stabilizedPreviewPosition = requestedPosition;
            hasStabilizedPreviewPosition = true;
            return stabilizedPreviewPosition;
        }

        float deadband = previewTargetDeadbandMeters;
        if (Vector3.Distance(stabilizedPreviewPosition, requestedPosition) >= Mathf.Max(0.0f, deadband))
        {
            stabilizedPreviewPosition = requestedPosition;
        }

        return stabilizedPreviewPosition;
    }

    private Vector3 AdvancePreviewPositionTrajectory(
        Vector3 currentPosition,
        Vector3 targetPosition,
        float maxSpeed,
        float deltaTime)
    {
        Vector3 error = targetPosition - currentPosition;
        float distance = error.magnitude;
        if (distance <= 0.00001f)
        {
            previewLinearVelocity = Vector3.zero;
            return targetPosition;
        }

        float acceleration = Mathf.Max(0.01f, previewMaxLinearAcceleration);
        float brakingLimitedSpeed = Mathf.Sqrt(2.0f * acceleration * distance);
        Vector3 desiredVelocity = error.normalized * Mathf.Min(Mathf.Max(0.0f, maxSpeed), brakingLimitedSpeed);
        previewLinearVelocity = Vector3.MoveTowards(
            previewLinearVelocity,
            desiredVelocity,
            acceleration * Mathf.Max(0.0001f, deltaTime));

        Vector3 nextPosition = currentPosition + previewLinearVelocity * deltaTime;
        if (Vector3.Dot(targetPosition - currentPosition, targetPosition - nextPosition) <= 0.0f)
        {
            previewLinearVelocity = Vector3.zero;
            return targetPosition;
        }

        return nextPosition;
    }

    private Quaternion AdvancePreviewRotationTrajectory(
        Quaternion currentRotation,
        Quaternion targetRotation,
        float deltaTime)
    {
        Quaternion errorRotation = targetRotation * Quaternion.Inverse(currentRotation);
        errorRotation.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (angleDegrees > 180.0f)
        {
            angleDegrees -= 360.0f;
        }

        if (axis.sqrMagnitude < 0.000001f || Mathf.Abs(angleDegrees) <= 0.001f)
        {
            previewAngularVelocityDegrees = Vector3.zero;
            return targetRotation;
        }

        float acceleration = Mathf.Max(1.0f, previewMaxAngularAccelerationDegreesPerSecondSquared);
        float brakingLimitedSpeed = Mathf.Sqrt(2.0f * acceleration * Mathf.Abs(angleDegrees));
        Vector3 desiredVelocity = axis.normalized * Mathf.Sign(angleDegrees)
            * Mathf.Min(Mathf.Max(0.0f, previewMaxAngularSpeedDegreesPerSecond), brakingLimitedSpeed);
        previewAngularVelocityDegrees = Vector3.MoveTowards(
            previewAngularVelocityDegrees,
            desiredVelocity,
            acceleration * Mathf.Max(0.0001f, deltaTime));

        float stepDegrees = previewAngularVelocityDegrees.magnitude * deltaTime;
        if (stepDegrees >= Mathf.Abs(angleDegrees))
        {
            previewAngularVelocityDegrees = Vector3.zero;
            return targetRotation;
        }

        return Quaternion.AngleAxis(
            stepDegrees,
            previewAngularVelocityDegrees.normalized) * currentRotation;
    }

    private Vector3 SmoothPreviewPosition(Vector3 currentPosition, Vector3 targetPosition, float maxSpeed, float deltaTime)
    {
        float blend = 1.0f - Mathf.Exp(
            -Mathf.Max(0.0f, previewPositionSmoothingSharpness) * Mathf.Max(0.0001f, deltaTime));
        Vector3 blendedPosition = Vector3.Lerp(currentPosition, targetPosition, blend);
        return Vector3.MoveTowards(currentPosition, blendedPosition, Mathf.Max(0.0f, maxSpeed) * deltaTime);
    }

    private Quaternion SmoothPreviewRotation(Quaternion currentRotation, Quaternion targetRotation, float deltaTime)
    {
        float blend = 1.0f - Mathf.Exp(
            -Mathf.Max(0.0f, previewRotationSmoothingSharpness) * Mathf.Max(0.0001f, deltaTime));
        Quaternion blendedRotation = Quaternion.Slerp(currentRotation, targetRotation, blend);
        return Quaternion.RotateTowards(
            currentRotation,
            blendedRotation,
            Mathf.Max(0.0f, previewMaxAngularSpeedDegreesPerSecond) * deltaTime);
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

    private float GetJoystickYawInput(Vector2 joystick)
    {
        // X remains the documented yaw axis. Y is accepted as a fallback so
        // either deliberate stick direction produces the same grasp-yaw command.
        return Mathf.Abs(joystick.x) > 0.000001f ? joystick.x : joystick.y;
    }

    private Vector3 GetGripperCenterSymmetryAxisWorld()
    {
        if (tcpFollower != null)
        {
            Vector3 physicalAxis = tcpFollower.GripperCenterSymmetryAxisWorld;
            if (physicalAxis.sqrMagnitude > 0.000001f)
            {
                return physicalAxis.normalized;
            }
        }

        return tcpPreviewTarget != null
            ? GetGripperCenterSymmetryAxisWorld(GetPreviewGraspRotation(tcpPreviewTarget.rotation))
            : Vector3.forward;
    }

    private Vector3 GetGripperCenterSymmetryAxisWorld(Quaternion graspRotation)
    {
        Vector3 axis = graspRotation * Vector3.forward;
        return axis.sqrMagnitude > 0.000001f ? axis.normalized : Vector3.forward;
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
            && device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool stickPressed)
            && stickPressed;
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
        leftSecondaryPoseYawDegreesPerMeter = Mathf.Max(0.0f, leftSecondaryPoseYawDegreesPerMeter);
        leftSecondaryPoseTwistScale = Mathf.Max(0.0f, leftSecondaryPoseTwistScale);
        joystickPitchSpeedDegreesPerSecond = Mathf.Max(0.0f, joystickPitchSpeedDegreesPerSecond);
        joystickRollSpeedDegreesPerSecond = Mathf.Max(0.0f, joystickRollSpeedDegreesPerSecond);
        commandSmoothingSharpness = Mathf.Max(0.0f, commandSmoothingSharpness);
        maxLinearAcceleration = Mathf.Max(0.0f, maxLinearAcceleration);
        maxAngularAcceleration = Mathf.Max(0.0f, maxAngularAcceleration);
        workspacePredictionHorizonSeconds = Mathf.Max(0.02f, workspacePredictionHorizonSeconds);
        relativePreviewPositionScale = Mathf.Max(0.0f, relativePreviewPositionScale);
        precisionPositionScale = Mathf.Max(0.0f, precisionPositionScale);
        progressivePositionTransitionMeters = Mathf.Max(0.0001f, progressivePositionTransitionMeters);
        relativePreviewRotationScale = Mathf.Max(0.0f, relativePreviewRotationScale);
        previewPositionSmoothingSharpness = Mathf.Max(0.0f, previewPositionSmoothingSharpness);
        previewRotationSmoothingSharpness = Mathf.Max(0.0f, previewRotationSmoothingSharpness);
        relativePoseCommandFilterRetention = Mathf.Clamp(relativePoseCommandFilterRetention, 0.0f, 0.95f);
        fineRelativePoseCommandFilterRetention = Mathf.Clamp(fineRelativePoseCommandFilterRetention, 0.0f, 0.95f);
        previewMaxLinearSpeed = Mathf.Max(0.0f, previewMaxLinearSpeed);
        previewMaxAngularSpeedDegreesPerSecond = Mathf.Max(0.0f, previewMaxAngularSpeedDegreesPerSecond);
        previewMaxLinearAcceleration = Mathf.Max(0.01f, previewMaxLinearAcceleration);
        previewMaxAngularAccelerationDegreesPerSecondSquared = Mathf.Max(1.0f, previewMaxAngularAccelerationDegreesPerSecondSquared);
        previewTargetDeadbandMeters = Mathf.Max(0.0f, previewTargetDeadbandMeters);
        finePreviewTargetDeadbandMeters = Mathf.Max(0.0f, finePreviewTargetDeadbandMeters);
        controllerPositionMinimumCutoffHz = Mathf.Max(0.01f, controllerPositionMinimumCutoffHz);
        controllerPositionFilterBeta = Mathf.Max(0.0f, controllerPositionFilterBeta);
        controllerPositionDerivativeCutoffHz = Mathf.Max(0.01f, controllerPositionDerivativeCutoffHz);
        controllerMotionEpsilonMeters = Mathf.Max(0.0f, controllerMotionEpsilonMeters);
        fineControllerMotionEpsilonMeters = Mathf.Max(0.0f, fineControllerMotionEpsilonMeters);
        controllerStopHoldSeconds = Mathf.Max(0.0f, controllerStopHoldSeconds);
        verticalApproachSnapDegrees = Mathf.Clamp(verticalApproachSnapDegrees, 1.0f, 89.0f);
        leftPrimaryReadyPoseHoldSeconds = Mathf.Max(0.0f, leftPrimaryReadyPoseHoldSeconds);
        maximumPreviewLeadMeters = Mathf.Max(0.0f, maximumPreviewLeadMeters);
        fineLinearSpeedMultiplier = Mathf.Clamp(fineLinearSpeedMultiplier, 0.1f, 1.0f);
        fineAngularSpeedMultiplier = Mathf.Clamp(fineAngularSpeedMultiplier, 0.1f, 1.0f);
        gripPressThreshold = Mathf.Clamp01(gripPressThreshold);
        gripReleaseThreshold = Mathf.Clamp(gripReleaseThreshold, 0.0f, gripPressThreshold);
    }
}

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
    public TcpTargetWriteMonitor targetWriteMonitor;
    public Ur5UrScriptSpeedlClient speedlClient;

    [Header("Controllers")]
    public XRNode positionControllerNode = XRNode.RightHand;
    public XRNode rotationControllerNode = XRNode.RightHand;
    [Tooltip("左手安全姿态使用独立设备，不能与右手 6DoF 旋转节点复用。")]
    public XRNode safetyControllerNode = XRNode.LeftHand;
    public bool usePositionGripAsDeadman = true;
    public bool useRotationGripAsDeadman = true;
    [Tooltip("Locked is the pick-and-place default: translation is tracked while the TCP attitude is held. Joystick and controller-pose modes are optional Inspector-only modes for special tasks.")]
    public RotationInputMode rotationInputMode = RotationInputMode.Locked;

    [Header("UR10 风格单模式 6DoF 离合")]
    [Tooltip("标准 Quest 控制：右手 Grip 的上升沿锚定原始手柄 Pose 和实际 TCP；保持期间仅执行相对 Pose 映射。")]
    public bool enableUr10StyleAnchoredPoseClutch = true;
    [Tooltip("兼容旧实验性连续速度控制。标准 Quest Profile 必须关闭。")]
    public bool enableContinuous6DofClutch;
    public Ur5Continuous6DofConfig continuous6DofConfig = Ur5Continuous6DofConfig.Default;

    [Header("Right Controller Position Stabilization")]
    [Tooltip("Rejects millimetre-level Quest controller jitter before it can move the TCP target. This affects only the right-hand Cartesian command, never joint-state recording.")]
    public bool filterControllerPosition = true;
    [Tooltip("Small controller-position changes below this radius are treated as tracking noise.")]
    public float controllerPositionJitterDeadbandMeters = 0.0025f;
    [Tooltip("Used only by the optional adaptive filter. The default noise gate has no time-based follow lag.")]
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

    [Header("右手 A 默认下抓姿态")]
    [Tooltip("右手 Grip+A 首次调姿时使用基座参考方向加默认 yaw 偏移，避免沿用当前向内反的腕部 yaw。")]
    public bool useRobotBaseForwardForRightAPose = true;
    [Tooltip("右手 Grip+A 默认下抓姿态相对基座 forward 的 yaw 偏移。180 度对应当前 UR5/Robotiq 模型的外翻下抓默认姿态。")]
    [Range(-180.0f, 180.0f)] public float rightADefaultJawYawOffsetDegrees = 180.0f;

    [Header("右手 B 自由腕部姿态")]
    [Tooltip("右手 Grip+B 进入通用自由姿态控制：位置保持，完整映射右手柄相对旋转，可用于横抓/侧抓等非朝下任务。")]
    public bool enableRightSecondaryFreeWristPoseControl = true;
    [Tooltip("右手 Grip+B 的 controller quaternion 旋转倍率。1.0 表示完全跟随手柄相对姿态。")]
    [Range(0.10f, 1.50f)] public float rightSecondaryFreeWristRotationScale = 1.0f;

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
    [Tooltip("Use the anchored relative-pose loop from elpis-lab/UR10_Teleop. Grip captures the controller and actual TCP together; tracking derives every target from that pair and applies one smoothing step.")]
    public bool useAnchoredPoseTeleopStrategy = true;
    [Tooltip("Single target smoothing step at each FixedUpdate. Higher is more responsive; this replaces target deadbands and the second MoveTowards limiter.")]
    [Range(0.01f, 1.0f)] public float anchoredPoseSmoothingStep = 0.18f;
    [Tooltip("Optional lower single-filter step while A is held. Keep it non-zero so precise motion remains responsive.")]
    [Range(0.01f, 1.0f)] public float anchoredPosePrecisionSmoothingStep = 0.18f;
    [Tooltip("采用 Open-Teach 同类的单级相对位姿滤波。开启后不再叠加 TCP 的第二层指数平滑，减少 Quest 操作中的迟滞与卡顿。")]
    public bool useRelativePoseCommandFilter = true;
    [Tooltip("最终 TCP 命令的上一帧保留比例。0 为完全直通；0.55 在 Quest 追踪噪声与快速响应之间取得平衡。")]
    [Range(0.0f, 0.95f)] public float relativePoseCommandFilterRetention = 0.55f;
    [Tooltip("精细模式使用的 TCP 命令保留比例。略高于常规模式，用于抑制 Quest 微抖，但不会产生松手后的滤波尾巴。")]
    [Range(0.0f, 0.95f)] public float fineRelativePoseCommandFilterRetention = 0.40f;
    [Tooltip("将旧的每帧保留比例解释为该刷新率下的值；90 Hz 保持旧版本的初始响应。")]
    [Range(30.0f, 144.0f)] public float relativePoseFilterReferenceRateHz = 90.0f;
    [Tooltip("实验性 TCP 加速度轨迹。默认关闭：Unity 预览使用直接限速目标，避免停手后继续追赶。")]
    public bool useAccelerationLimitedPreviewTrajectory = false;
    public float previewMaxLinearAcceleration = 1.60f;
    public float previewMaxAngularAccelerationDegreesPerSecondSquared = 1800.0f;
    [Tooltip("TCP target changes smaller than this are held during normal right-hand control, preventing tracking noise from entering IK.")]
    public float previewTargetDeadbandMeters = 0.00060f;
    [Tooltip("Smaller target deadband used while A precision modifier is held.")]
    public float finePreviewTargetDeadbandMeters = 0.00020f;

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
    [Tooltip("短按 X 朝下动作的最长执行时间，单位秒；超时后立即保持当前姿态。")]
    public float leftPrimarySnapTimeoutSeconds = 3.0f;
    [Tooltip("短按 X 朝下动作完成时允许的 TCP 位置误差，单位米。")]
    public float leftPrimarySnapPositionToleranceMeters = 0.003f;
    [Tooltip("短按 X 朝下动作完成时允许的抓取姿态误差，单位度。")]
    public float leftPrimarySnapRotationToleranceDegrees = 0.50f;
    [Tooltip("Hold Y on the left controller to freeze the current TCP orientation while the right hand translates.")]
    public bool enableLeftSecondaryOrientationHold = true;

    [Header("Actual TCP Lead Limit")]
    [Tooltip("Prevents the IK command target from running far ahead of the real two-pad TCP when the hand moves faster than the arm can track.")]
    public bool limitPreviewLeadToActualTcp = true;
    [Tooltip("Safety window used while the controller is stopped or being released. Keep this small so the arm cannot chase a stale target.")]
    public float maximumPreviewLeadMeters = 0.060f;
    [Tooltip("Larger command lead used while Grip is held and stop-hold has not frozen the TCP. Stop-hold returns to maximumPreviewLeadMeters.")]
    public float movingPreviewLeadMeters = 0.160f;

    [Header("Grip Hysteresis")]
    [Range(0.0f, 1.0f)] public float gripPressThreshold = 0.65f;
    [Range(0.0f, 1.0f)] public float gripReleaseThreshold = 0.40f;

    [Header("Legacy Optional Fine Mode")]
    [Tooltip("Compatibility-only thumbstick-click scaling. The standard assembly profile keeps this disabled because precision tracking is always active.")]
    public bool enableFineControlButton = false;
    public bool applyFineControlToRelativePreview = false;
    [Range(0.1f, 1.0f)] public float fineLinearSpeedMultiplier = 1.00f;
    [Range(0.1f, 1.0f)] public float fineAngularSpeedMultiplier = 1.00f;

    [Header("A Precision Modifier")]
    [Tooltip("Quest 右手 A 键。它是临时倍率修饰键，不会切换控制模式。")]
    public bool enableAPrecisionModifier = true;
    [Tooltip("正常相对手部位移到 TCP 的倍率。")]
    [Range(0.05f, 4.5f)] public float normalPositionScale = 0.80f;
    [Tooltip("按住 A 时使用的精细位置倍率。")]
    [Range(0.01f, 1.0f)] public float precisionModifierPositionScale = 0.15f;
    [Tooltip("正常左摇杆工具轴旋转最高速度。")]
    [Range(1.0f, 180.0f)] public float normalJoystickYawSpeedDegreesPerSecond = 55.0f;
    [Tooltip("按住 A 时左摇杆工具轴旋转最高速度。")]
    [Range(1.0f, 60.0f)] public float precisionJoystickYawSpeedDegreesPerSecond = 10.0f;
    [Range(1.0f, 3.0f)] public float joystickResponseExponent = 1.5f;

    [Header("三模式离合控制")]
    [Tooltip("启用后，右手 Grip 进入显式 Idle/Clutched/Paused/Fault 状态机；A=fine，B=insert，无按键=free。")]
    public bool enableThreeModeController = true;
    [Tooltip("右手 B 键请求 insert 模式；优先级高于 A 键 fine 模式。")]
    public bool useRightSecondaryButtonForInsertMode = true;
    public Ur5TeleopModeConfig freeModeConfig = Ur5ClutchModeController.DefaultFreeConfig;
    public Ur5TeleopModeConfig fineModeConfig = Ur5ClutchModeController.DefaultFineConfig;
    public Ur5TeleopModeConfig insertModeConfig = Ur5ClutchModeController.DefaultInsertConfig;

    [Header("Output")]
    public bool sendToSpeedlClient = true;
    public bool logDeviceStatus = true;

    private InputDevice positionDevice;
    private InputDevice rotationDevice;
    private InputDevice safetyDevice;
    private bool hasLoggedMissingPositionDevice;
    private bool hasLoggedMissingRotationDevice;
    private bool hasLoggedMissingSafetyDevice;
    private bool positionGripLatched;
    private bool rotationGripLatched;
    private bool safetyGripLatched;
    private bool wasPositionClutched;
    private bool wasRotationClutched;
    private bool wasFinePositionControlActive;
    private Ur5TeleopMode requestedTeleopMode;
    private Ur5TeleopMode previousPositionTeleopMode;
    private Ur5TeleopStepResult latestClutchModeStep;

    private Vector3 positionNeutralWorldPosition;
    private Quaternion rotationNeutralWorldRotation;
    private Vector3 positionClutchStartTargetWorldPosition;
    private Vector3 positionClutchStartActualToolWorldPosition;
    private Quaternion rotationClutchStartTargetWorldRotation;
    private Quaternion rotationClutchStartGraspWorldRotation;
    // Y 姿态通道有自己的零点，不能复用摇杆/普通姿态的 clutch，
    // 否则双手切换控制时会把旧的输入基准带入当前目标。
    private Vector3 secondaryPoseNeutralWorldPosition;
    private Quaternion secondaryPoseNeutralWorldRotation = Quaternion.identity;
    private Quaternion secondaryPoseStartGraspWorldRotation = Quaternion.identity;
    private Vector3 filteredControllerPositionWorld;
    private readonly Ur5OneEuroVectorFilter controllerPositionOneEuroFilter = new Ur5OneEuroVectorFilter();
    private readonly Ur5PositionNoiseGate controllerPositionNoiseGate = new Ur5PositionNoiseGate();
    private readonly Ur5RotationNoiseGate ur10StyleRotationNoiseGate = new Ur5RotationNoiseGate();
    private readonly Ur5RelativePoseCommandFilter relativePoseCommandFilter = new Ur5RelativePoseCommandFilter();
    private readonly Ur5RelativePoseClutchMapper relativePoseClutchMapper = new Ur5RelativePoseClutchMapper();
    private readonly Ur5AnchoredPoseTeleopStrategy anchoredPoseTeleop = new Ur5AnchoredPoseTeleopStrategy();
    private readonly Ur5ClutchModeController clutchModeController = new Ur5ClutchModeController();
    private readonly Ur5Continuous6DofClutchController continuous6DofController =
        new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
    private Ur5Continuous6DofStepResult latestContinuous6DofStep;
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
    private Quaternion positionMotionReferenceRotation = Quaternion.identity;
    private bool hasPositionMotionReference;
    private Vector3 ur10StyleCommandPositionWorld;
    private Quaternion ur10StyleCommandRotationWorld = Quaternion.identity;
    private bool hasUr10StyleCommandPose;
    private Quaternion ur10StyleLockedToolRotation = Quaternion.identity;
    private bool hasUr10StyleLockedToolRotation;
    private bool ur10StyleRotationAdjustActive;
    private bool wasUr10StyleRotationAdjustActive;
    private bool ur10StyleFreeWristAdjustActive;
    private bool wasUr10StyleFreeWristAdjustActive;
    private Quaternion ur10StyleFreeWristStartControllerRotation = Quaternion.identity;
    private Quaternion ur10StyleFreeWristStartGraspWorldRotation = Quaternion.identity;
    private bool hasUr10StyleFreeWristReference;
    private Quaternion ur10StyleRotationAdjustStartControllerRotation = Quaternion.identity;
    private Quaternion ur10StyleRotationAdjustStartGraspWorldRotation = Quaternion.identity;
    private bool hasUr10StyleRotationAdjustReference;
    private float positionHandStillSeconds;
    private bool positionHandStopHoldActive;
    private Ur5TcpTargetFollower tcpFollower;
    private Ur5LeftSafetyPoseController leftSafetyPoseController;
    private float configuredLeftPrimaryReadyPoseHoldSeconds = -1.0f;
    private float configuredLeftPrimarySnapTimeoutSeconds = -1.0f;
    private bool hasLeftPrimarySnapTarget;
    private Vector3 leftPrimarySnapTargetPosition;
    private Quaternion leftPrimarySnapTargetRotation = Quaternion.identity;
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
    private bool wasUr10StyleGripHeld;

    public Vector3 LogicalCommandPosition { get; private set; }
    public Quaternion LogicalCommandRotation { get; private set; } = Quaternion.identity;
    public Vector3 FilteredCommandPosition { get; private set; }
    public Quaternion FilteredCommandRotation { get; private set; } = Quaternion.identity;
    public Vector3 ConstrainedCommandPosition { get; private set; }
    public bool IsPreviewLeadLimited { get; private set; }
    public float ActivePreviewLeadLimitMeters => GetActivePreviewLeadLimitMeters();
    public bool IsResponsiveMovingPreviewLeadActive => UsesResponsiveMovingPreviewLead();
    public bool IsPrecisionModifierHeld { get; private set; }
    public Ur5TeleopMode ActiveTeleopMode => enableContinuous6DofClutch
        ? Ur5TeleopMode.Free
        : enableThreeModeController
        ? latestClutchModeStep.ActiveMode
        : (IsFinePositionControlActive ? Ur5TeleopMode.Fine : Ur5TeleopMode.Free);
    public Ur5TeleopControllerState TeleopControllerState => enableContinuous6DofClutch
        ? latestContinuous6DofStep.State
        : enableThreeModeController
        ? latestClutchModeStep.State
        : (IsCommandActive ? Ur5TeleopControllerState.Clutched : Ur5TeleopControllerState.Idle);
    public Ur5TeleopModeConfig ActiveTeleopModeConfig => GetActiveTeleopModeConfig();
    public bool EnableContinuous6DofClutch => enableContinuous6DofClutch;
    public bool UsesUr10StyleAnchoredPoseClutch => enableUr10StyleAnchoredPoseClutch
        && !enableContinuous6DofClutch;
    public Ur5Continuous6DofFaultReason Continuous6DofFaultReason =>
        latestContinuous6DofStep.FaultReason;
    public float ContinuousControllerDistanceMeters =>
        latestContinuous6DofStep.ControllerDistanceMeters;
    public float ContinuousControllerAngleDegrees =>
        latestContinuous6DofStep.ControllerAngleDegrees;
    public float ContinuousTranslationGain =>
        latestContinuous6DofStep.TranslationGain;
    public float ContinuousRotationGain =>
        latestContinuous6DofStep.RotationGain;
    public bool IsAnchoredPoseStrategyActive => useAnchoredPoseTeleopStrategy && anchoredPoseTeleop.IsTracking;
    public float ActiveAnchoredPoseSmoothingStep => GetAnchoredPoseSmoothingStep();
    public Vector3 RawControllerPositionWorld => latestRawPositionWorld;
    public Quaternion RawControllerRotationWorld => latestRotationValid
        ? latestRotationWorld
        : Quaternion.identity;
    public Vector3 StabilizedControllerPositionWorld => filteredControllerPositionWorld;
    public float ControllerPositionInputDifferenceMeters => latestRawPositionValid && hasFilteredControllerPosition
        ? Vector3.Distance(latestRawPositionWorld, filteredControllerPositionWorld)
        : 0.0f;
    public bool IsControllerPositionNoiseGateHolding => filterControllerPosition
        && latestRawPositionValid
        && hasFilteredControllerPosition
        && ControllerPositionInputDifferenceMeters <= controllerPositionJitterDeadbandMeters;
    public bool IsSafetyPoseCommandActive => leftSafetyPoseController != null
        && leftSafetyPoseController.BlocksRightGrip;
    public Ur5LeftSafetyPoseState SafetyPoseState => leftSafetyPoseController != null
        ? leftSafetyPoseController.State
        : Ur5LeftSafetyPoseState.Idle;

    public bool IsDeviceValid => positionDevice.isValid || rotationDevice.isValid;
    public bool IsPositionClutched { get; private set; }
    public bool IsRotationClutched { get; private set; }
    // 右手 Grip 是唯一平移离合；Grip+A 和 Grip+B 是显式姿态入口。
    // 释放后不再产生新命令，避免 IK 追赶旧目标。
    public bool IsCommandActive => IsPositionClutched
        || IsRotationCommandActive
        || IsSafetyPoseCommandActive;
    public bool IsFineControlActive { get; private set; }
    /// <summary>右手 Grip + 右摇杆按下时的无跳变精细平移模式。</summary>
    public bool IsFinePositionControlActive { get; private set; }
    public bool IsInsertModeActive { get; private set; }
    public bool IsWorkspaceLimited { get; private set; }
    public bool IsInputPoseValid { get; private set; }
    public Vector3 RawBaseLinearVelocity => rawBaseLinearVelocity;
    public Vector3 RawBaseAngularVelocity => rawBaseAngularVelocity;
    public Vector3 BaseLinearVelocity => filteredBaseLinearVelocity;
    public Vector3 BaseAngularVelocity => filteredBaseAngularVelocity;
    public RotationInputMode CurrentRotationInputMode => rotationInputMode;
    public bool IsOrientationLocked => rotationInputMode == RotationInputMode.Locked;
    public bool IsUr10StyleRotationAdjustActive => UsesUr10StyleAnchoredPoseClutch
        && IsPositionClutched
        && ur10StyleRotationAdjustActive;
    public bool IsUr10StyleFreeWristAdjustActive => UsesUr10StyleAnchoredPoseClutch
        && IsPositionClutched
        && ur10StyleFreeWristAdjustActive;
    public bool IsUr10StyleWristRotationAdjustActive => IsUr10StyleRotationAdjustActive
        || IsUr10StyleFreeWristAdjustActive;
    /// <summary>右手 Grip 单独平移时是否已捕获完整 TCP 姿态锁。</summary>
    public bool IsPositionOrientationLocked => IsLegacyPositionOrientationLocked
        || IsUr10StyleGripOnlyOrientationLocked;
    private bool IsLegacyPositionOrientationLocked => hasPositionOrientationLock
        && IsPositionClutched
        && !IsRotationClutched;
    private bool IsUr10StyleGripOnlyOrientationLocked => UsesUr10StyleAnchoredPoseClutch
        && IsPositionClutched
        && !ur10StyleRotationAdjustActive
        && !ur10StyleFreeWristAdjustActive
        && hasUr10StyleLockedToolRotation;
    public bool IsRotationCommandActive => IsRotationClutched
        && (ur10StyleRotationAdjustActive
            || ur10StyleFreeWristAdjustActive
            || isSecondaryPoseRotationActive
            || !IsJoystickRotationMode()
            || (latestRotationJoystickValid
                && ApplyJoystickDeadband(latestRotationJoystick, rotationJoystickDeadband).sqrMagnitude > 0.000001f));
    public bool IsSafetyOrientationHoldActive { get; private set; }
    public bool IsRotationJoystickValid => latestRotationJoystickValid;
    public Vector2 RotationJoystickInput => latestRotationJoystickValid ? latestRotationJoystick : Vector2.zero;
    public bool IsJoystickRollModifierActive => isJoystickRollModifierActive;

    private void Awake()
    {
        EnsureLeftSafetyPoseController();
        ResolveReferences();
    }

    private void Start()
    {
        TryRefreshPositionDevice();
        TryRefreshRotationDevice();
        TryRefreshSafetyDevice();
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

        if (UsesUr10StyleAnchoredPoseClutch)
        {
            UpdateUr10StyleAnchoredPoseInput(
                hasPosition,
                positionWorld,
                hasRotation,
                rotationWorld);
            wasPositionClutched = IsPositionClutched;
            wasRotationClutched = IsRotationClutched;
            wasSecondaryPoseRotationActive = false;
            previousPositionTeleopMode = Ur5TeleopMode.Free;
            return;
        }

        bool continuousPoseValid = hasPosition && hasRotation;
        bool continuousRightGripHeld = enableContinuous6DofClutch
            && continuousPoseValid
            && (!usePositionGripAsDeadman
                || ReadGripDeadman(positionDevice, ref positionGripLatched));

        UpdateLeftControllerSafetyPose(continuousRightGripHeld);
        bool continuousGripHeld = continuousRightGripHeld && !IsSafetyPoseCommandActive;
        SynchronizeOrientationAfterReadyPose();

        if (enableContinuous6DofClutch)
        {
            UpdateContinuous6DofInput(
                hasPosition,
                positionWorld,
                hasRotation,
                rotationWorld,
                continuousGripHeld);
            wasPositionClutched = IsPositionClutched;
            wasRotationClutched = IsRotationClutched;
            wasSecondaryPoseRotationActive = isSecondaryPoseRotationActive;
            previousPositionTeleopMode = requestedTeleopMode;
            return;
        }

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
            && !IsSafetyPoseCommandActive)
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
        IsPrecisionModifierHeld = ReadAPrecisionModifier(positionDevice);
        requestedTeleopMode = ResolveRequestedTeleopMode();
        IsInsertModeActive = IsPositionClutched && requestedTeleopMode == Ur5TeleopMode.Insert;
        IsFinePositionControlActive = IsPositionClutched && requestedTeleopMode == Ur5TeleopMode.Fine;
        IsFineControlActive = requestedTeleopMode == Ur5TeleopMode.Fine
            && (IsPositionClutched || IsRotationClutched);
        UpdateClutchModeState(positionWorld, rotationWorld);

        CaptureClutchOrigins(positionWorld, rotationWorld);
        RebasePositionClutchForModeChange(positionWorld, rotationWorld);
        UpdatePositionHandStopHold(Time.deltaTime);
        CalculateRawVelocity(positionWorld, rotationWorld);
        ApplySafetyLimitsAndFiltering(Time.deltaTime);

        wasPositionClutched = IsPositionClutched;
        wasRotationClutched = IsRotationClutched;
        wasSecondaryPoseRotationActive = isSecondaryPoseRotationActive;
        previousPositionTeleopMode = requestedTeleopMode;
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
                IsCommandActive && IsInputPoseValid && IsTeleopMotionAllowed());
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
        controllerPositionNoiseGate.Clear();
        relativePoseCommandFilter.Clear();
        relativePoseClutchMapper.End();
        anchoredPoseTeleop.Pause();
        ResetUr10StyleCommandPoseGate();
        wasUr10StyleGripHeld = false;
        clutchModeController.Pause();
        continuous6DofController.Pause();
        leftSafetyPoseController?.Reset();
        hasLeftPrimarySnapTarget = false;
        latestContinuous6DofStep = CreateContinuousInactiveStep(
            Ur5TeleopControllerState.Paused,
            Ur5Continuous6DofFaultReason.None);

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

        if (!safetyDevice.isValid)
        {
            TryRefreshSafetyDevice();
        }
    }

    private Ur5TeleopMode ResolveRequestedTeleopMode()
    {
        if (!enableThreeModeController)
        {
            return IsPrecisionModifierHeld || ReadFineControl(positionDevice)
                ? Ur5TeleopMode.Fine
                : Ur5TeleopMode.Free;
        }

        if (useRightSecondaryButtonForInsertMode && ReadSecondaryButton(positionDevice))
        {
            return Ur5TeleopMode.Insert;
        }

        return IsPrecisionModifierHeld || ReadFineControl(positionDevice)
            ? Ur5TeleopMode.Fine
            : Ur5TeleopMode.Free;
    }

    private void UpdateClutchModeState(Vector3 positionWorld, Quaternion rotationWorld)
    {
        freeModeConfig = freeModeConfig.Sanitized();
        fineModeConfig = fineModeConfig.Sanitized();
        insertModeConfig = insertModeConfig.Sanitized();
        clutchModeController.Configure(freeModeConfig, fineModeConfig, insertModeConfig);

        if (!enableThreeModeController)
        {
            latestClutchModeStep = new Ur5TeleopStepResult(
                IsPositionClutched ? Ur5TeleopControllerState.Clutched : Ur5TeleopControllerState.Idle,
                requestedTeleopMode,
                GetActiveTeleopModeConfig(),
                IsPositionClutched,
                false,
                Vector3.zero,
                0.0f,
                tcpPreviewTarget != null ? tcpPreviewTarget.position : Vector3.zero,
                tcpPreviewTarget != null ? tcpPreviewTarget.rotation : Quaternion.identity);
            return;
        }

        latestClutchModeStep = clutchModeController.Step(new Ur5TeleopStepInput(
            gripHeld: IsPositionClutched,
            requestedMode: requestedTeleopMode,
            controllerPosition: latestPositionValid ? positionWorld : Vector3.zero,
            controllerRotation: latestRotationValid ? rotationWorld : Quaternion.identity,
            actualTcpPosition: tcpPreviewTarget != null ? GetActualToolPosition() : Vector3.zero,
            actualTcpRotation: tcpPreviewTarget != null ? GetActualToolRotation() : Quaternion.identity,
            isInputPoseValid: latestPositionValid,
            isRobotStateValid: tcpPreviewTarget != null && IsRobotOutputReady(),
            isSafetyAccepted: targetWriteMonitor == null || !targetWriteMonitor.HadWriteConflictThisFrame));
    }

    private void UpdateContinuous6DofInput(
        bool hasPosition,
        Vector3 positionWorld,
        bool hasRotation,
        Quaternion rotationWorld,
        bool gripHeld)
    {
        bool poseValid = hasPosition && hasRotation;
        bool safetyPoseOwnsTarget = IsSafetyPoseCommandActive
            || (tcpFollower != null && tcpFollower.IsReadyPoseActive);

        if (safetyPoseOwnsTarget)
        {
            // 左手 X/ready pose 是显式安全写入源；取得所有权后必须要求
            // 右手 Grip 先释放再重新离合，避免同一轮 clutch 接着写旧基准。
            continuous6DofController.Pause();
            latestContinuous6DofStep = CreateContinuousInactiveStep(
                Ur5TeleopControllerState.Paused,
                Ur5Continuous6DofFaultReason.None);
            IsPositionClutched = false;
            IsRotationClutched = false;
            IsInputPoseValid = poseValid;
            return;
        }

        IsPositionClutched = gripHeld;
        IsRotationClutched = gripHeld;
        IsInputPoseValid = poseValid;
        IsFineControlActive = false;
        IsFinePositionControlActive = false;
        IsInsertModeActive = false;
        isSecondaryPoseRotationActive = false;
        IsPrecisionModifierHeld = false;
        requestedTeleopMode = Ur5TeleopMode.Free;

        continuous6DofConfig = continuous6DofConfig.Sanitized();
        continuous6DofController.Configure(continuous6DofConfig);
        latestContinuous6DofStep = continuous6DofController.Step(
            new Ur5Continuous6DofStepInput(
                gripHeld,
                positionWorld,
                rotationWorld,
                tcpPreviewTarget != null ? GetActualToolPosition() : Vector3.zero,
                tcpPreviewTarget != null ? GetActualToolRotation() : Quaternion.identity,
                poseValid,
                tcpPreviewTarget != null && IsRobotOutputReady(),
                targetWriteMonitor == null || !targetWriteMonitor.HadWriteConflictThisFrame));

        if (latestContinuous6DofStep.WasAnchoredThisStep)
        {
            relativePoseCommandFilter.Reset(
                latestContinuous6DofStep.TargetPosition,
                latestContinuous6DofStep.TargetRotation);
        }

        rawBaseLinearVelocity = Vector3.zero;
        rawBaseAngularVelocity = Vector3.zero;
        limitedBaseLinearVelocity = Vector3.zero;
        limitedBaseAngularVelocity = Vector3.zero;
        filteredBaseLinearVelocity = Vector3.zero;
        filteredBaseAngularVelocity = Vector3.zero;
    }

    private void UpdateUr10StyleAnchoredPoseInput(
        bool hasPosition,
        Vector3 rawPositionWorld,
        bool hasRotation,
        Quaternion rawRotationWorld)
    {
        bool gripHeld = !usePositionGripAsDeadman
            || ReadGripDeadman(positionDevice, ref positionGripLatched);
        bool poseValid = hasPosition && hasRotation;

        // Grip is the only state transition in the reference controller. A
        // temporary tracking dropout suppresses writes but does not silently
        // replace the immutable anchors while the user is still holding Grip.
        if (!gripHeld)
        {
            anchoredPoseTeleop.Pause();
            ResetUr10StyleHandStopHold();
            ResetUr10StyleCommandPoseGate();
            hasUr10StyleLockedToolRotation = false;
            ur10StyleRotationAdjustActive = false;
            wasUr10StyleRotationAdjustActive = false;
            hasUr10StyleRotationAdjustReference = false;
            ur10StyleFreeWristAdjustActive = false;
            wasUr10StyleFreeWristAdjustActive = false;
            hasUr10StyleFreeWristReference = false;
        }
        else if (poseValid)
        {
            bool freeWristAdjustRequested = IsUr10StyleFreeWristRotationAdjustmentRequested();
            bool rotationAdjustRequested = !freeWristAdjustRequested
                && IsUr10StyleRotationAdjustmentRequested();
            bool freeWristAdjustStarted = freeWristAdjustRequested
                && !wasUr10StyleFreeWristAdjustActive;
            bool freeWristAdjustEnded = !freeWristAdjustRequested
                && wasUr10StyleFreeWristAdjustActive;
            bool rotationAdjustStarted = rotationAdjustRequested && !wasUr10StyleRotationAdjustActive;
            bool rotationAdjustEnded = !rotationAdjustRequested && wasUr10StyleRotationAdjustActive;
            ur10StyleFreeWristAdjustActive = freeWristAdjustRequested;
            ur10StyleRotationAdjustActive = rotationAdjustRequested;

            UpdateUr10StyleCommandPose(
                rawPositionWorld,
                rawRotationWorld,
                !wasUr10StyleGripHeld || rotationAdjustStarted || freeWristAdjustStarted);
            if (!wasUr10StyleGripHeld && tcpPreviewTarget != null)
            {
                tcpFollower?.HoldCurrentJointCommandsAtMeasuredPose();
                Vector3 toolPosition = GetActualToolPosition();
                Quaternion toolRotation = GetUr10StyleGripStartToolRotation();
                anchoredPoseTeleop.Resume(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld,
                    toolPosition,
                    toolRotation);
                SetUr10StylePreviewTargetImmediate(
                    toolPosition,
                    toolRotation,
                    "QuestTeleopUr10StyleAnchoredPose");
                ur10StyleLockedToolRotation = toolRotation;
                hasUr10StyleLockedToolRotation = true;
                hasUr10StyleRotationAdjustReference = false;
                hasUr10StyleFreeWristReference = false;
                ResetUr10StyleHandStopHold(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld);
            }
            else if (freeWristAdjustStarted && tcpPreviewTarget != null)
            {
                CaptureUr10StyleFreeWristReference();
                RebaseUr10StyleAtCommandPose(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld);
                ResetUr10StyleHandStopHold(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld);
            }
            else if (rotationAdjustStarted && tcpPreviewTarget != null)
            {
                CaptureUr10StyleRotationAdjustReference();
                RebaseUr10StyleAtCommandPose(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld);
                ResetUr10StyleHandStopHold(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld);
            }
            else if ((rotationAdjustEnded || freeWristAdjustEnded) && tcpPreviewTarget != null)
            {
                ur10StyleLockedToolRotation = tcpPreviewTarget.rotation;
                hasUr10StyleLockedToolRotation = true;
                persistentOrientationTarget = ur10StyleLockedToolRotation;
                hasPersistentOrientationTarget = true;
                hasUr10StyleRotationAdjustReference = false;
                hasUr10StyleFreeWristReference = false;
                RebaseUr10StyleAtCommandPose(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld);
                ResetUr10StyleHandStopHold(
                    ur10StyleCommandPositionWorld,
                    ur10StyleCommandRotationWorld);
            }

            wasUr10StyleRotationAdjustActive = rotationAdjustRequested;
            wasUr10StyleFreeWristAdjustActive = freeWristAdjustRequested;
        }

        IsPositionClutched = gripHeld && poseValid && anchoredPoseTeleop.IsTracking;
        IsRotationClutched = IsPositionClutched;
        IsInputPoseValid = poseValid;
        IsFineControlActive = false;
        IsFinePositionControlActive = false;
        IsInsertModeActive = false;
        IsPrecisionModifierHeld = false;
        isSecondaryPoseRotationActive = false;
        requestedTeleopMode = Ur5TeleopMode.Free;
        wasUr10StyleGripHeld = gripHeld;

        if (IsPositionClutched)
        {
            UpdateUr10StyleHandStopHold(
                ur10StyleCommandPositionWorld,
                ur10StyleCommandRotationWorld,
                Time.deltaTime);
        }

        continuous6DofController.Pause();
        latestContinuous6DofStep = CreateContinuousInactiveStep(
            IsCommandActive ? Ur5TeleopControllerState.Clutched : Ur5TeleopControllerState.Idle,
            Ur5Continuous6DofFaultReason.None);
        rawBaseLinearVelocity = Vector3.zero;
        rawBaseAngularVelocity = Vector3.zero;
        limitedBaseLinearVelocity = Vector3.zero;
        limitedBaseAngularVelocity = Vector3.zero;
        filteredBaseLinearVelocity = Vector3.zero;
        filteredBaseAngularVelocity = Vector3.zero;
    }

    private void UpdateUr10StyleHandStopHold(
        Vector3 rawPositionWorld,
        Quaternion rawRotationWorld,
        float deltaTime)
    {
        if (!freezeRobotWhenPositionHandStops)
        {
            ResetUr10StyleHandStopHold(rawPositionWorld, rawRotationWorld);
            return;
        }

        if (!hasPositionMotionReference)
        {
            ResetUr10StyleHandStopHold(rawPositionWorld, rawRotationWorld);
            return;
        }

        float positionDelta = Vector3.Distance(positionMotionReferenceWorld, rawPositionWorld);
        float rotationDelta = Quaternion.Angle(positionMotionReferenceRotation, rawRotationWorld);
        float positionEpsilon = Mathf.Max(0.0f, controllerMotionEpsilonMeters);
        const float rotationEpsilonDegrees = 1.25f;
        if (positionDelta >= positionEpsilon || rotationDelta >= rotationEpsilonDegrees)
        {
            if (positionHandStopHoldActive)
            {
                FreezeUr10StyleAtCurrentPose(rawPositionWorld, rawRotationWorld);
            }

            ResetUr10StyleHandStopHold(rawPositionWorld, rawRotationWorld);
            return;
        }

        positionHandStillSeconds += Mathf.Max(0.0f, deltaTime);
        if (positionHandStopHoldActive
            || positionHandStillSeconds < Mathf.Max(0.0f, controllerStopHoldSeconds))
        {
            return;
        }

        FreezeUr10StyleAtCurrentPose(rawPositionWorld, rawRotationWorld);
        positionHandStopHoldActive = true;
    }

    private void ResetUr10StyleHandStopHold()
    {
        hasPositionMotionReference = false;
        positionHandStillSeconds = 0.0f;
        positionHandStopHoldActive = false;
    }

    private void ResetUr10StyleHandStopHold(Vector3 rawPositionWorld, Quaternion rawRotationWorld)
    {
        positionMotionReferenceWorld = rawPositionWorld;
        positionMotionReferenceRotation = rawRotationWorld;
        hasPositionMotionReference = true;
        positionHandStillSeconds = 0.0f;
        positionHandStopHoldActive = false;
    }

    private void FreezeUr10StyleAtCurrentPose(Vector3 rawPositionWorld, Quaternion rawRotationWorld)
    {
        if (tcpPreviewTarget == null)
        {
            return;
        }

        Quaternion holdRotation = IsUr10StyleRotationLocked()
            ? GetUr10StyleLockedToolRotation()
            : tcpPreviewTarget.rotation;
        tcpFollower?.FreezeAtCurrentPose();
        Vector3 holdPosition = GetActualToolPosition();
        tcpPreviewTarget.SetPositionAndRotation(holdPosition, holdRotation);
        ur10StyleLockedToolRotation = holdRotation;
        hasUr10StyleLockedToolRotation = true;
        anchoredPoseTeleop.Rebase(
            rawPositionWorld,
            rawRotationWorld,
            holdPosition,
            holdRotation);
        relativePoseCommandFilter.Reset(holdPosition, holdRotation);
        LogicalCommandPosition = holdPosition;
        LogicalCommandRotation = holdRotation;
        ConstrainedCommandPosition = holdPosition;
        FilteredCommandPosition = holdPosition;
        FilteredCommandRotation = holdRotation;
        IsPreviewLeadLimited = false;
        previewLinearVelocity = Vector3.zero;
        previewAngularVelocityDegrees = Vector3.zero;
    }

    private void SetUr10StylePreviewTargetImmediate(
        Vector3 position,
        Quaternion rotation,
        string writeSource)
    {
        if (tcpPreviewTarget == null)
        {
            return;
        }

        tcpPreviewTarget.SetPositionAndRotation(position, rotation);
        anchoredPoseTeleop.SetCommandPose(position, rotation);
        relativePoseCommandFilter.Reset(position, rotation);
        LogicalCommandPosition = position;
        LogicalCommandRotation = rotation;
        ConstrainedCommandPosition = position;
        FilteredCommandPosition = position;
        FilteredCommandRotation = rotation;
        IsPreviewLeadLimited = false;
        previewLinearVelocity = Vector3.zero;
        previewAngularVelocityDegrees = Vector3.zero;
        targetWriteMonitor?.RecordWrite(writeSource);
    }

    private bool IsUr10StyleRotationLocked()
    {
        return !ur10StyleRotationAdjustActive && !ur10StyleFreeWristAdjustActive;
    }

    private bool IsUr10StyleRotationAdjustmentRequested()
    {
        return ShouldAdjustUr10StyleRotation(
            rotationInputMode,
            positionDevice.isValid && ReadPrimaryButton(positionDevice),
            secondaryButtonPressed: false);
    }

    private bool IsUr10StyleFreeWristRotationAdjustmentRequested()
    {
        return ShouldAdjustUr10StyleFreeWristRotation(
            rotationInputMode,
            enableRightSecondaryFreeWristPoseControl,
            positionDevice.isValid && ReadPrimaryButton(positionDevice),
            positionDevice.isValid && ReadSecondaryButton(positionDevice));
    }

    public static bool ShouldAdjustUr10StyleRotation(
        RotationInputMode mode,
        bool primaryButtonPressed,
        bool secondaryButtonPressed)
    {
        _ = secondaryButtonPressed;

        if (mode == RotationInputMode.ControllerPoseDelta)
        {
            return true;
        }

        return mode == RotationInputMode.Locked && primaryButtonPressed;
    }

    public static bool ShouldAdjustUr10StyleFreeWristRotation(
        RotationInputMode mode,
        bool enableFreeWristControl,
        bool primaryButtonPressed,
        bool secondaryButtonPressed)
    {
        _ = primaryButtonPressed;
        return enableFreeWristControl
            && mode == RotationInputMode.Locked
            && secondaryButtonPressed;
    }

    public static Quaternion ApplyFreeWristControllerRotation(
        Quaternion startGraspRotation,
        Quaternion controllerRotationDelta,
        float rotationScale)
    {
        float scale = Mathf.Max(0.0f, rotationScale);
        Quaternion scaledDelta = Mathf.Approximately(scale, 1.0f)
            ? controllerRotationDelta
            : Quaternion.SlerpUnclamped(Quaternion.identity, controllerRotationDelta, scale);
        return NormalizeQuaternion(scaledDelta * startGraspRotation);
    }

    private void RebaseUr10StyleAtCommandPose(Vector3 inputPositionWorld, Quaternion inputRotationWorld)
    {
        if (tcpPreviewTarget == null)
        {
            return;
        }

        anchoredPoseTeleop.Rebase(
            inputPositionWorld,
            inputRotationWorld,
            tcpPreviewTarget.position,
            tcpPreviewTarget.rotation);
        relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
    }

    private void CaptureUr10StyleRotationAdjustReference()
    {
        if (tcpPreviewTarget == null)
        {
            hasUr10StyleRotationAdjustReference = false;
            return;
        }

        ur10StyleRotationAdjustStartControllerRotation = ur10StyleCommandRotationWorld;
        ur10StyleRotationAdjustStartGraspWorldRotation = GetRightARotationAdjustStartGraspRotation();
        hasUr10StyleRotationAdjustReference = true;
    }

    private void CaptureUr10StyleFreeWristReference()
    {
        if (tcpPreviewTarget == null)
        {
            hasUr10StyleFreeWristReference = false;
            return;
        }

        ur10StyleFreeWristStartControllerRotation = ur10StyleCommandRotationWorld;
        ur10StyleFreeWristStartGraspWorldRotation = GetPreviewGraspRotation(tcpPreviewTarget.rotation);
        hasUr10StyleFreeWristReference = true;
    }

    private Quaternion GetRightARotationAdjustStartGraspRotation()
    {
        if (!useRobotBaseForwardForRightAPose)
        {
            return GetPreviewGraspRotation(tcpPreviewTarget.rotation);
        }

        Vector3 yawReference = robotBaseFrame != null
            ? robotBaseFrame.forward
            : Vector3.forward;
        yawReference = Vector3.ProjectOnPlane(yawReference, Vector3.down);
        if (yawReference.sqrMagnitude < 0.0001f)
        {
            yawReference = Vector3.forward;
        }

        yawReference = Quaternion.AngleAxis(
            rightADefaultJawYawOffsetDegrees,
            Vector3.down) * yawReference.normalized;
        return Quaternion.LookRotation(Vector3.down, yawReference.normalized);
    }

    private Quaternion GetUr10StyleRotationAdjustToolRotation()
    {
        if (!hasUr10StyleRotationAdjustReference)
        {
            CaptureUr10StyleRotationAdjustReference();
        }

        if (!hasUr10StyleRotationAdjustReference)
        {
            return tcpPreviewTarget != null ? tcpPreviewTarget.rotation : Quaternion.identity;
        }

        float yawDegrees = CalculateHorizontalControllerTwistDegrees(
            ur10StyleRotationAdjustStartControllerRotation,
            ur10StyleCommandRotationWorld);
        return BuildDownwardGraspYawRotation(
            ur10StyleRotationAdjustStartGraspWorldRotation,
            yawDegrees);
    }

    private Quaternion GetUr10StyleFreeWristAdjustToolRotation()
    {
        if (!hasUr10StyleFreeWristReference)
        {
            CaptureUr10StyleFreeWristReference();
        }

        if (!hasUr10StyleFreeWristReference)
        {
            return tcpPreviewTarget != null ? tcpPreviewTarget.rotation : Quaternion.identity;
        }

        Quaternion controllerRotationDelta = NormalizeQuaternion(ur10StyleCommandRotationWorld)
            * Quaternion.Inverse(NormalizeQuaternion(ur10StyleFreeWristStartControllerRotation));
        Quaternion desiredGraspRotation = ApplyFreeWristControllerRotation(
            ur10StyleFreeWristStartGraspWorldRotation,
            controllerRotationDelta,
            rightSecondaryFreeWristRotationScale);
        return tcpFollower != null
            ? tcpFollower.GetToolRotationForGraspRotation(desiredGraspRotation)
            : desiredGraspRotation;
    }

    private Quaternion GetUr10StyleGripStartToolRotation()
    {
        if (ur10StyleRotationAdjustActive)
        {
            return GetUr10StyleRotationAdjustToolRotation();
        }

        if (IsUr10StyleRotationLocked() && hasPersistentOrientationTarget)
        {
            return persistentOrientationTarget;
        }

        return GetActualToolRotation();
    }

    private Quaternion GetUr10StyleLockedToolRotation()
    {
        return hasUr10StyleLockedToolRotation
            ? ur10StyleLockedToolRotation
            : GetUr10StyleGripStartToolRotation();
    }

    private void UpdateUr10StyleCommandPose(
        Vector3 positionWorld,
        Quaternion rotationWorld,
        bool resetRotationGate)
    {
        // The caller has already applied the current controller-position gate.
        // Re-reading filteredControllerPositionWorld here can use stale pre-Grip
        // state during clutch startup and create a false TCP jump while the hand
        // is visually still.
        ur10StyleCommandPositionWorld = positionWorld;
        if (resetRotationGate || !ur10StyleRotationNoiseGate.IsInitialized)
        {
            ur10StyleRotationNoiseGate.Reset(rotationWorld);
        }

        ur10StyleCommandRotationWorld = ur10StyleRotationNoiseGate.Filter(
            rotationWorld,
            angularDeadbandDegrees);
        hasUr10StyleCommandPose = true;
    }

    private void ResetUr10StyleCommandPoseGate()
    {
        hasUr10StyleCommandPose = false;
        ur10StyleRotationNoiseGate.Clear();
    }

    private bool IsRobotOutputReady()
    {
        return !sendToSpeedlClient
            || speedlClient == null
            || !speedlClient.enableRealRobotOutput
            || speedlClient.IsConnected;
    }

    private bool IsTeleopMotionAllowed()
    {
        if (enableContinuous6DofClutch)
        {
            return latestContinuous6DofStep.State == Ur5TeleopControllerState.Clutched;
        }

        return !enableThreeModeController
            || latestClutchModeStep.State == Ur5TeleopControllerState.Clutched;
    }

    private void CaptureClutchOrigins(Vector3 positionWorld, Quaternion rotationWorld)
    {
        if (IsPositionClutched && !wasPositionClutched)
        {
            positionNeutralWorldPosition = positionWorld;
            if (tcpPreviewTarget != null)
            {
                // The upstream strategy anchors the input to the measured tool
                // pose, never to a potentially lagging command target.
                Vector3 actualToolPosition = GetActualToolPosition();
                Quaternion actualToolRotation = GetActualToolRotation();
                positionClutchStartTargetWorldPosition = tcpPreviewTarget.position;
                positionClutchStartActualToolWorldPosition = actualToolPosition;
                relativePoseClutchMapper.Begin(
                    positionWorld,
                    latestRotationValid ? latestRotationWorld : Quaternion.identity,
                    actualToolPosition,
                    actualToolRotation);
                anchoredPoseTeleop.Resume(
                    positionWorld,
                    latestRotationValid ? latestRotationWorld : Quaternion.identity,
                    actualToolPosition,
                    actualToolRotation);
                stabilizedPreviewPosition = actualToolPosition;
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
                    tcpFollower.HoldTargetRotationAtCurrentGraspFrame("QuestTeleop");
                    persistentOrientationTarget = tcpPreviewTarget.rotation;
                    hasPersistentOrientationTarget = true;
                }

                positionOrientationLock = persistentOrientationTarget;
                hasPositionOrientationLock = true;
            }
        }

        if (!IsPositionClutched)
        {
            relativePoseClutchMapper.End();
            anchoredPoseTeleop.Pause();
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
        if (!IsTeleopMotionAllowed())
        {
            rawBaseLinearVelocity = Vector3.zero;
            rawBaseAngularVelocity = Vector3.zero;
            return;
        }

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
        if (!IsTeleopMotionAllowed())
        {
            limitedBaseLinearVelocity = Vector3.zero;
            limitedBaseAngularVelocity = Vector3.zero;
            filteredBaseLinearVelocity = Vector3.zero;
            filteredBaseAngularVelocity = Vector3.zero;
            IsWorkspaceLimited = false;
            return;
        }

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
        Ur5TeleopModeConfig config = GetActiveTeleopModeConfig();
        float deadband = enableThreeModeController ? config.DeadbandMeters : linearDeadbandMeters;
        float gain = enableThreeModeController ? linearSpeedGain * config.TranslationGain : linearSpeedGain;
        Vector3 velocity = ApplyVectorDeadband(baseDelta, deadband) * Mathf.Max(0.0f, gain);
        velocity = ApplyActiveBaseTranslationConstraint(velocity);
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
        Ur5TeleopModeConfig config = GetActiveTeleopModeConfig();
        float gain = enableThreeModeController ? angularSpeedGain * config.RotationGain : angularSpeedGain;
        Vector3 angularVelocity = baseAxis.normalized * signedAngleRadians * Mathf.Max(0.0f, gain);
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
            * GetActiveJoystickYawSpeedDegreesPerSecond() * Mathf.Deg2Rad * fineMultiplier;
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

        if (UsesUr10StyleAnchoredPoseClutch)
        {
            ApplyUr10StyleAnchoredPosePreview();
            return;
        }

        if (ApplyLeftPrimarySnapDownPreview())
        {
            return;
        }

        if (enableContinuous6DofClutch)
        {
            ApplyContinuous6DofPreview(deltaTime);
            return;
        }

        if (unityPreviewMode == UnityPreviewMode.RelativePoseTarget)
        {
            ApplyRelativePosePreview(deltaTime);
            return;
        }

        ApplyVelocityIntegrationPreview(deltaTime);
    }

    private void ApplyUr10StyleAnchoredPosePreview()
    {
        if (!IsPositionClutched
            || !IsRotationClutched
            || !latestPositionValid
            || !latestRotationValid
            || !hasUr10StyleCommandPose
            || !anchoredPoseTeleop.IsTracking)
        {
            // Release preserves the last target. The strategy is intentionally
            // not re-anchored or written here.
            LogicalCommandPosition = tcpPreviewTarget.position;
            LogicalCommandRotation = tcpPreviewTarget.rotation;
            FilteredCommandPosition = tcpPreviewTarget.position;
            FilteredCommandRotation = tcpPreviewTarget.rotation;
            ConstrainedCommandPosition = tcpPreviewTarget.position;
            IsPreviewLeadLimited = false;
            return;
        }

        Vector3 requestedPosition;
        Quaternion requestedRotation;
        bool hasRequestedPose;
        if (ur10StyleFreeWristAdjustActive)
        {
            Quaternion requestedToolRotation = GetUr10StyleFreeWristAdjustToolRotation();
            hasRequestedPose = anchoredPoseTeleop.TryGetRequestedPose(
                ur10StyleCommandPositionWorld,
                ur10StyleCommandRotationWorld,
                GetUr10StylePositionMapping(),
                requestedToolRotation,
                out requestedPosition,
                out requestedRotation);
        }
        else if (IsUr10StyleRotationLocked())
        {
            hasRequestedPose = anchoredPoseTeleop.TryGetRequestedPose(
                ur10StyleCommandPositionWorld,
                ur10StyleCommandRotationWorld,
                GetUr10StylePositionMapping(),
                GetUr10StyleLockedToolRotation(),
                out requestedPosition,
                out requestedRotation);
        }
        else if (rotationInputMode == RotationInputMode.Locked)
        {
            Quaternion requestedToolRotation = GetUr10StyleRotationAdjustToolRotation();
            hasRequestedPose = anchoredPoseTeleop.TryGetRequestedPose(
                ur10StyleCommandPositionWorld,
                ur10StyleCommandRotationWorld,
                GetUr10StylePositionMapping(),
                requestedToolRotation,
                out requestedPosition,
                out requestedRotation);
        }
        else
        {
            hasRequestedPose = anchoredPoseTeleop.TryGetRequestedPose(
                ur10StyleCommandPositionWorld,
                ur10StyleCommandRotationWorld,
                GetUr10StylePositionMapping(),
                out requestedPosition,
                out requestedRotation);
        }

        if (!hasRequestedPose)
        {
            anchoredPoseTeleop.Pause();
            return;
        }

        LogicalCommandPosition = requestedPosition;
        LogicalCommandRotation = requestedRotation;
        bool workspaceConstrained = false;
        if (clampPreviewWithWorkspaceLimiter && workspaceLimiter != null)
        {
            Vector3 constrainedPosition = workspaceLimiter.ClampWorldPosition(requestedPosition);
            workspaceConstrained = (constrainedPosition - requestedPosition).sqrMagnitude > 0.0000000001f;
            requestedPosition = constrainedPosition;
        }

        IsWorkspaceLimited = workspaceConstrained;
        requestedPosition = LimitPreviewLeadToActualTcp(requestedPosition);
        bool leadConstrained = IsPreviewLeadLimited;

        ConstrainedCommandPosition = requestedPosition;
        if (!anchoredPoseTeleop.FilterRequestedPose(
            requestedPosition,
            requestedRotation,
            GetAnchoredPoseSmoothingStep(),
            out Vector3 filteredPosition,
            out Quaternion filteredRotation))
        {
            anchoredPoseTeleop.Pause();
            return;
        }

        if (workspaceConstrained || leadConstrained)
        {
            // A hard workspace projection must also consume the corresponding
            // controller overtravel. The same applies to the dynamic lead
            // window: once the requested pose is projected near the actual TCP,
            // the hand anchor must be updated so reverse motion responds
            // immediately instead of first cancelling hidden backlog.
            anchoredPoseTeleop.RebaseInputAnchorPreservingCommand(
                ur10StyleCommandPositionWorld,
                ur10StyleCommandRotationWorld,
                requestedPosition,
                requestedRotation);
        }

        FilteredCommandPosition = filteredPosition;
        FilteredCommandRotation = filteredRotation;
        tcpPreviewTarget.SetPositionAndRotation(filteredPosition, filteredRotation);
        targetWriteMonitor?.RecordWrite("QuestTeleopUr10StyleAnchoredPose");
    }

    private Vector3 GetUr10StylePositionMapping()
    {
        if (ur10StyleRotationAdjustActive || ur10StyleFreeWristAdjustActive)
        {
            // Grip+A and Grip+B are orientation-only. Controller wrist arcs must
            // not drag the TCP position while attitude is being adjusted.
            return Vector3.zero;
        }

        if (enableThreeModeController && IsPositionClutched)
        {
            return Vector3.one * GetActiveTeleopModeConfig().TranslationGain;
        }

        return Vector3.one * Mathf.Max(0.0f, normalPositionScale);
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
        if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite("QuestTeleopVelocityIntegration");
        }
    }

    private void ApplyRelativePosePreview(float deltaTime)
    {
        if (!IsCommandActive || !IsTeleopMotionAllowed())
        {
            // Grip release is an ownership boundary. Do not write TcpTarget
            // here: the follower performs one explicit safe freeze instead.
            LogicalCommandPosition = tcpPreviewTarget.position;
            LogicalCommandRotation = tcpPreviewTarget.rotation;
            FilteredCommandPosition = tcpPreviewTarget.position;
            FilteredCommandRotation = tcpPreviewTarget.rotation;
            ConstrainedCommandPosition = tcpPreviewTarget.position;
            IsPreviewLeadLimited = false;
            return;
        }

        Vector3 requestedPosition = tcpPreviewTarget.position;
        Quaternion requestedRotation = tcpPreviewTarget.rotation;

        if (hasPositionOrientationLock && IsPositionClutched && !IsRotationClutched)
        {
            // 右手只允许改变位置；此处保持的是完整旋转，不允许任何偏航自转。
            requestedRotation = positionOrientationLock;
        }
        else if (IsRotationClutched
            && isSecondaryPoseRotationActive
            && latestRotationValid
            && latestRotationControllerPositionValid)
        {
            // 左手 Grip + Y：仅写竖直下抓姿态中的偏航；右手同时 Grip 时位置仍由右手写入。
            requestedRotation = CalculateSecondaryPoseYawRotation();
            persistentOrientationTarget = requestedRotation;
            hasPersistentOrientationTarget = true;
        }
        else if (IsRotationClutched && IsJoystickRotationMode())
        {
            // Integrate from the logical orientation rather than the filtered
            // TcpTarget. Feeding the filtered output back into the integrator
            // is a second servo loop and was the direct cause of stick wobble.
            Quaternion rotationReference = hasPersistentOrientationTarget
                ? persistentOrientationTarget
                : tcpPreviewTarget.rotation;
            requestedRotation = CalculateJoystickPreviewRotation(rotationReference, deltaTime);
            persistentOrientationTarget = requestedRotation;
            hasPersistentOrientationTarget = true;
        }
        else if (IsRotationClutched && latestRotationValid)
        {
            requestedRotation = CalculateRelativePreviewRotation(latestRotationWorld);
        }
        else if (hasPersistentOrientationTarget)
        {
            requestedRotation = persistentOrientationTarget;
        }

        if (IsPositionClutched && latestPositionValid)
        {
            if (useAnchoredPoseTeleopStrategy)
            {
                if (!anchoredPoseTeleop.TryGetRequestedPose(
                    latestPositionWorld,
                    latestRotationValid ? latestRotationWorld : Quaternion.identity,
                    Vector3.one * GetActivePositionScale(),
                    requestedRotation,
                    out requestedPosition,
                    out requestedRotation))
                {
                    anchoredPoseTeleop.Resume(
                        latestPositionWorld,
                        latestRotationValid ? latestRotationWorld : Quaternion.identity,
                        GetActualToolPosition(),
                        GetActualToolRotation());
                    anchoredPoseTeleop.TryGetRequestedPose(
                        latestPositionWorld,
                        latestRotationValid ? latestRotationWorld : Quaternion.identity,
                        Vector3.one * GetActivePositionScale(),
                        requestedRotation,
                        out requestedPosition,
                        out requestedRotation);
                }
            }
            else
            {
                relativePoseClutchMapper.TryMap(
                    latestPositionWorld,
                    latestRotationValid ? latestRotationWorld : Quaternion.identity,
                    GetActivePositionScale(),
                    GetRelativePreviewRotationScale(),
                    out requestedPosition,
                    out _);
            }
        }

        requestedPosition = ApplyActiveModePositionConstraints(requestedPosition);
        LogicalCommandPosition = requestedPosition;
        LogicalCommandRotation = requestedRotation;
        if (clampPreviewWithWorkspaceLimiter && workspaceLimiter != null)
        {
            requestedPosition = workspaceLimiter.ClampWorldPosition(requestedPosition);
        }

        ConstrainedCommandPosition = requestedPosition;
        IsPreviewLeadLimited = false;
        Vector3 nextPosition;
        Quaternion nextRotation;
        if (useAnchoredPoseTeleopStrategy && IsPositionClutched)
        {
            anchoredPoseTeleop.FilterRequestedPose(
                requestedPosition,
                requestedRotation,
                GetAnchoredPoseSmoothingStep(),
                out nextPosition,
                out nextRotation);
        }
        else
        {
            // Left-stick-only rotation still uses exactly one command filter.
            float step = GetAnchoredPoseSmoothingStep();
            nextPosition = tcpPreviewTarget.position;
            nextRotation = Quaternion.Slerp(tcpPreviewTarget.rotation, requestedRotation, step);
        }

        FilteredCommandPosition = nextPosition;
        FilteredCommandRotation = nextRotation;
        tcpPreviewTarget.SetPositionAndRotation(nextPosition, nextRotation);
        if (targetWriteMonitor != null)
        {
            targetWriteMonitor.RecordWrite("QuestTeleopAnchoredPose");
        }
    }

    private void ApplyContinuous6DofPreview(float deltaTime)
    {
        if (tcpPreviewTarget == null
            || latestContinuous6DofStep.State != Ur5TeleopControllerState.Clutched
            || !latestContinuous6DofStep.IsMotionCommandActive)
        {
            return;
        }

        Vector3 requestedPosition = latestContinuous6DofStep.TargetPosition;
        Quaternion requestedRotation = latestContinuous6DofStep.TargetRotation;
        LogicalCommandPosition = requestedPosition;
        LogicalCommandRotation = requestedRotation;
        IsPreviewLeadLimited = false;

        if (clampPreviewWithWorkspaceLimiter && workspaceLimiter != null)
        {
            requestedPosition = workspaceLimiter.ClampWorldPosition(requestedPosition);
        }

        ConstrainedCommandPosition = requestedPosition;
        if (!relativePoseCommandFilter.FilterByTimeConstants(
            requestedPosition,
            requestedRotation,
            continuous6DofConfig.PositionTimeConstantSeconds,
            continuous6DofConfig.RotationTimeConstantSeconds,
            deltaTime,
            continuous6DofConfig.MaxLinearSpeedMetersPerSecond,
            continuous6DofConfig.MaxAngularSpeedDegreesPerSecond,
            out Vector3 filteredPosition,
            out Quaternion filteredRotation))
        {
            continuous6DofController.Pause();
            latestContinuous6DofStep = CreateContinuousInactiveStep(
                Ur5TeleopControllerState.Paused,
                Ur5Continuous6DofFaultReason.None);
            return;
        }

        FilteredCommandPosition = filteredPosition;
        FilteredCommandRotation = filteredRotation;
        tcpPreviewTarget.SetPositionAndRotation(filteredPosition, filteredRotation);
        targetWriteMonitor?.RecordWrite("QuestTeleopContinuous6Dof");
    }

    private Ur5Continuous6DofStepResult CreateContinuousInactiveStep(
        Ur5TeleopControllerState state,
        Ur5Continuous6DofFaultReason faultReason)
    {
        return new Ur5Continuous6DofStepResult(
            state,
            faultReason,
            isMotionCommandActive: false,
            wasAnchoredThisStep: false,
            controllerDistanceMeters: 0.0f,
            controllerAngleDegrees: 0.0f,
            translationGain: 0.0f,
            rotationGain: 0.0f,
            mappedAngleDegrees: 0.0f,
            targetPosition: tcpPreviewTarget != null ? tcpPreviewTarget.position : Vector3.zero,
            targetRotation: tcpPreviewTarget != null ? tcpPreviewTarget.rotation : Quaternion.identity);
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

    private void UpdateLeftControllerSafetyPose(bool rightGripHeld)
    {
        EnsureLeftSafetyPoseController();
        bool safetyPoseValid = safetyDevice.isValid && tcpPreviewTarget != null;
        bool leftGripHeld = safetyPoseValid
            && ReadGripDeadman(safetyDevice, ref safetyGripLatched);
        bool primaryPressed = leftGripHeld
            && enableLeftPrimarySnapDown
            && safetyDevice.TryGetFeatureValue(
                CommonUsages.primaryButton,
                out bool primaryValue)
            && primaryValue;
        bool snapTargetReached = leftSafetyPoseController.State
                == Ur5LeftSafetyPoseState.SnapDownActive
            && tcpFollower != null
            && IsSafetySnapTargetReached(
                tcpFollower.PositionError,
                tcpFollower.RotationErrorDegrees,
                leftPrimarySnapPositionToleranceMeters,
                leftPrimarySnapRotationToleranceDegrees);

        Ur5LeftSafetyPoseStepResult safetyStep = leftSafetyPoseController.Step(
            new Ur5LeftSafetyPoseStepInput(
                rightGripHeld,
                safetyPoseValid,
                leftGripHeld,
                primaryPressed,
                snapTargetReached,
                tcpFollower != null && tcpFollower.IsReadyPoseActive,
                Time.deltaTime));

        if (safetyStep.RequestSnapDown)
        {
            CaptureLeftPrimarySnapDownTarget();
        }

        if (safetyStep.RequestReadyPose && enableLeftPrimaryReadyPose && tcpFollower != null)
        {
            hasLeftPrimarySnapTarget = false;
            tcpFollower.BeginReadyPose();
        }

        if (safetyStep.CancelReadyPose && tcpFollower != null)
        {
            tcpFollower.CancelReadyPose();
        }

        if (safetyStep.SnapTimedOut)
        {
            // 朝下目标不可达时只冻结一次当前测得姿态，不允许 IK 继续追赶旧目标。
            hasLeftPrimarySnapTarget = false;
            tcpFollower?.FreezeAtCurrentPose();
        }
        else if (snapTargetReached)
        {
            hasLeftPrimarySnapTarget = false;
        }

        // Y 是兼容的安全姿态保持入口，也必须读取独立左手设备。
        bool secondaryPressed = leftGripHeld
            && enableLeftSecondaryOrientationHold
            && !enableLeftSecondaryPoseRotation
            && safetyDevice.TryGetFeatureValue(
                CommonUsages.secondaryButton,
                out bool secondaryValue)
            && secondaryValue;
        if (secondaryPressed && !leftSecondaryWasPressed)
        {
            persistentOrientationTarget = tcpPreviewTarget.rotation;
            hasPersistentOrientationTarget = true;
            relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
        }

        IsSafetyOrientationHoldActive = secondaryPressed;
        leftSecondaryWasPressed = secondaryPressed;
    }

    private void CaptureLeftPrimarySnapDownTarget()
    {
        if (tcpPreviewTarget == null || tcpFollower == null)
        {
            hasLeftPrimarySnapTarget = false;
            return;
        }

        Vector3 yawReference = robotBaseFrame != null
            ? robotBaseFrame.forward
            : Vector3.forward;
        leftPrimarySnapTargetPosition = GetActualToolPosition();
        leftPrimarySnapTargetRotation = tcpFollower.GetToolRotationForGraspApproach(
            Vector3.down,
            yawReference);
        hasLeftPrimarySnapTarget = true;
    }

    private bool ApplyLeftPrimarySnapDownPreview()
    {
        if (SafetyPoseState != Ur5LeftSafetyPoseState.SnapDownActive
            || !hasLeftPrimarySnapTarget
            || tcpPreviewTarget == null)
        {
            return false;
        }

        Vector3 targetPosition = leftPrimarySnapTargetPosition;
        if (clampPreviewWithWorkspaceLimiter && workspaceLimiter != null)
        {
            targetPosition = workspaceLimiter.ClampWorldPosition(targetPosition);
        }

        // 安全朝下路径是本帧唯一写入源；位置保持在短按完成时的实际 TCP，
        // 姿态由物理抓取帧计算，不能再叠加右手相对四元数。
        LogicalCommandPosition = leftPrimarySnapTargetPosition;
        LogicalCommandRotation = leftPrimarySnapTargetRotation;
        ConstrainedCommandPosition = targetPosition;
        FilteredCommandPosition = targetPosition;
        FilteredCommandRotation = leftPrimarySnapTargetRotation;
        tcpPreviewTarget.SetPositionAndRotation(targetPosition, leftPrimarySnapTargetRotation);
        targetWriteMonitor?.RecordWrite("LeftSafetySnapDown");
        return true;
    }

    public static bool IsSafetySnapTargetReached(
        float positionErrorMeters,
        float rotationErrorDegrees,
        float positionToleranceMeters,
        float rotationToleranceDegrees)
    {
        return IsFinite(positionErrorMeters)
            && IsFinite(rotationErrorDegrees)
            && positionErrorMeters <= Mathf.Max(0.0f, positionToleranceMeters)
            && rotationErrorDegrees <= Mathf.Max(0.0f, rotationToleranceDegrees);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static Quaternion NormalizeQuaternion(Quaternion value)
    {
        float squaredMagnitude = value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w;
        if (!IsFinite(squaredMagnitude) || squaredMagnitude < 0.000001f)
        {
            return Quaternion.identity;
        }

        float inverseMagnitude = 1.0f / Mathf.Sqrt(squaredMagnitude);
        return new Quaternion(
            value.x * inverseMagnitude,
            value.y * inverseMagnitude,
            value.z * inverseMagnitude,
            value.w * inverseMagnitude);
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
        float localYDegrees = ApplyJoystickResponse(yawInput)
            * GetActiveJoystickYawSpeedDegreesPerSecond()
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
        if (enableThreeModeController && IsPositionClutched)
        {
            return GetActiveTeleopModeConfig().CommandFilterRetention;
        }

        return IsFineControlActive
            ? Mathf.Clamp(fineRelativePoseCommandFilterRetention, 0.0f, 0.95f)
            : Mathf.Clamp(relativePoseCommandFilterRetention, 0.0f, 0.95f);
    }

    private float GetAnchoredPoseSmoothingStep()
    {
        if (enableThreeModeController && IsPositionClutched)
        {
            return GetActiveTeleopModeConfig().PoseSmoothingStep;
        }

        return Mathf.Clamp01(IsPrecisionModifierHeld
            ? anchoredPosePrecisionSmoothingStep
            : anchoredPoseSmoothingStep);
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
    /// free/fine/insert 切换时重新捕获右手与当前命令的相对基准。
    /// 这样模式只改变后续位移分辨率或轴约束，不会在切换采样点产生目标跳变。
    /// </summary>
    private void RebasePositionClutchForModeChange(Vector3 positionWorld, Quaternion rotationWorld)
    {
        if (!IsPositionClutched || !latestPositionValid)
        {
            wasFinePositionControlActive = false;
            return;
        }

        bool modeChanged = enableThreeModeController
            ? wasPositionClutched && requestedTeleopMode != previousPositionTeleopMode
            : IsFinePositionControlActive != wasFinePositionControlActive;
        if (!modeChanged)
        {
            return;
        }

        positionNeutralWorldPosition = positionWorld;
        if (tcpPreviewTarget != null)
        {
            positionClutchStartTargetWorldPosition = tcpPreviewTarget.position;
            positionClutchStartActualToolWorldPosition = tcpPreviewTarget.position;
            relativePoseClutchMapper.Rebase(
                positionWorld,
                latestRotationValid ? rotationWorld : Quaternion.identity,
                tcpPreviewTarget.position,
                tcpPreviewTarget.rotation);
            anchoredPoseTeleop.Rebase(
                positionWorld,
                latestRotationValid ? rotationWorld : Quaternion.identity,
                tcpPreviewTarget.position,
                tcpPreviewTarget.rotation);
            stabilizedPreviewPosition = tcpPreviewTarget.position;
            hasStabilizedPreviewPosition = true;
            relativePoseCommandFilter.Reset(tcpPreviewTarget.position, tcpPreviewTarget.rotation);
        }

        positionMotionReferenceWorld = latestRawPositionWorld;
        hasPositionMotionReference = latestRawPositionValid;
        positionHandStillSeconds = 0.0f;
        positionHandStopHoldActive = false;

        wasFinePositionControlActive = IsFinePositionControlActive;
    }

    private float GetRelativePreviewRotationScale()
    {
        if (enableThreeModeController && IsPositionClutched)
        {
            return GetActiveTeleopModeConfig().RotationGain;
        }

        float scale = Mathf.Max(0.0f, relativePreviewRotationScale);
        return IsPrecisionModifierHeld
            ? scale * Mathf.Clamp01(fineAngularSpeedMultiplier)
            : scale;
    }

    private float GetActivePositionScale()
    {
        if (enableThreeModeController && IsPositionClutched)
        {
            return GetActiveTeleopModeConfig().TranslationGain;
        }

        if (enableAPrecisionModifier)
        {
            return IsPrecisionModifierHeld
                ? Mathf.Max(0.0f, precisionModifierPositionScale)
                : Mathf.Max(0.0f, normalPositionScale);
        }

        return Mathf.Max(0.0f, relativePreviewPositionScale);
    }

    private Ur5TeleopModeConfig GetActiveTeleopModeConfig()
    {
        switch (requestedTeleopMode)
        {
            case Ur5TeleopMode.Fine:
                return fineModeConfig.Sanitized();
            case Ur5TeleopMode.Insert:
                return insertModeConfig.Sanitized();
            default:
                return freeModeConfig.Sanitized();
        }
    }

    private float GetActiveJoystickYawSpeedDegreesPerSecond()
    {
        if (enableAPrecisionModifier)
        {
            return IsPrecisionModifierHeld
                ? Mathf.Max(0.0f, precisionJoystickYawSpeedDegreesPerSecond)
                : Mathf.Max(0.0f, normalJoystickYawSpeedDegreesPerSecond);
        }

        return Mathf.Max(0.0f, joystickYawSpeedDegreesPerSecond);
    }

    private Vector3 ApplyActiveModePositionConstraints(Vector3 requestedPosition)
    {
        if (!enableThreeModeController || !IsPositionClutched)
        {
            return requestedPosition;
        }

        Ur5TeleopModeConfig config = GetActiveTeleopModeConfig();
        Vector3 deltaFromClutch = requestedPosition - positionClutchStartActualToolWorldPosition;
        if (config.ConstrainToInsertAxis)
        {
            // Insert 轴按 robot base frame 配置，运行时转换到 world，避免场景父级旋转污染插入方向。
            deltaFromClutch = Vector3.Project(deltaFromClutch, GetInsertAxisWorld(config));
        }

        if (config.MaxLinearDeltaMeters > 0.0f)
        {
            deltaFromClutch = Vector3.ClampMagnitude(deltaFromClutch, config.MaxLinearDeltaMeters);
        }

        return positionClutchStartActualToolWorldPosition + deltaFromClutch;
    }

    private Vector3 GetInsertAxisWorld(Ur5TeleopModeConfig config)
    {
        Vector3 axis = config.InsertAxis.sqrMagnitude > 0.000001f
            ? config.InsertAxis.normalized
            : Vector3.forward;
        Vector3 worldAxis = robotBaseFrame != null ? robotBaseFrame.TransformDirection(axis) : axis;
        return worldAxis.sqrMagnitude > 0.000001f ? worldAxis.normalized : Vector3.forward;
    }

    private float ApplyJoystickResponse(float value)
    {
        if (Mathf.Abs(value) < 0.000001f)
        {
            return 0.0f;
        }

        return Mathf.Sign(value) * Mathf.Pow(Mathf.Abs(value), Mathf.Max(1.0f, joystickResponseExponent));
    }

    private Vector3 LimitPreviewLeadToActualTcp(Vector3 requestedPosition)
    {
        IsPreviewLeadLimited = false;
        if (!limitPreviewLeadToActualTcp
            || Mathf.Max(maximumPreviewLeadMeters, movingPreviewLeadMeters) <= 0.0f
            || tcpFollower == null
            || !tcpFollower.enabled)
        {
            return requestedPosition;
        }

        Vector3 actualTcpPosition = tcpFollower.ControlPointPosition;
        Vector3 lead = requestedPosition - actualTcpPosition;
        float maximumLead = GetActivePreviewLeadLimitMeters();
        IsPreviewLeadLimited = lead.sqrMagnitude > maximumLead * maximumLead;
        return actualTcpPosition + Vector3.ClampMagnitude(lead, maximumLead);
    }

    private float GetActivePreviewLeadLimitMeters()
    {
        float stopLead = Mathf.Max(0.0f, maximumPreviewLeadMeters);
        if (!UsesResponsiveMovingPreviewLead())
        {
            return stopLead;
        }

        return Mathf.Max(stopLead, movingPreviewLeadMeters);
    }

    private bool UsesResponsiveMovingPreviewLead()
    {
        return UsesUr10StyleAnchoredPoseClutch
            && IsPositionClutched
            && !positionHandStopHoldActive
            && movingPreviewLeadMeters > maximumPreviewLeadMeters;
    }

    private Vector3 GetActualToolPosition()
    {
        return tcpFollower != null && tcpFollower.enabled
            ? tcpFollower.ControlPointPosition
            : tcpPreviewTarget.position;
    }

    private Quaternion GetActualToolRotation()
    {
        if (tcpFollower == null || !tcpFollower.enabled)
        {
            return tcpPreviewTarget.rotation;
        }

        return tcpFollower.GetToolRotationForGraspRotation(tcpFollower.ActualGraspRotation);
    }

    private Vector3 ApplyAxisLocks(Vector3 value, bool allowX, bool allowY, bool allowZ)
    {
        return new Vector3(
            allowX ? value.x : 0.0f,
            allowY ? value.y : 0.0f,
            allowZ ? value.z : 0.0f);
    }

    private Vector3 ApplyActiveBaseTranslationConstraint(Vector3 baseVelocity)
    {
        if (!enableThreeModeController || !IsInsertModeActive)
        {
            return baseVelocity;
        }

        Ur5TeleopModeConfig config = GetActiveTeleopModeConfig();
        if (!config.ConstrainToInsertAxis)
        {
            return baseVelocity;
        }

        Vector3 baseAxis = config.InsertAxis.sqrMagnitude > 0.000001f
            ? config.InsertAxis.normalized
            : Vector3.forward;
        return Vector3.Project(baseVelocity, baseAxis);
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
            controllerPositionNoiseGate.Reset(rawPosition);
            hasFilteredControllerPosition = true;
            controllerPositionFilterIsSettling = false;
            return rawPosition;
        }

        if (!hasFilteredControllerPosition)
        {
            filteredControllerPositionWorld = rawPosition;
            controllerPositionOneEuroFilter.Reset(rawPosition);
            controllerPositionNoiseGate.Reset(rawPosition);
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

        // A radial noise gate is deliberately used instead of another Lerp.
        // It is stable while the controller is still, but does not create a
        // trailing target after the operator stops a deliberate movement.
        float jitterRadius = Mathf.Max(0.0f, controllerPositionJitterDeadbandMeters);
        controllerPositionFilterIsSettling = Vector3.Distance(rawPosition, filteredControllerPositionWorld) > jitterRadius;
        filteredControllerPositionWorld = controllerPositionNoiseGate.Filter(rawPosition, jitterRadius);
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

        float deadband = IsPrecisionModifierHeld
            ? finePreviewTargetDeadbandMeters
            : previewTargetDeadbandMeters;
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

    private bool ReadAPrecisionModifier(InputDevice device)
    {
        return enableAPrecisionModifier
            && device.isValid
            && device.TryGetFeatureValue(CommonUsages.primaryButton, out bool pressed)
            && pressed;
    }

    private bool ReadPrimaryButton(InputDevice device)
    {
        return device.isValid
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

    private void TryRefreshSafetyDevice()
    {
        safetyDevice = InputDevices.GetDeviceAtXRNode(safetyControllerNode);
        LogDeviceStatus(safetyDevice, "left safety", ref hasLoggedMissingSafetyDevice);
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

        if (targetWriteMonitor == null && tcpPreviewTarget != null)
        {
            targetWriteMonitor = tcpPreviewTarget.GetComponent<TcpTargetWriteMonitor>();
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

    private void EnsureLeftSafetyPoseController()
    {
        float readyHold = Mathf.Max(0.0f, leftPrimaryReadyPoseHoldSeconds);
        float snapTimeout = Mathf.Max(0.0f, leftPrimarySnapTimeoutSeconds);
        if (leftSafetyPoseController != null
            && Mathf.Approximately(configuredLeftPrimaryReadyPoseHoldSeconds, readyHold)
            && Mathf.Approximately(configuredLeftPrimarySnapTimeoutSeconds, snapTimeout))
        {
            return;
        }

        leftSafetyPoseController = new Ur5LeftSafetyPoseController(readyHold, snapTimeout);
        configuredLeftPrimaryReadyPoseHoldSeconds = readyHold;
        configuredLeftPrimarySnapTimeoutSeconds = snapTimeout;
        hasLeftPrimarySnapTarget = false;
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
        rightADefaultJawYawOffsetDegrees = Mathf.Clamp(
            rightADefaultJawYawOffsetDegrees,
            -180.0f,
            180.0f);
        rightSecondaryFreeWristRotationScale = Mathf.Clamp(
            rightSecondaryFreeWristRotationScale,
            0.10f,
            1.50f);
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
        relativePoseFilterReferenceRateHz = Mathf.Max(1.0f, relativePoseFilterReferenceRateHz);
        anchoredPoseSmoothingStep = Mathf.Clamp01(anchoredPoseSmoothingStep);
        anchoredPosePrecisionSmoothingStep = Mathf.Clamp01(anchoredPosePrecisionSmoothingStep);
        normalPositionScale = Mathf.Max(0.0f, normalPositionScale);
        precisionModifierPositionScale = Mathf.Max(0.0f, precisionModifierPositionScale);
        normalJoystickYawSpeedDegreesPerSecond = Mathf.Max(0.0f, normalJoystickYawSpeedDegreesPerSecond);
        precisionJoystickYawSpeedDegreesPerSecond = Mathf.Max(0.0f, precisionJoystickYawSpeedDegreesPerSecond);
        joystickResponseExponent = Mathf.Max(1.0f, joystickResponseExponent);
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
        leftPrimarySnapTimeoutSeconds = Mathf.Max(0.0f, leftPrimarySnapTimeoutSeconds);
        leftPrimarySnapPositionToleranceMeters = Mathf.Max(
            0.0f,
            leftPrimarySnapPositionToleranceMeters);
        leftPrimarySnapRotationToleranceDegrees = Mathf.Max(
            0.0f,
            leftPrimarySnapRotationToleranceDegrees);
        maximumPreviewLeadMeters = Mathf.Max(0.0f, maximumPreviewLeadMeters);
        movingPreviewLeadMeters = Mathf.Max(0.0f, movingPreviewLeadMeters);
        fineLinearSpeedMultiplier = Mathf.Clamp(fineLinearSpeedMultiplier, 0.1f, 1.0f);
        fineAngularSpeedMultiplier = Mathf.Clamp(fineAngularSpeedMultiplier, 0.1f, 1.0f);
        gripPressThreshold = Mathf.Clamp01(gripPressThreshold);
        gripReleaseThreshold = Mathf.Clamp(gripReleaseThreshold, 0.0f, gripPressThreshold);
        continuous6DofConfig = continuous6DofConfig.Sanitized();
        freeModeConfig = freeModeConfig.Sanitized();
        fineModeConfig = fineModeConfig.Sanitized();
        insertModeConfig = insertModeConfig.Sanitized();
    }
}

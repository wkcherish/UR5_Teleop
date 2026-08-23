using UnityEngine;

public class Ur5ControlBootstrap : MonoBehaviour
{
    public const float DefaultWorkspaceMinimumLocalHeightMeters = 0.015f;

    private static Ur5ControlBootstrap runtimeOwner;
    private bool hasConfiguredRuntime;
    [Header("Scene References")]
    public Transform robotRoot;
    public Transform tcpTarget;
    public bool createTcpTargetIfMissing = true;

    [Header("Legacy Target/IK Control")]
    public bool enableKeyboardControl = false;
    public bool enableQuest3Control = true;
    public bool enableTcpTargetFollower = true;

    [Header("Cartesian Velocity Teleop")]
    public bool enableCartesianVelocityTeleop = true;
    [Tooltip("Disable the Unity IK visual follower only after real robot feedback is driving the digital twin. Keep false during Unity-only testing so the arm model follows the velocity-driven TcpTarget.")]
    public bool disablePoseIkWhenVelocityTeleopEnabled = false;
    public bool addUrScriptSpeedlClient = true;

    [Header("Quest 平移速度配置")]
    [Tooltip("Compatibility switch for old scenes that need a minimum fast profile. Standard precision teleop keeps this off and uses the Inspector values exactly.")]
    public bool enforceFastQuestMotionProfile = false;
    [Tooltip("右手位移映射到 TCP 的比例。增大后相同手部移动距离会产生更大的 TCP 位移。")]
    [Range(0.5f, 4.5f)] public float questTranslationScale = 2.10f;
    [Tooltip("Unity 中 TCP 预览的最高平移速度（米/秒）。这不改变真机 RTDE 安全限速。")]
    [Range(0.05f, 0.85f)] public float questPreviewMaxLinearSpeed = 0.70f;
    [Tooltip("TcpTarget 相对实际两指中心允许的最大超前距离（米）。较大值更灵敏，但视觉超前也更明显。")]
    [Range(0.01f, 0.10f)] public float questMaximumPreviewLeadMeters = 0.040f;
    [Tooltip("右手确实在移动时允许的临时 TcpTarget 超前距离（米）。停手后仍回到 questMaximumPreviewLeadMeters。")]
    [Range(0.04f, 0.30f)] public float questMovingPreviewLeadMeters = 0.160f;
    [Tooltip("仅用于 Unity speedl 影子命令的线速度上限（米/秒）；真机实际限速由 Fedora 端安全配置决定。")]
    [Range(0.05f, 0.50f)] public float questCommandMaxLinearSpeed = 0.32f;
    [Tooltip("线速度变化上限（米/秒²）。增大后起停更快，仍保留平滑滤波。")]
    [Range(0.10f, 2.50f)] public float questCommandMaxLinearAcceleration = 1.60f;
    [Tooltip("每次 IK 更新允许的最大关节目标步长（度）。用于平衡机械臂响应速度与轨迹平滑度。")]
    [Range(0.50f, 6.00f)] public float questMaxJointStepDegrees = 3.85f;

    [Header("TCP 工作空间安全")]
    [Tooltip("TCP 目标在 UR5 基座/桌面局部坐标中的最低高度。保持略高于 0，允许低位抓取但避免目标穿到地面以下。")]
    [Range(0.0f, 0.10f)] public float workspaceMinimumLocalHeightMeters = DefaultWorkspaceMinimumLocalHeightMeters;

    [Header("真实机械臂 speedl 安全限速")]
    [Tooltip("真实 UR speedl 输出的线速度上限，独立于 Quest/Unity dry-run 的视觉跟随速度。首次上真机建议保持保守。")]
    [Range(0.005f, 0.10f)] public float realRobotMaxLinearSpeed = 0.035f;
    [Tooltip("真实 UR speedl 输出的角速度上限，单位 rad/s。")]
    [Range(0.05f, 0.60f)] public float realRobotMaxAngularSpeedRadiansPerSecond = 0.25f;
    [Tooltip("发送给 speedl 的运动加速度参数。")]
    [Range(0.02f, 0.50f)] public float realRobotCommandAcceleration = 0.12f;
    [Tooltip("停止时发送给 stopl/speedl 的加速度参数。")]
    [Range(0.05f, 0.80f)] public float realRobotStopAcceleration = 0.35f;
    [Tooltip("本地输出速度斜坡的线加速度上限，避免速度命令突然跃迁。")]
    [Range(0.02f, 0.30f)] public float realRobotMaxOutputLinearAcceleration = 0.10f;
    [Tooltip("本地输出速度斜坡的角加速度上限，避免姿态速度命令突然跃迁。")]
    [Range(0.10f, 2.00f)] public float realRobotMaxOutputAngularAcceleration = 0.70f;

    public float EffectiveQuestTranslationScale => GetFastProfileMinimum(questTranslationScale, 3.10f);
    public float EffectiveQuestPreviewMaxLinearSpeed => GetFastProfileMinimum(questPreviewMaxLinearSpeed, 0.70f);
    public float EffectiveQuestMaximumPreviewLeadMeters => GetFastProfileMinimum(questMaximumPreviewLeadMeters, 0.075f);
    public float EffectiveQuestMovingPreviewLeadMeters => GetFastProfileMinimum(questMovingPreviewLeadMeters, 0.160f);
    public float EffectiveQuestCommandMaxLinearSpeed => GetFastProfileMinimum(questCommandMaxLinearSpeed, 0.26f);
    public float EffectiveQuestCommandMaxLinearAcceleration => GetFastProfileMinimum(questCommandMaxLinearAcceleration, 1.20f);
    public float EffectiveQuestMaxJointStepDegrees => GetFastProfileMinimum(questMaxJointStepDegrees, 2.80f);

    [Header("Passthrough 显示")]
    [Tooltip("默认隐藏 Unity 虚拟手柄，直接使用 Passthrough 中可见的真实 Quest 手柄。不会影响控制输入或遥测。")]
    public bool showVirtualControllersInPassthrough = false;

    [Header("Quest UDP PC Teleop")]
    [Tooltip("Sends raw Quest controller telemetry to the DG-VLA PC. The Quest app never enables or commands the real robot directly.")]
    public bool enableQuestUdpShadowTelemetry;
    [Tooltip("Optional debug mode: when enabled, Quest only sends UDP telemetry and all Unity-side TCP/IK/gripper/speedl writers are disabled.")]
    public bool disableLocalTeleopWhenQuestUdpShadowTelemetry = false;
    [Tooltip("IP address of the DG-VLA capture computer. If this is filled, UDP sending is enabled even when the legacy shadow flag is false.")]
    public string questShadowReceiverHost = "";
    public int questShadowReceiverPort = 8080;
    public float questShadowSendRateHz = 100.0f;

    private void Awake()
    {
        if (runtimeOwner != null && runtimeOwner != this)
        {
            enabled = false;
            Debug.LogWarning("Disabled duplicate Ur5ControlBootstrap; the first bootstrap owns UR5 teleoperation configuration.");
            return;
        }

        runtimeOwner = this;
        if (hasConfiguredRuntime)
        {
            return;
        }

        hasConfiguredRuntime = true;
        if (robotRoot == null)
        {
            robotRoot = FindRobotRoot();
        }

        DisableLegacyUrdfImporterController();
        ResolveTcpTarget();
        ConfigureTcpTargetSafety();
        ConfigurePhysicsStabilizer();

        if (ShouldUseQuestUdpTelemetryOnlyMode())
        {
            DisableLocalTeleopWritersForQuestUdpBridge();
            ConfigureQuestUdpShadowTelemetry();
            ConfigureSpectatorCamera();
            ConfigureControllerVisualizer();
            ConfigureRecorder(null, null, null, null);
            return;
        }

        bool useVelocityTeleop = enableCartesianVelocityTeleop;
        if (!useVelocityTeleop
            && enableKeyboardControl
            && tcpTarget != null
            && tcpTarget.GetComponent<TcpTargetKeyboardController>() == null)
        {
            tcpTarget.gameObject.AddComponent<TcpTargetKeyboardController>();
        }

        if (useVelocityTeleop)
        {
            DisableQuestPoseController();
            DisableLegacyVrTeleoperationControllers();
            DisableKeyboardTargetController();
        }
        else if (enableQuest3Control && tcpTarget != null && tcpTarget.GetComponent<Quest3TcpTargetController>() == null)
        {
            tcpTarget.gameObject.AddComponent<Quest3TcpTargetController>();
        }

        Ur5ArticulationJointController jointController = ResolveJointController();
        Ur5JointTrajectoryPlayer trajectoryPlayer = ConfigureJointTrajectoryPlayer(jointController);
        ConfigureVelocityTeleop(useVelocityTeleop);
        ConfigureQuestUdpShadowTelemetry();

        Ur5TcpTargetFollower follower = null;
        bool disablePoseIkForThisRun = useVelocityTeleop
            && disablePoseIkWhenVelocityTeleopEnabled
            && IsRealRobotOutputEnabled();
        if (disablePoseIkForThisRun)
        {
            DisablePoseIkFollower();
        }
        else if (enableTcpTargetFollower && robotRoot != null && tcpTarget != null && jointController != null)
        {
            follower = ConfigureTcpTargetFollower(jointController, trajectoryPlayer);
        }
        ConfigureGripperController();
        Ur5GraspAssistController graspAssist = ConfigureGraspAssist(follower);
        ConfigureTeleopDiagnostics(follower, trajectoryPlayer, jointController);
        ConfigureSpectatorCamera();
        ConfigureControllerVisualizer();
        ConfigureRecorder(jointController, trajectoryPlayer, follower, graspAssist);
    }

    private void ConfigurePhysicsStabilizer()
    {
        Ur5PhysicsStabilizer stabilizer = GetComponent<Ur5PhysicsStabilizer>();
        if (stabilizer == null)
        {
            stabilizer = gameObject.AddComponent<Ur5PhysicsStabilizer>();
        }

        stabilizer.ConfigureRootAndStabilize(robotRoot);
    }

    private void ResolveTcpTarget()
    {
        if (tcpTarget == null)
        {
            GameObject foundTarget = GameObject.Find("TcpTarget");
            if (foundTarget != null)
            {
                tcpTarget = foundTarget.transform;
            }
        }

        if (tcpTarget != null || !createTcpTargetIfMissing)
        {
            return;
        }

        GameObject target = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        target.name = "TcpTarget";
        target.transform.position = new Vector3(0.5f, 0.3f, 0.3f);
        target.transform.localScale = Vector3.one * 0.05f;

        Renderer renderer = target.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = Color.red;
        }

        tcpTarget = target.transform;
    }

    private void ConfigureTcpTargetSafety()
    {
        if (tcpTarget == null)
        {
            return;
        }

        TcpTargetWorkspaceLimiter workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
        if (workspaceLimiter == null)
        {
            workspaceLimiter = tcpTarget.gameObject.AddComponent<TcpTargetWorkspaceLimiter>();
        }

        workspaceLimiter.robotRoot = robotRoot;
        // Teleop applies this constraint before publishing its final command.
        // Keeping the legacy LateUpdate writer disabled gives TcpTarget one
        // explicit owner during manual control.
        workspaceLimiter.constrainInLateUpdate = false;
        // Keep a small clearance above the calibrated base/ground plane while
        // still allowing the Robotiq pads to reach low tabletop targets.
        ApplyWorkspaceSafetyProfile(workspaceLimiter);

        TcpTargetCollisionGuard collisionGuard = tcpTarget.GetComponent<TcpTargetCollisionGuard>();
        if (collisionGuard == null)
        {
            collisionGuard = tcpTarget.gameObject.AddComponent<TcpTargetCollisionGuard>();
        }

        collisionGuard.robotRoot = robotRoot;
        collisionGuard.preventObstaclePenetration = false;

        TcpTargetWriteMonitor writeMonitor = tcpTarget.GetComponent<TcpTargetWriteMonitor>();
        if (writeMonitor == null)
        {
            writeMonitor = tcpTarget.gameObject.AddComponent<TcpTargetWriteMonitor>();
        }

        writeMonitor.enableDiagnostics = false;
    }

    private Ur5ArticulationJointController ResolveJointController()
    {
        if (robotRoot == null)
        {
            return null;
        }

        Ur5ArticulationJointController jointController = robotRoot.GetComponent<Ur5ArticulationJointController>();
        if (jointController == null)
        {
            jointController = robotRoot.gameObject.AddComponent<Ur5ArticulationJointController>();
        }

        jointController.robotRoot = robotRoot;
        ApplyStableJointDefaults(jointController);
        return jointController;
    }

    public static void ApplyStableJointDefaults(Ur5ArticulationJointController jointController)
    {
        jointController.stiffness = 12000.0f;
        // 提高物理驱动阻尼以消除末端停止后的弹簧感，不降低最高关节速度。
        jointController.damping = 7000.0f;
        jointController.forceLimit = 30000.0f;
        jointController.smoothDriveTargets = true;
        jointController.maxDriveSpeedDegreesPerSecond = 480.0f;
        jointController.maxDriveAccelerationDegreesPerSecondSquared = 18000.0f;
        jointController.driveTargetToleranceDegrees = 0.005f;
        // IK 目标和物理关节之间保留很小、受控的领先量，防止高刚度 Drive 形成弹簧摆动。
        jointController.limitDriveTargetLeadFromMeasuredJoint = true;
        // 仍以实际关节反馈限制命令领先量；适当放宽受控窗口，避免四层限幅
        // 叠加后出现“推一下才走、走一下又停”的卡顿感。
        jointController.maximumDriveTargetLeadDegrees = 6.0f;
        jointController.maximumWristDriveTargetLeadDegrees = 6.0f;
        jointController.readyPoseMaximumDriveTargetLeadDegrees = 16.0f;
        jointController.readyPoseMaximumWristDriveTargetLeadDegrees = 6.0f;
        jointController.useDirectJointStateForMeasuredTeleop = true;
        jointController.measuredStateTeleopMaximumJointStepDegrees = 3.85f;
        jointController.ApplyConfiguredDriveSettings();
    }

    /// <summary>
    /// Shared follower profile for Editor and auto-bootstrapped Quest scenes.
    /// Per-scene code may override only the configured IK joint step afterward.
    /// </summary>
    public static void ApplyDefaultFollowerProfile(Ur5TcpTargetFollower follower)
    {
        if (follower == null)
        {
            return;
        }

        follower.useGripperPadCenter = true;
        follower.usePadGeometryCenter = true;
        follower.usePhysicalGraspFrameForOrientation = true;
        follower.gripperBase = null;
        follower.positionTolerance = 0.008f;
        follower.maxJointStepDegrees = 3.85f;
        follower.maxJointSpeedDegreesPerSecond = 3.85f * Ur5AutoSceneBootstrap.TeleopControlRateHz;
        follower.minimumJointDeltaDegrees = 0.015f;
        // Active Quest teleoperation solves from measured joints. The
        // ArticulationBody drive is the only physical lead guard in that path;
        // these windows remain available for scripted/legacy follower users.
        follower.useMeasuredStateTeleopSolve = true;
        follower.maximumCommandLeadDegrees = 4.00f;
        follower.maximumWristCommandLeadDegrees = 8.00f;
        follower.useTimedJointAssignments = true;
        follower.jointAssignmentIntervalSeconds = 0.0f;
        follower.dlsDamping = 0.16f;
        follower.dlsOrientationWeight = 1.50f;
        follower.translationOrientationHoldWeight = 8.00f;
        follower.dlsGain = 0.95f;
        follower.proximalOrientationWeight = 0.05f;
        follower.wristPriorityDuringRotationAdjust = true;
        follower.rotationAdjustProximalJointWeight = 0.02f;
        follower.rotationAdjustPositionTaskWeight = 0.15f;
        follower.rotationAdjustDlsDampingMultiplier = 1.60f;
        // The anchored teleop path already has its one upstream-style target
        // smoothing stage. Do not add a second low-pass to DLS joint deltas.
        follower.jointDeltaSmoothing = 0.00f;
        follower.enableStationaryDlsDamping = false;
        follower.rotationToleranceDegrees = 0.03f;
        follower.rotationBlend = 0.70f;
        follower.maxWristStepDegrees = 4.00f;
        follower.maxWristSpeedDegreesPerSecond = 360.0f;
        follower.graspAssistPositionTolerance = 0.0015f;
        follower.graspAssistRotationToleranceDegrees = 0.35f;
        follower.enablePrecisionAssemblyTracking = false;
        follower.precisionAssemblyPositionTolerance = 0.0010f;
        follower.precisionAssemblyRotationToleranceDegrees = 0.25f;
        follower.precisionAssemblyTargetChangeEpsilonMeters = 0.00015f;
        follower.precisionAssemblyTargetChangeEpsilonDegrees = 0.04f;
        follower.precisionAssemblySettledPositionError = 0.0012f;
        follower.precisionAssemblySettledRotationErrorDegrees = 0.30f;
        follower.suppressRotationOnlyIkDuringPositionControl = true;
        follower.finishVelocityTargetAfterRelease = false;
        follower.snapTargetToActualPoseWhenQuestReleased = false;
        follower.velocityReleasePositionTolerance = 0.003f;
        follower.velocityReleaseRotationToleranceDegrees = 0.50f;
        follower.velocityReleaseSettleTimeoutSeconds = 2.0f;
        follower.safeReleaseMaximumResidualMeters = 0.003f;
        follower.safeReleaseMaximumResidualDegrees = 0.50f;
        follower.safeReleaseSettleTimeoutSeconds = 0.20f;
        follower.holdJointPoseWhenTargetSettled = false;
        follower.targetStationaryHoldSeconds = 0.12f;
        follower.targetStationaryPositionEpsilon = 0.0015f;
        follower.targetStationaryRotationEpsilonDegrees = 0.30f;
        follower.settledPositionError = 0.010f;
        follower.settledRotationErrorDegrees = 1.50f;
        follower.enableReadyPose = true;
        follower.readyPoseJointDegrees = new[] { 0.0f, -90.0f, 90.0f, -90.0f, -90.0f, 0.0f };
        follower.readyPoseMaxJointSpeedDegreesPerSecond = 320.0f;
        follower.readyPoseJointToleranceDegrees = 1.5f;
    }

    private Ur5JointTrajectoryPlayer ConfigureJointTrajectoryPlayer(Ur5ArticulationJointController jointController)
    {
        if (robotRoot == null || jointController == null)
        {
            return null;
        }

        Ur5JointTrajectoryPlayer trajectoryPlayer = robotRoot.GetComponent<Ur5JointTrajectoryPlayer>();
        if (trajectoryPlayer == null)
        {
            trajectoryPlayer = robotRoot.gameObject.AddComponent<Ur5JointTrajectoryPlayer>();
        }

        trajectoryPlayer.enabled = true;
        trajectoryPlayer.jointController = jointController;
        trajectoryPlayer.play = true;
        trajectoryPlayer.queueMode = Ur5JointTrajectoryPlayer.QueueMode.LatestOnly;
        trajectoryPlayer.jointAssignmentIntervalSeconds = 0.0f;
        trajectoryPlayer.applyDirectlyToDrive = false;
        trajectoryPlayer.clampToDriveLimits = true;
        trajectoryPlayer.maxQueuedWaypoints = 1;
        return trajectoryPlayer;
    }

    private Ur5TcpTargetFollower ConfigureTcpTargetFollower(
        Ur5ArticulationJointController jointController,
        Ur5JointTrajectoryPlayer trajectoryPlayer)
    {
        Ur5TcpTargetFollower follower = robotRoot.GetComponent<Ur5TcpTargetFollower>();
        if (follower == null)
        {
            follower = robotRoot.gameObject.AddComponent<Ur5TcpTargetFollower>();
        }

        follower.enabled = true;
        follower.robotRoot = robotRoot;
        follower.tcpTarget = tcpTarget;
        follower.jointController = jointController;
        follower.trajectoryPlayer = trajectoryPlayer;
        follower.targetWriteMonitor = tcpTarget.GetComponent<TcpTargetWriteMonitor>();
        follower.questController = tcpTarget != null
            ? tcpTarget.GetComponent<Quest3TcpTargetController>()
            : null;
        follower.velocityTeleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        follower.graspAssist = GetComponent<Ur5GraspAssistController>();
        follower.pauseIkWhenVelocityTeleopIdle = follower.velocityTeleop != null;
        ApplyDefaultFollowerProfile(follower);
        follower.maxJointStepDegrees = EffectiveQuestMaxJointStepDegrees;
        follower.maxJointSpeedDegreesPerSecond = EffectiveQuestMaxJointStepDegrees
            * Ur5AutoSceneBootstrap.TeleopControlRateHz;

        Ur5ActualTcpMarker actualMarker = tcpTarget.GetComponent<Ur5ActualTcpMarker>();
        if (actualMarker == null)
        {
            actualMarker = tcpTarget.gameObject.AddComponent<Ur5ActualTcpMarker>();
        }

        actualMarker.enabled = true;
        actualMarker.follower = follower;
        actualMarker.hideCommandTargetRenderer = true;
        actualMarker.markerDiameter = 0.035f;
        return follower;
    }

    private void ConfigureVelocityTeleop(bool useVelocityTeleop)
    {
        Ur5CartesianVelocityTeleopController velocityTeleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        if (!useVelocityTeleop)
        {
            if (velocityTeleop != null)
            {
                velocityTeleop.enabled = false;
            }

            return;
        }

        if (velocityTeleop == null)
        {
            velocityTeleop = gameObject.AddComponent<Ur5CartesianVelocityTeleopController>();
        }

        velocityTeleop.enabled = true;
        velocityTeleop.robotBaseFrame = robotRoot;
        velocityTeleop.xrOrigin = FindXrOrigin();
        velocityTeleop.tcpPreviewTarget = tcpTarget;
        velocityTeleop.workspaceLimiter = tcpTarget != null
            ? tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>()
            : null;
        velocityTeleop.targetWriteMonitor = tcpTarget != null
            ? tcpTarget.GetComponent<TcpTargetWriteMonitor>()
            : null;
        ApplyDefaultQuestTeleopProfile(velocityTeleop);
        ApplyQuestTeleopSpeedProfile(velocityTeleop);
        velocityTeleop.angularSpeedGain = 1.30f;
        velocityTeleop.maxAngularSpeedRadiansPerSecond = 4.00f;
        velocityTeleop.maxAngularAcceleration = 10.00f;

        if (addUrScriptSpeedlClient)
        {
            Ur5UrScriptSpeedlClient speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
            if (speedlClient == null)
            {
                speedlClient = gameObject.AddComponent<Ur5UrScriptSpeedlClient>();
            }

            ApplyRealRobotSpeedlSafetyProfile(speedlClient);
            velocityTeleop.speedlClient = speedlClient;
        }
    }

    /// <summary>
    /// Shared XR clutch profile. Both runtime bootstraps call this method so
    /// Editor and Quest builds use the same controller-to-TCP semantics.
    /// </summary>
    public static void ApplyDefaultQuestTeleopProfile(Ur5CartesianVelocityTeleopController velocityTeleop)
    {
        if (velocityTeleop == null)
        {
            return;
        }

        // Standard Quest control follows the UR10_Teleop anchored-pose loop.
        // The legacy continuous controller is kept only for old Inspector data.
        velocityTeleop.enableUr10StyleAnchoredPoseClutch = true;
        velocityTeleop.enableContinuous6DofClutch = false;
        velocityTeleop.continuous6DofConfig = Ur5Continuous6DofConfig.Default;
        velocityTeleop.positionControllerNode = UnityEngine.XR.XRNode.RightHand;
        velocityTeleop.rotationControllerNode = UnityEngine.XR.XRNode.RightHand;
        velocityTeleop.safetyControllerNode = UnityEngine.XR.XRNode.LeftHand;
        velocityTeleop.usePositionGripAsDeadman = true;
        velocityTeleop.useRotationGripAsDeadman = true;
        velocityTeleop.unityPreviewMode = Ur5CartesianVelocityTeleopController.UnityPreviewMode.RelativePoseTarget;
        velocityTeleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
        velocityTeleop.enableThreeModeController = false;
        velocityTeleop.useRightSecondaryButtonForInsertMode = false;

        // A relative clutch maps controller displacement to TCP displacement.
        // It is intentionally not a joystick-velocity integrator.
        velocityTeleop.linearDeadbandMeters = 0.0f;
        // Use a radial gate rather than a time-based hand low-pass. This
        // rejects Quest idle jitter without adding trailing motion.
        velocityTeleop.filterControllerPosition = true;
        velocityTeleop.controllerPositionJitterDeadbandMeters = 0.0025f;
        velocityTeleop.controllerPositionFilterSharpness = 16.0f;
        velocityTeleop.useAdaptiveControllerPositionFilter = false;
        velocityTeleop.angularDeadbandDegrees = 1.0f;
        velocityTeleop.relativePreviewPositionScale = 1.00f;
        velocityTeleop.normalPositionScale = 1.00f;
        velocityTeleop.precisionModifierPositionScale = 0.15f;
        velocityTeleop.useProgressivePositionResponse = false;
        velocityTeleop.precisionPositionScale = 2.80f;
        velocityTeleop.progressivePositionTransitionMeters = 0.030f;
        velocityTeleop.relativePreviewRotationScale = 1.00f;
        velocityTeleop.previewPositionSmoothingSharpness = 26.0f;
        velocityTeleop.previewRotationSmoothingSharpness = 18.0f;
        velocityTeleop.useAnchoredPoseTeleopStrategy = true;
        velocityTeleop.anchoredPoseSmoothingStep = 0.70f;
        velocityTeleop.anchoredPosePrecisionSmoothingStep = 0.70f;
        velocityTeleop.useRelativePoseCommandFilter = false;
        velocityTeleop.relativePoseCommandFilterRetention = 0.0f;
        velocityTeleop.fineRelativePoseCommandFilterRetention = 0.0f;
        velocityTeleop.previewMaxLinearSpeed = 0.0f;
        velocityTeleop.previewMaxAngularSpeedDegreesPerSecond = 420.0f;
        velocityTeleop.limitPreviewLeadToActualTcp = true;
        velocityTeleop.maximumPreviewLeadMeters = 0.040f;
        velocityTeleop.movingPreviewLeadMeters = 0.160f;
        velocityTeleop.useAccelerationLimitedPreviewTrajectory = false;
        velocityTeleop.freezeRobotWhenPositionHandStops = true;
        velocityTeleop.controllerMotionEpsilonMeters = 0.0025f;
        velocityTeleop.controllerStopHoldSeconds = 0.04f;
        velocityTeleop.previewTargetDeadbandMeters = 0.0f;
        velocityTeleop.finePreviewTargetDeadbandMeters = 0.0f;

        // 左摇杆旋转在单模式标准链路中关闭，避免形成第二个姿态写入入口。
        velocityTeleop.rotationJoystickDeadband = 0.12f;
        velocityTeleop.joystickYawSpeedDegreesPerSecond = 220.0f;
        velocityTeleop.normalJoystickYawSpeedDegreesPerSecond = 60.0f;
        velocityTeleop.precisionJoystickYawSpeedDegreesPerSecond = 18.0f;
        velocityTeleop.joystickResponseExponent = 1.0f;
        velocityTeleop.joystickPitchSpeedDegreesPerSecond = 0.0f;
        velocityTeleop.joystickRollSpeedDegreesPerSecond = 0.0f;
        velocityTeleop.useSecondaryButtonForJoystickRoll = false;
        velocityTeleop.enableLeftSecondaryPoseRotation = false;
        velocityTeleop.useRobotBaseForwardForRightAPose = true;
        velocityTeleop.rightADefaultJawYawOffsetDegrees = 0.0f;
        velocityTeleop.enableRightSecondaryFreeWristPoseControl = true;
        velocityTeleop.rightSecondaryFreeWristRotationScale = 1.0f;
        velocityTeleop.snapJoystickRotationToZeroInDeadband = true;

        // No alternate hand mode can change the standard clutch mapping.
        velocityTeleop.enableFineControlButton = false;
        velocityTeleop.applyFineControlToRelativePreview = false;
        velocityTeleop.fineLinearSpeedMultiplier = 1.00f;
        velocityTeleop.fineAngularSpeedMultiplier = 1.00f;
        velocityTeleop.enableAPrecisionModifier = false;
        velocityTeleop.precisionModifierPositionScale = 0.30f;

        velocityTeleop.snapToZeroOnRelease = true;
        velocityTeleop.snapGraspApproachToVertical = false;
        velocityTeleop.verticalApproachSnapDegrees = 32.0f;
        velocityTeleop.enableLeftPrimarySnapDown = false;
        velocityTeleop.enableLeftPrimaryReadyPose = false;
        velocityTeleop.leftPrimaryReadyPoseHoldSeconds = 0.45f;
        velocityTeleop.leftPrimarySnapTimeoutSeconds = 3.0f;
        velocityTeleop.leftPrimarySnapPositionToleranceMeters = 0.003f;
        velocityTeleop.leftPrimarySnapRotationToleranceDegrees = 0.50f;
        velocityTeleop.enableLeftSecondaryOrientationHold = false;
        velocityTeleop.linearSpeedGain = 1.20f;
        velocityTeleop.maxLinearSpeed = 0.32f;
        velocityTeleop.maxLinearAcceleration = 1.60f;
        velocityTeleop.angularSpeedGain = 1.30f;
        velocityTeleop.maxAngularSpeedRadiansPerSecond = 4.00f;
        velocityTeleop.maxAngularAcceleration = 10.00f;
    }

    /// <summary>
    /// 将 Inspector 中的 Quest 平移速度配置集中应用到控制器。
    /// 统一入口可避免自动引导脚本覆盖操作者在 Inspector 中的速度设置。
    /// </summary>
    public void ApplyQuestTeleopSpeedProfile(Ur5CartesianVelocityTeleopController velocityTeleop)
    {
        if (velocityTeleop == null)
        {
            return;
        }

        velocityTeleop.relativePreviewPositionScale = EffectiveQuestTranslationScale;
        velocityTeleop.normalPositionScale = EffectiveQuestTranslationScale;
        velocityTeleop.previewMaxLinearSpeed = EffectiveQuestPreviewMaxLinearSpeed;
        velocityTeleop.maximumPreviewLeadMeters = EffectiveQuestMaximumPreviewLeadMeters;
        velocityTeleop.movingPreviewLeadMeters = EffectiveQuestMovingPreviewLeadMeters;
        velocityTeleop.linearSpeedGain = 1.20f;
        velocityTeleop.maxLinearSpeed = EffectiveQuestCommandMaxLinearSpeed;
        velocityTeleop.maxLinearAcceleration = EffectiveQuestCommandMaxLinearAcceleration;
    }

    public void ApplyRealRobotSpeedlSafetyProfile(Ur5UrScriptSpeedlClient speedlClient)
    {
        if (speedlClient == null)
        {
            return;
        }

        speedlClient.requireMotionArmed = true;
        speedlClient.armOnStart = false;
        speedlClient.maxLinearSpeed = Mathf.Max(0.0f, realRobotMaxLinearSpeed);
        speedlClient.maxAngularSpeedRadiansPerSecond = Mathf.Max(0.0f, realRobotMaxAngularSpeedRadiansPerSecond);
        speedlClient.acceleration = Mathf.Max(0.0f, realRobotCommandAcceleration);
        speedlClient.stopAcceleration = Mathf.Max(0.0f, realRobotStopAcceleration);
        speedlClient.limitOutputAcceleration = true;
        speedlClient.maxOutputLinearAcceleration = Mathf.Max(0.0f, realRobotMaxOutputLinearAcceleration);
        speedlClient.maxOutputAngularAcceleration = Mathf.Max(0.0f, realRobotMaxOutputAngularAcceleration);
        speedlClient.sendStoplOnStop = true;
    }

    public void ApplyWorkspaceSafetyProfile(TcpTargetWorkspaceLimiter workspaceLimiter)
    {
        if (workspaceLimiter == null)
        {
            return;
        }

        float minimumHeight = Mathf.Max(0.0f, workspaceMinimumLocalHeightMeters);
        workspaceLimiter.minimumLocalPosition = new Vector3(
            workspaceLimiter.minimumLocalPosition.x,
            minimumHeight,
            workspaceLimiter.minimumLocalPosition.z);
    }

    private float GetFastProfileMinimum(float configuredValue, float fastProfileMinimum)
    {
        return enforceFastQuestMotionProfile
            ? Mathf.Max(configuredValue, fastProfileMinimum)
            : configuredValue;
    }

    private bool IsRealRobotOutputEnabled()
    {
        Ur5UrScriptSpeedlClient speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
        return speedlClient != null && speedlClient.enableRealRobotOutput;
    }

    private bool ShouldSendQuestPackets()
    {
        return enableQuestUdpShadowTelemetry
            || !string.IsNullOrWhiteSpace(questShadowReceiverHost);
    }

    private bool ShouldUseQuestUdpTelemetryOnlyMode()
    {
        return disableLocalTeleopWhenQuestUdpShadowTelemetry
            && ShouldSendQuestPackets();
    }

    private void DisableLocalTeleopWritersForQuestUdpBridge()
    {
        DisableQuestPoseController();
        DisableLegacyVrTeleoperationControllers();
        DisableKeyboardTargetController();
        DisablePoseIkFollower();

        Ur5CartesianVelocityTeleopController velocityTeleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        if (velocityTeleop != null)
        {
            velocityTeleop.enabled = false;
        }

        Ur5UrScriptSpeedlClient speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
        if (speedlClient != null)
        {
            speedlClient.StopRobot();
            speedlClient.enabled = false;
        }

        Ur5GraspAssistController graspAssist = GetComponent<Ur5GraspAssistController>();
        if (graspAssist != null)
        {
            graspAssist.enabled = false;
        }

        if (robotRoot != null)
        {
            Quest3RobotiqGripperController gripperController =
                robotRoot.GetComponent<Quest3RobotiqGripperController>();
            if (gripperController != null)
            {
                gripperController.enabled = false;
            }
        }
    }

    private void ConfigureQuestUdpShadowTelemetry()
    {
        Quest3UdpTeleopSender sender = GetComponent<Quest3UdpTeleopSender>();
        if (!ShouldSendQuestPackets())
        {
            if (sender != null)
            {
                sender.sendPackets = false;
            }

            return;
        }

        if (sender == null)
        {
            sender = gameObject.AddComponent<Quest3UdpTeleopSender>();
        }

        sender.receiverHost = questShadowReceiverHost;
        sender.receiverPort = questShadowReceiverPort;
        sender.sendRateHz = questShadowSendRateHz;
        // 此处仅打开 Quest -> PC 遥测，不关联 Ur5UrScriptSpeedlClient。
        sender.sendPackets = true;
    }

    private void ConfigureGripperController()
    {
        if (robotRoot == null)
        {
            return;
        }

        Quest3RobotiqGripperController gripperController = robotRoot.GetComponent<Quest3RobotiqGripperController>();
        if (gripperController == null)
        {
            gripperController = robotRoot.gameObject.AddComponent<Quest3RobotiqGripperController>();
        }

        gripperController.robotRoot = robotRoot;
        gripperController.controllerNode = UnityEngine.XR.XRNode.RightHand;
        gripperController.useTrigger = true;
        gripperController.requireGripDeadman = true;
        gripperController.gripPressThreshold = 0.65f;
        gripperController.gripReleaseThreshold = 0.40f;
        gripperController.triggerDeadband = 0.04f;
        gripperController.triggerSmoothingSharpness = 22.0f;
        gripperController.closeSpeedPerSecond = 2.60f;
        gripperController.openSpeedPerSecond = 3.20f;
        gripperController.damping = 450.0f;
        gripperController.forceLimit = 160.0f;
        gripperController.ApplyConfiguredDriveSettings();
    }

    private Ur5GraspAssistController ConfigureGraspAssist(Ur5TcpTargetFollower follower)
    {
        if (robotRoot == null || tcpTarget == null)
        {
            return null;
        }

        Ur5GraspAssistController graspAssist = GetComponent<Ur5GraspAssistController>();
        if (graspAssist == null)
        {
            graspAssist = gameObject.AddComponent<Ur5GraspAssistController>();
        }

        graspAssist.enabled = true;
        graspAssist.robotRoot = robotRoot;
        graspAssist.tcpTarget = tcpTarget;
        graspAssist.tcpFollower = follower != null
            ? follower
            : robotRoot.GetComponent<Ur5TcpTargetFollower>();
        if (graspAssist.tcpFollower != null)
        {
            graspAssist.tcpFollower.graspAssist = graspAssist;
        }
        graspAssist.velocityTeleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        graspAssist.workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
        graspAssist.targetWriteMonitor = tcpTarget.GetComponent<TcpTargetWriteMonitor>();
        graspAssist.gripperController = robotRoot.GetComponent<Quest3RobotiqGripperController>();
        graspAssist.autoSelectNearestTarget = true;
        graspAssist.targetSearchRadius = 0.50f;
        graspAssist.maxAutoTargetSize = 0.35f;
        // A is the manual precision modifier and B remains unbound during the
        // virtual-UR5 teleoperation profile. Keep the assist component and its
        // keyboard path available without competing for controller ownership.
        graspAssist.startWithPrimaryButton = false;
        graspAssist.startWithSecondaryButton = false;
        graspAssist.preGraspHeight = 0.12f;
        graspAssist.alignPadCenterToObjectCenter = true;
        graspAssist.padCenterOffsetAlongApproach = 0.0f;
        graspAssist.graspClearance = 0.015f;
        graspAssist.liftHeight = 0.16f;
        graspAssist.assistMoveSpeed = 0.18f;
        graspAssist.finalApproachSpeed = 0.050f;
        graspAssist.assistMoveAcceleration = 0.85f;
        graspAssist.waypointTolerance = 0.012f;
        graspAssist.transitActualPositionTolerance = 0.008f;
        graspAssist.finalGraspActualPositionTolerance = 0.0015f;
        graspAssist.finalGraspActualRotationToleranceDegrees = 0.35f;
        graspAssist.finalGraspSettleSeconds = 0.12f;
        graspAssist.actualPositionTolerance = 0.008f;
        graspAssist.robotBodyClearanceRadius = 0.075f;
        // All grasp stages use the calibrated downward tool attitude instead
        // of inheriting any transient manual pose.
        graspAssist.preserveCurrentTcpRotation = false;
        graspAssist.alignToolForwardAgainstApproachDirection = true;
        graspAssist.graspYawReferenceWorld = Vector3.zero;
        graspAssist.fixedTcpRotationEuler = new Vector3(180.0f, 0.0f, 0.0f);
        return graspAssist;
    }

    private void ConfigureSpectatorCamera()
    {
        Ur5EditorSpectatorCamera spectatorCamera = GetComponent<Ur5EditorSpectatorCamera>();
        if (spectatorCamera == null)
        {
            spectatorCamera = gameObject.AddComponent<Ur5EditorSpectatorCamera>();
        }

        spectatorCamera.robotRoot = robotRoot;
        spectatorCamera.tcpTarget = tcpTarget;
    }

    private void ConfigureTeleopDiagnostics(
        Ur5TcpTargetFollower follower,
        Ur5JointTrajectoryPlayer trajectoryPlayer,
        Ur5ArticulationJointController jointController)
    {
        Ur5TeleopDiagnostics diagnostics = GetComponent<Ur5TeleopDiagnostics>();
        if (diagnostics == null)
        {
            diagnostics = gameObject.AddComponent<Ur5TeleopDiagnostics>();
        }

        diagnostics.enableDiagnostics = false;
        diagnostics.teleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        diagnostics.follower = follower;
        diagnostics.trajectoryPlayer = trajectoryPlayer;
        diagnostics.jointController = jointController;
        diagnostics.targetWriteMonitor = tcpTarget != null
            ? tcpTarget.GetComponent<TcpTargetWriteMonitor>()
            : null;
    }

    private void ConfigureControllerVisualizer()
    {
        Quest3ControllerVisualizer controllerVisualizer = GetComponent<Quest3ControllerVisualizer>();
        if (controllerVisualizer == null)
        {
            controllerVisualizer = gameObject.AddComponent<Quest3ControllerVisualizer>();
        }

        controllerVisualizer.xrOrigin = FindXrOrigin();
        controllerVisualizer.SetVirtualControllerVisibility(
            !IsPassthroughEnabled() || showVirtualControllersInPassthrough);
    }

    private bool IsPassthroughEnabled()
    {
        Transform xrOrigin = FindXrOrigin();
        return xrOrigin != null && xrOrigin.GetComponent("OVRPassthroughLayer") != null;
    }

    private void ConfigureRecorder(
        Ur5ArticulationJointController jointController,
        Ur5JointTrajectoryPlayer trajectoryPlayer,
        Ur5TcpTargetFollower follower,
        Ur5GraspAssistController graspAssist)
    {
        Ur5PoseCsvRecorder recorder = GetComponent<Ur5PoseCsvRecorder>();
        if (recorder == null)
        {
            recorder = gameObject.AddComponent<Ur5PoseCsvRecorder>();
        }

        recorder.tcpTarget = tcpTarget;
        recorder.jointController = jointController;
        recorder.trajectoryPlayer = trajectoryPlayer;
        recorder.graspAssist = graspAssist;
        recorder.tcpFollower = follower != null
            ? follower
            : FindObjectOfType<Ur5TcpTargetFollower>();
        recorder.questController = tcpTarget != null ? tcpTarget.GetComponent<Quest3TcpTargetController>() : null;
        recorder.velocityTeleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        recorder.speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
        recorder.autoRecordOnAndroid = true;
        recorder.flushIntervalSeconds = 1.0f;
        recorder.sampleInterval = 0.02f;
    }

    private void DisableQuestPoseController()
    {
        if (tcpTarget == null)
        {
            return;
        }

        Quest3TcpTargetController questPoseController = tcpTarget.GetComponent<Quest3TcpTargetController>();
        if (questPoseController != null)
        {
            questPoseController.enabled = false;
        }
    }

    private void DisableLegacyVrTeleoperationControllers()
    {
        foreach (VRTeleoperationController legacyController in FindObjectsOfType<VRTeleoperationController>())
        {
            legacyController.enabled = false;
        }
    }

    private void DisableKeyboardTargetController()
    {
        if (tcpTarget == null)
        {
            return;
        }

        TcpTargetKeyboardController keyboardController = tcpTarget.GetComponent<TcpTargetKeyboardController>();
        if (keyboardController != null)
        {
            keyboardController.enabled = false;
        }
    }

    private void DisablePoseIkFollower()
    {
        if (robotRoot != null)
        {
            Ur5TcpTargetFollower follower = robotRoot.GetComponent<Ur5TcpTargetFollower>();
            if (follower != null)
            {
                follower.enabled = false;
            }

            Ur5JointTrajectoryPlayer trajectoryPlayer = robotRoot.GetComponent<Ur5JointTrajectoryPlayer>();
            if (trajectoryPlayer != null)
            {
                trajectoryPlayer.ClearQueue();
                trajectoryPlayer.enabled = false;
            }
        }

        if (tcpTarget != null)
        {
            Ur5ActualTcpMarker actualMarker = tcpTarget.GetComponent<Ur5ActualTcpMarker>();
            if (actualMarker != null)
            {
                actualMarker.enabled = false;
            }
        }
    }

    private Transform FindXrOrigin()
    {
        GameObject xrOrigin = GameObject.Find("XR Origin (VR)");
        return xrOrigin != null ? xrOrigin.transform : null;
    }

    private Transform FindRobotRoot()
    {
        string[] candidateNames =
        {
            "ur5_robot",
            "ur5",
            "UR5",
            "base_link"
        };

        foreach (string candidateName in candidateNames)
        {
            GameObject candidate = GameObject.Find(candidateName);
            if (candidate != null)
            {
                return candidate.transform;
            }
        }

        return null;
    }

    private void DisableLegacyUrdfImporterController()
    {
        if (robotRoot == null)
        {
            return;
        }

        MonoBehaviour[] components = robotRoot.GetComponents<MonoBehaviour>();
        foreach (MonoBehaviour component in components)
        {
            if (component == null)
            {
                continue;
            }

            string typeName = component.GetType().FullName;
            if (typeName == "Unity.Robotics.UrdfImporter.Control.Controller"
                || typeName == "Unity.Robotics.UrdfImporter.Control.FKRobot"
                || typeName == "Unity.Robotics.UrdfImporter.Control.IKRobot")
            {
                component.enabled = false;
                Debug.Log("Disabled legacy URDF Importer component: " + typeName);
            }
        }
    }
}

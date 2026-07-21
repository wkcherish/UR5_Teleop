using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR;

public static class Ur5AutoSceneBootstrap
{
    private const float FallbackQuestRefreshRate = 72.0f;
    private static readonly List<XRDisplaySubsystem> DisplaySubsystems = new List<XRDisplaySubsystem>();
    private static readonly List<XRInputSubsystem> InputSubsystems = new List<XRInputSubsystem>();
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void InstallAfterSceneLoad()
    {
        ApplyQuestFloorTrackingOrigin();
        ConfigurePassthroughCameraBackground();
        ApplyQuestRuntimeTiming();
        InstallStabilizer();
        InstallTargetWorkspaceLimiter();
        InstallCartesianVelocityTeleop();
        InstallTcpTargetFollower();
        InstallGripperController();
        InstallGraspAssist();
        InstallSpectatorCamera();
        InstallControllerVisualizer();
        SceneManager.sceneLoaded -= HandleSceneLoaded;
        SceneManager.sceneLoaded += HandleSceneLoaded;
    }

    private static void HandleSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        ApplyQuestFloorTrackingOrigin();
        ConfigurePassthroughCameraBackground();
        ApplyQuestRuntimeTiming();
        InstallStabilizer();
        InstallTargetWorkspaceLimiter();
        InstallCartesianVelocityTeleop();
        InstallTcpTargetFollower();
        InstallGripperController();
        InstallGraspAssist();
        InstallSpectatorCamera();
        InstallControllerVisualizer();
    }

    private static void InstallStabilizer()
    {
        Ur5PhysicsStabilizer existing = Object.FindObjectOfType<Ur5PhysicsStabilizer>();
        if (existing != null)
        {
            existing.robotRoot = FindRobotRoot();
            existing.Stabilize();
            return;
        }

        GameObject stabilizerObject = new GameObject("UR5AutoRuntimeStabilizer");
        Object.DontDestroyOnLoad(stabilizerObject);

        Ur5PhysicsStabilizer stabilizer = stabilizerObject.AddComponent<Ur5PhysicsStabilizer>();
        stabilizer.robotRoot = FindRobotRoot();
        stabilizer.scanSceneIfRobotRootMissing = true;
        stabilizer.disableGravity = true;
        stabilizer.fixArticulationRoots = true;
        stabilizer.makeRigidbodiesKinematic = true;
        stabilizer.lockRobotRootTransform = true;
        stabilizer.holdCurrentJointPose = true;
        stabilizer.Stabilize();
    }

    private static void InstallTcpTargetFollower()
    {
        if (ShouldDisablePoseIkForVelocityTeleop())
        {
            DisablePoseIkControllers();
            return;
        }

        Transform robotRoot = FindRobotRoot();
        GameObject target = GameObject.Find("TcpTarget");
        if (robotRoot == null || target == null)
        {
            return;
        }

        Ur5ArticulationJointController jointController = robotRoot.GetComponent<Ur5ArticulationJointController>();
        if (jointController == null)
        {
            jointController = robotRoot.gameObject.AddComponent<Ur5ArticulationJointController>();
        }

        jointController.robotRoot = robotRoot;
        ApplyStableJointDefaults(jointController);

        Ur5JointTrajectoryPlayer trajectoryPlayer = robotRoot.GetComponent<Ur5JointTrajectoryPlayer>();
        if (trajectoryPlayer == null)
        {
            trajectoryPlayer = robotRoot.gameObject.AddComponent<Ur5JointTrajectoryPlayer>();
        }

        trajectoryPlayer.jointController = jointController;
        trajectoryPlayer.enabled = true;
        ApplyStableTrajectoryDefaults(trajectoryPlayer);

        Ur5TcpTargetFollower follower = robotRoot.GetComponent<Ur5TcpTargetFollower>();
        if (follower == null)
        {
            follower = robotRoot.gameObject.AddComponent<Ur5TcpTargetFollower>();
        }

        follower.robotRoot = robotRoot;
        follower.tcpTarget = target.transform;
        follower.jointController = jointController;
        follower.trajectoryPlayer = trajectoryPlayer;
        follower.enabled = true;
        follower.questController = target.GetComponent<Quest3TcpTargetController>();
        follower.velocityTeleop = Object.FindObjectOfType<Ur5CartesianVelocityTeleopController>();
        follower.graspAssist = Object.FindObjectOfType<Ur5GraspAssistController>();
        follower.pauseIkWhenVelocityTeleopIdle = follower.velocityTeleop != null;
        ApplyStableFollowerDefaults(follower);
        Ur5ControlBootstrap controlBootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        if (controlBootstrap != null)
        {
            follower.maxJointStepDegrees = controlBootstrap.questMaxJointStepDegrees;
        }

        Ur5ActualTcpMarker actualMarker = target.GetComponent<Ur5ActualTcpMarker>();
        if (actualMarker == null)
        {
            actualMarker = target.AddComponent<Ur5ActualTcpMarker>();
        }

        actualMarker.enabled = true;
        actualMarker.follower = follower;
        actualMarker.hideCommandTargetRenderer = true;
        actualMarker.markerDiameter = 0.035f;
    }

    private static void InstallCartesianVelocityTeleop()
    {
        Ur5ControlBootstrap bootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        if (bootstrap == null || !bootstrap.enableCartesianVelocityTeleop)
        {
            return;
        }

        Transform robotRoot = FindRobotRoot();
        Ur5CartesianVelocityTeleopController velocityTeleop =
            bootstrap.GetComponent<Ur5CartesianVelocityTeleopController>();
        if (velocityTeleop == null)
        {
            velocityTeleop = bootstrap.gameObject.AddComponent<Ur5CartesianVelocityTeleopController>();
        }

        velocityTeleop.enabled = true;
        velocityTeleop.robotBaseFrame = robotRoot;
        velocityTeleop.xrOrigin = FindXrOrigin();
        GameObject target = GameObject.Find("TcpTarget");
        velocityTeleop.tcpPreviewTarget = target != null ? target.transform : null;
        velocityTeleop.workspaceLimiter = target != null
            ? target.GetComponent<TcpTargetWorkspaceLimiter>()
            : null;
        ApplyStableVelocityTeleopDefaults(velocityTeleop);
        bootstrap.ApplyQuestTeleopSpeedProfile(velocityTeleop);

        if (bootstrap.addUrScriptSpeedlClient)
        {
            Ur5UrScriptSpeedlClient speedlClient = bootstrap.GetComponent<Ur5UrScriptSpeedlClient>();
            if (speedlClient == null)
            {
                speedlClient = bootstrap.gameObject.AddComponent<Ur5UrScriptSpeedlClient>();
            }

            velocityTeleop.speedlClient = speedlClient;
        }

        Quest3TcpTargetController questPoseController =
            target != null ? target.GetComponent<Quest3TcpTargetController>() : null;
        if (questPoseController != null)
        {
            questPoseController.enabled = false;
        }

        foreach (VRTeleoperationController legacyController in Object.FindObjectsOfType<VRTeleoperationController>())
        {
            legacyController.enabled = false;
        }
    }

    private static void ApplyStableJointDefaults(Ur5ArticulationJointController jointController)
    {
        jointController.stiffness = 12000.0f;
        jointController.damping = 5200.0f;
        jointController.forceLimit = 30000.0f;
        jointController.smoothDriveTargets = true;
        jointController.maxDriveSpeedDegreesPerSecond = 240.0f;
        jointController.maxDriveAccelerationDegreesPerSecondSquared = 7000.0f;
        jointController.driveTargetToleranceDegrees = 0.005f;
        jointController.ApplyConfiguredDriveSettings();
    }

    private static void ApplyStableFollowerDefaults(Ur5TcpTargetFollower follower)
    {
        follower.useGripperPadCenter = true;
        follower.usePadGeometryCenter = true;
        // 姿态与位置统一以两指中心抓取坐标系计算，避免 tool0 轴约定导致夹爪偏航。
        follower.usePhysicalGraspFrameForOrientation = true;
        follower.gripperBase = null;
        follower.positionTolerance = 0.008f;
        follower.maxJointStepDegrees = 1.45f;
        follower.minimumJointDeltaDegrees = 0.015f;
        follower.maximumCommandLeadDegrees = 9.00f;
        follower.useTimedJointAssignments = true;
        follower.jointAssignmentIntervalSeconds = 0.016f;
        follower.dlsDamping = 0.18f;
        follower.dlsOrientationWeight = 1.50f;
        follower.translationOrientationHoldWeight = 3.00f;
        follower.dlsGain = 0.85f;
        follower.proximalOrientationWeight = 0.05f;
        follower.jointDeltaSmoothing = 0.10f;
        follower.rotationToleranceDegrees = 0.03f;
        follower.rotationBlend = 0.70f;
        follower.maxWristStepDegrees = 3.00f;
        // 保持当前工具姿态约束；右手平移时不能让位置 IK 自由改变腕关节姿态。
        follower.suppressRotationOnlyIkDuringPositionControl = false;
        follower.finishVelocityTargetAfterRelease = true;
        follower.velocityReleasePositionTolerance = 0.003f;
        follower.velocityReleaseRotationToleranceDegrees = 0.50f;
        follower.velocityReleaseSettleTimeoutSeconds = 2.0f;
        follower.holdJointPoseWhenTargetSettled = true;
        follower.targetStationaryHoldSeconds = 0.12f;
        follower.targetStationaryPositionEpsilon = 0.0015f;
        follower.targetStationaryRotationEpsilonDegrees = 0.30f;
        follower.settledPositionError = 0.010f;
        follower.settledRotationErrorDegrees = 1.50f;
        follower.enableReadyPose = true;
        follower.readyPoseJointDegrees = new[] { 0.0f, -90.0f, 90.0f, -90.0f, -90.0f, 0.0f };
        follower.readyPoseMaxJointSpeedDegreesPerSecond = 40.0f;
        follower.readyPoseJointToleranceDegrees = 1.5f;
    }

    private static void ApplyStableTrajectoryDefaults(Ur5JointTrajectoryPlayer trajectoryPlayer)
    {
        trajectoryPlayer.play = true;
        trajectoryPlayer.queueMode = Ur5JointTrajectoryPlayer.QueueMode.LatestOnly;
        trajectoryPlayer.jointAssignmentIntervalSeconds = 0.016f;
        trajectoryPlayer.applyDirectlyToDrive = true;
        trajectoryPlayer.clampToDriveLimits = true;
        trajectoryPlayer.maxQueuedWaypoints = 1;
    }

    private static void ApplyStableVelocityTeleopDefaults(Ur5CartesianVelocityTeleopController velocityTeleop)
    {
        velocityTeleop.positionControllerNode = UnityEngine.XR.XRNode.RightHand;
        velocityTeleop.rotationControllerNode = UnityEngine.XR.XRNode.LeftHand;
        velocityTeleop.usePositionGripAsDeadman = true;
        velocityTeleop.useRotationGripAsDeadman = true;
        velocityTeleop.unityPreviewMode = Ur5CartesianVelocityTeleopController.UnityPreviewMode.RelativePoseTarget;
        // 左手 Grip + 摇杆只绕基座 Y 轴旋转夹爪，避免手柄自身姿态噪声影响 IK。
        velocityTeleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Joystick;
        velocityTeleop.rotationJoystickDeadband = 0.12f;
        velocityTeleop.joystickYawSpeedDegreesPerSecond = 125.0f;
        velocityTeleop.joystickPitchSpeedDegreesPerSecond = 0.0f;
        velocityTeleop.joystickRollSpeedDegreesPerSecond = 0.0f;
        velocityTeleop.useSecondaryButtonForJoystickRoll = false;
        velocityTeleop.invertJoystickPitch = false;
        velocityTeleop.snapJoystickRotationToZeroInDeadband = true;
        velocityTeleop.filterControllerPosition = true;
        velocityTeleop.controllerPositionJitterDeadbandMeters = 0.0025f;
        velocityTeleop.controllerPositionFilterSharpness = 16.0f;
        velocityTeleop.linearDeadbandMeters = 0.005f;
        velocityTeleop.angularDeadbandDegrees = 2.5f;
        velocityTeleop.relativePreviewPositionScale = 2.40f;
        velocityTeleop.relativePreviewRotationScale = 0.80f;
        velocityTeleop.previewMaxLinearSpeed = 0.35f;
        velocityTeleop.previewMaxAngularSpeedDegreesPerSecond = 420.0f;
        velocityTeleop.limitPreviewLeadToActualTcp = true;
        velocityTeleop.maximumPreviewLeadMeters = 0.025f;
        velocityTeleop.previewPositionSmoothingSharpness = 26.0f;
        velocityTeleop.previewRotationSmoothingSharpness = 18.0f;
        velocityTeleop.snapGraspApproachToVertical = true;
        velocityTeleop.verticalApproachSnapDegrees = 32.0f;
        velocityTeleop.enableLeftPrimarySnapDown = true;
        velocityTeleop.enableLeftPrimaryReadyPose = true;
        velocityTeleop.leftPrimaryReadyPoseHoldSeconds = 0.45f;
        velocityTeleop.enableLeftSecondaryOrientationHold = true;
        velocityTeleop.enableFineControlButton = true;
        velocityTeleop.applyFineControlToRelativePreview = true;
        velocityTeleop.fineLinearSpeedMultiplier = 0.25f;
        velocityTeleop.fineAngularSpeedMultiplier = 0.25f;
        velocityTeleop.linearSpeedGain = 1.20f;
        velocityTeleop.maxLinearSpeed = 0.14f;
        velocityTeleop.maxLinearAcceleration = 0.40f;
        velocityTeleop.angularSpeedGain = 1.30f;
        velocityTeleop.maxAngularSpeedRadiansPerSecond = 4.00f;
        velocityTeleop.maxAngularAcceleration = 10.00f;
    }

    private static void InstallGripperController()
    {
        Transform robotRoot = FindRobotRoot();
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

    private static void InstallGraspAssist()
    {
        Ur5ControlBootstrap bootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        Transform robotRoot = FindRobotRoot();
        GameObject target = GameObject.Find("TcpTarget");
        if (bootstrap == null || robotRoot == null || target == null)
        {
            return;
        }

        Ur5GraspAssistController graspAssist = bootstrap.GetComponent<Ur5GraspAssistController>();
        if (graspAssist == null)
        {
            graspAssist = bootstrap.gameObject.AddComponent<Ur5GraspAssistController>();
        }

        graspAssist.enabled = true;
        graspAssist.robotRoot = robotRoot;
        graspAssist.tcpTarget = target.transform;
        graspAssist.tcpFollower = robotRoot.GetComponent<Ur5TcpTargetFollower>();
        if (graspAssist.tcpFollower != null)
        {
            graspAssist.tcpFollower.graspAssist = graspAssist;
        }
        graspAssist.velocityTeleop = bootstrap.GetComponent<Ur5CartesianVelocityTeleopController>();
        graspAssist.workspaceLimiter = target.GetComponent<TcpTargetWorkspaceLimiter>();
        graspAssist.gripperController = robotRoot.GetComponent<Quest3RobotiqGripperController>();
        graspAssist.autoSelectNearestTarget = true;
        graspAssist.targetSearchRadius = 0.50f;
        graspAssist.maxAutoTargetSize = 0.35f;
        graspAssist.startWithPrimaryButton = true;
        graspAssist.startWithSecondaryButton = true;
        graspAssist.preGraspHeight = 0.12f;
        graspAssist.graspClearance = 0.015f;
        graspAssist.liftHeight = 0.16f;
        graspAssist.assistMoveSpeed = 0.10f;
        graspAssist.assistMoveAcceleration = 0.45f;
        graspAssist.waypointTolerance = 0.012f;
        graspAssist.actualPositionTolerance = 0.025f;
        graspAssist.robotBodyClearanceRadius = 0.075f;
        graspAssist.preserveCurrentTcpRotation = false;
        graspAssist.alignToolForwardAgainstApproachDirection = true;
        graspAssist.graspYawReferenceWorld = Vector3.zero;
        graspAssist.fixedTcpRotationEuler = new Vector3(180.0f, 0.0f, 0.0f);
    }

    private static void InstallTargetWorkspaceLimiter()
    {
        Transform robotRoot = FindRobotRoot();
        GameObject target = GameObject.Find("TcpTarget");
        if (robotRoot == null || target == null)
        {
            return;
        }

        TcpTargetWorkspaceLimiter workspaceLimiter = target.GetComponent<TcpTargetWorkspaceLimiter>();
        if (workspaceLimiter == null)
        {
            workspaceLimiter = target.AddComponent<TcpTargetWorkspaceLimiter>();
        }

        workspaceLimiter.robotRoot = robotRoot;
        workspaceLimiter.minimumLocalPosition = new Vector3(
            workspaceLimiter.minimumLocalPosition.x,
            0.05f,
            workspaceLimiter.minimumLocalPosition.z);

        TcpTargetCollisionGuard collisionGuard = target.GetComponent<TcpTargetCollisionGuard>();
        if (collisionGuard == null)
        {
            collisionGuard = target.AddComponent<TcpTargetCollisionGuard>();
        }

        collisionGuard.robotRoot = robotRoot;
        collisionGuard.preventObstaclePenetration = false;
    }

    private static void InstallSpectatorCamera()
    {
        Transform robotRoot = FindRobotRoot();
        if (robotRoot == null)
        {
            return;
        }

        Ur5ControlBootstrap bootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        if (bootstrap == null)
        {
            return;
        }

        Ur5EditorSpectatorCamera spectatorCamera = bootstrap.GetComponent<Ur5EditorSpectatorCamera>();
        if (spectatorCamera == null)
        {
            spectatorCamera = bootstrap.gameObject.AddComponent<Ur5EditorSpectatorCamera>();
        }

        spectatorCamera.robotRoot = robotRoot;
        spectatorCamera.tcpTarget = GameObject.Find("TcpTarget")?.transform;
    }

    private static void InstallControllerVisualizer()
    {
        Ur5ControlBootstrap bootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        if (bootstrap == null)
        {
            return;
        }

        Quest3ControllerVisualizer visualizer = bootstrap.GetComponent<Quest3ControllerVisualizer>();
        if (visualizer == null)
        {
            visualizer = bootstrap.gameObject.AddComponent<Quest3ControllerVisualizer>();
        }

        GameObject xrOrigin = GameObject.Find("XR Origin (VR)");
        visualizer.xrOrigin = xrOrigin != null ? xrOrigin.transform : null;
        visualizer.SetVirtualControllerVisibility(
            !IsPassthroughEnabled() || bootstrap.showVirtualControllersInPassthrough);
    }

    private static bool IsPassthroughEnabled()
    {
        Transform xrOrigin = FindXrOrigin();
        return xrOrigin != null && xrOrigin.GetComponent("OVRPassthroughLayer") != null;
    }

    private static bool ShouldDisablePoseIkForVelocityTeleop()
    {
        Ur5ControlBootstrap bootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        if (bootstrap == null
            || !bootstrap.enableCartesianVelocityTeleop
            || !bootstrap.disablePoseIkWhenVelocityTeleopEnabled)
        {
            return false;
        }

        Ur5UrScriptSpeedlClient speedlClient = bootstrap.GetComponent<Ur5UrScriptSpeedlClient>();
        return speedlClient != null && speedlClient.enableRealRobotOutput;
    }

    private static void DisablePoseIkControllers()
    {
        Transform robotRoot = FindRobotRoot();
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

        GameObject target = GameObject.Find("TcpTarget");
        if (target != null)
        {
            Ur5ActualTcpMarker actualMarker = target.GetComponent<Ur5ActualTcpMarker>();
            if (actualMarker != null)
            {
                actualMarker.enabled = false;
            }

            Quest3TcpTargetController questPoseController = target.GetComponent<Quest3TcpTargetController>();
            if (questPoseController != null)
            {
                questPoseController.enabled = false;
            }
        }
    }

    private static Transform FindXrOrigin()
    {
        GameObject xrOrigin = GameObject.Find("XR Origin (VR)");
        return xrOrigin != null ? xrOrigin.transform : null;
    }

    private static void ApplyQuestRuntimeTiming()
    {
        float refreshRate = ResolveDisplayRefreshRate();
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = Mathf.RoundToInt(refreshRate);
        Time.fixedDeltaTime = 1.0f / refreshRate;
        Time.maximumDeltaTime = Mathf.Max(Time.fixedDeltaTime * 4.0f, Time.fixedDeltaTime);
    }

    private static void ApplyQuestFloorTrackingOrigin()
    {
        SubsystemManager.GetSubsystems(InputSubsystems);
        foreach (XRInputSubsystem inputSubsystem in InputSubsystems)
        {
            if (inputSubsystem == null || !inputSubsystem.running)
            {
                continue;
            }

            // 将 Unity 世界的 Y=0 固定为 Quest Guardian 地面，避免设备原点
            // 落在头部高度时让机器人模型悬浮在手柄上方。
            inputSubsystem.TrySetTrackingOriginMode(TrackingOriginModeFlags.Floor);
        }
    }

    private static void ConfigurePassthroughCameraBackground()
    {
        Transform xrOrigin = FindXrOrigin();
        if (xrOrigin == null || xrOrigin.GetComponent("OVRPassthroughLayer") == null)
        {
            return;
        }

        Camera xrCamera = xrOrigin.GetComponentInChildren<Camera>(true);
        if (xrCamera == null)
        {
            return;
        }

        // Underlay Passthrough 只能透过 Unity 眼睛缓冲区的透明区域显示。
        // 即使 Inspector 被手动改动，也在运行时强制使用透明纯色背景。
        Color transparentBackground = xrCamera.backgroundColor;
        transparentBackground.a = 0.0f;
        xrCamera.clearFlags = CameraClearFlags.SolidColor;
        xrCamera.backgroundColor = transparentBackground;
    }

    private static float ResolveDisplayRefreshRate()
    {
        float refreshRate = FallbackQuestRefreshRate;
        SubsystemManager.GetSubsystems(DisplaySubsystems);
        foreach (XRDisplaySubsystem displaySubsystem in DisplaySubsystems)
        {
            if (displaySubsystem == null || !displaySubsystem.running)
            {
                continue;
            }

            if (displaySubsystem.TryGetDisplayRefreshRate(out float displayRefreshRate)
                && displayRefreshRate > 1.0f)
            {
                refreshRate = displayRefreshRate;
                break;
            }
        }

        return Mathf.Clamp(refreshRate, 60.0f, 120.0f);
    }

    private static Transform FindRobotRoot()
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
}

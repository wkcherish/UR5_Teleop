using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR;

public static class Ur5AutoSceneBootstrap
{
    private const float FallbackQuestRefreshRate = 72.0f;
    private static readonly List<XRDisplaySubsystem> DisplaySubsystems = new List<XRDisplaySubsystem>();
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void InstallAfterSceneLoad()
    {
        ApplyQuestRuntimeTiming();
        InstallStabilizer();
        InstallTargetWorkspaceLimiter();
        InstallCartesianVelocityTeleop();
        InstallTcpTargetFollower();
        InstallGripperController();
        InstallSpectatorCamera();
        InstallControllerVisualizer();
        SceneManager.sceneLoaded -= HandleSceneLoaded;
        SceneManager.sceneLoaded += HandleSceneLoaded;
    }

    private static void HandleSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        ApplyQuestRuntimeTiming();
        InstallStabilizer();
        InstallTargetWorkspaceLimiter();
        InstallCartesianVelocityTeleop();
        InstallTcpTargetFollower();
        InstallGripperController();
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
        follower.pauseIkWhenVelocityTeleopIdle = follower.velocityTeleop != null;
        ApplyStableFollowerDefaults(follower);

        Ur5ActualTcpMarker actualMarker = target.GetComponent<Ur5ActualTcpMarker>();
        if (actualMarker == null)
        {
            actualMarker = target.AddComponent<Ur5ActualTcpMarker>();
        }

        actualMarker.enabled = true;
        actualMarker.follower = follower;
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
        jointController.maxDriveSpeedDegreesPerSecond = 90.0f;
        jointController.maxDriveAccelerationDegreesPerSecondSquared = 1800.0f;
        jointController.driveTargetToleranceDegrees = 0.005f;
        jointController.ApplyConfiguredDriveSettings();
    }

    private static void ApplyStableFollowerDefaults(Ur5TcpTargetFollower follower)
    {
        follower.useGripperPadCenter = false;
        follower.positionTolerance = 0.008f;
        follower.maxJointStepDegrees = 0.55f;
        follower.minimumJointDeltaDegrees = 0.030f;
        follower.maximumCommandLeadDegrees = 2.20f;
        follower.useTimedJointAssignments = true;
        follower.jointAssignmentIntervalSeconds = 0.03f;
        follower.dlsDamping = 0.45f;
        follower.dlsOrientationWeight = 0.35f;
        follower.dlsGain = 0.22f;
        follower.jointDeltaSmoothing = 0.74f;
        follower.rotationToleranceDegrees = 2.00f;
        follower.rotationBlend = 0.24f;
        follower.maxWristStepDegrees = 0.45f;
        follower.holdJointPoseWhenTargetSettled = true;
        follower.targetStationaryHoldSeconds = 0.12f;
        follower.targetStationaryPositionEpsilon = 0.0015f;
        follower.targetStationaryRotationEpsilonDegrees = 0.35f;
        follower.settledPositionError = 0.010f;
        follower.settledRotationErrorDegrees = 2.50f;
    }

    private static void ApplyStableTrajectoryDefaults(Ur5JointTrajectoryPlayer trajectoryPlayer)
    {
        trajectoryPlayer.play = true;
        trajectoryPlayer.queueMode = Ur5JointTrajectoryPlayer.QueueMode.LatestOnly;
        trajectoryPlayer.jointAssignmentIntervalSeconds = 0.03f;
        trajectoryPlayer.applyDirectlyToDrive = true;
        trajectoryPlayer.clampToDriveLimits = true;
        trajectoryPlayer.maxQueuedWaypoints = 1;
    }

    private static void ApplyStableVelocityTeleopDefaults(Ur5CartesianVelocityTeleopController velocityTeleop)
    {
        velocityTeleop.unityPreviewMode = Ur5CartesianVelocityTeleopController.UnityPreviewMode.RelativePoseTarget;
        velocityTeleop.relativePreviewPositionScale = 0.75f;
        velocityTeleop.relativePreviewRotationScale = 0.90f;
        velocityTeleop.previewMaxLinearSpeed = 0.14f;
        velocityTeleop.previewMaxAngularSpeedDegreesPerSecond = 150.0f;
        velocityTeleop.previewPositionSmoothingSharpness = 16.0f;
        velocityTeleop.previewRotationSmoothingSharpness = 18.0f;
        velocityTeleop.linearSpeedGain = 0.55f;
        velocityTeleop.maxLinearSpeed = 0.06f;
        velocityTeleop.maxLinearAcceleration = 0.16f;
        velocityTeleop.angularSpeedGain = 0.75f;
        velocityTeleop.maxAngularSpeedRadiansPerSecond = 0.45f;
        velocityTeleop.maxAngularAcceleration = 0.90f;
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

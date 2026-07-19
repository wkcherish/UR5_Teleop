using UnityEngine;

public class Ur5ControlBootstrap : MonoBehaviour
{
    [Header("Scene References")]
    public Transform robotRoot;
    public Transform tcpTarget;
    public bool createTcpTargetIfMissing = true;

    [Header("Legacy Target/IK Control")]
    public bool enableKeyboardControl = true;
    public bool enableQuest3Control = true;
    public bool enableTcpTargetFollower = true;

    [Header("Cartesian Velocity Teleop")]
    public bool enableCartesianVelocityTeleop = true;
    [Tooltip("Disable the Unity IK visual follower only after real robot feedback is driving the digital twin. Keep false during Unity-only testing so the arm model follows the velocity-driven TcpTarget.")]
    public bool disablePoseIkWhenVelocityTeleopEnabled = false;
    public bool addUrScriptSpeedlClient = true;

    private void Awake()
    {
        if (robotRoot == null)
        {
            robotRoot = FindRobotRoot();
        }

        DisableLegacyUrdfImporterController();
        ResolveTcpTarget();
        ConfigureTcpTargetSafety();

        bool useVelocityTeleop = enableCartesianVelocityTeleop;
        if (enableKeyboardControl && tcpTarget != null && tcpTarget.GetComponent<TcpTargetKeyboardController>() == null)
        {
            tcpTarget.gameObject.AddComponent<TcpTargetKeyboardController>();
        }

        if (useVelocityTeleop)
        {
            DisableQuestPoseController();
            DisableLegacyVrTeleoperationControllers();
        }
        else if (enableQuest3Control && tcpTarget != null && tcpTarget.GetComponent<Quest3TcpTargetController>() == null)
        {
            tcpTarget.gameObject.AddComponent<Quest3TcpTargetController>();
        }

        Ur5PhysicsStabilizer stabilizer = GetComponent<Ur5PhysicsStabilizer>();
        if (stabilizer == null)
        {
            stabilizer = gameObject.AddComponent<Ur5PhysicsStabilizer>();
        }

        stabilizer.robotRoot = robotRoot;
        stabilizer.Stabilize();

        Ur5ArticulationJointController jointController = ResolveJointController();
        Ur5JointTrajectoryPlayer trajectoryPlayer = ConfigureJointTrajectoryPlayer(jointController);
        ConfigureVelocityTeleop(useVelocityTeleop);

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
        ConfigureSpectatorCamera();
        ConfigureControllerVisualizer();
        ConfigureRecorder(jointController, trajectoryPlayer, follower);
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

        TcpTargetCollisionGuard collisionGuard = tcpTarget.GetComponent<TcpTargetCollisionGuard>();
        if (collisionGuard == null)
        {
            collisionGuard = tcpTarget.gameObject.AddComponent<TcpTargetCollisionGuard>();
        }

        collisionGuard.robotRoot = robotRoot;
        collisionGuard.preventObstaclePenetration = false;
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

    private void ApplyStableJointDefaults(Ur5ArticulationJointController jointController)
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
        trajectoryPlayer.jointAssignmentIntervalSeconds = 0.03f;
        trajectoryPlayer.applyDirectlyToDrive = true;
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
        follower.questController = tcpTarget != null
            ? tcpTarget.GetComponent<Quest3TcpTargetController>()
            : null;
        follower.velocityTeleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        follower.pauseIkWhenVelocityTeleopIdle = follower.velocityTeleop != null;
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

        Ur5ActualTcpMarker actualMarker = tcpTarget.GetComponent<Ur5ActualTcpMarker>();
        if (actualMarker == null)
        {
            actualMarker = tcpTarget.gameObject.AddComponent<Ur5ActualTcpMarker>();
        }

        actualMarker.enabled = true;
        actualMarker.follower = follower;
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

        if (addUrScriptSpeedlClient)
        {
            Ur5UrScriptSpeedlClient speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
            if (speedlClient == null)
            {
                speedlClient = gameObject.AddComponent<Ur5UrScriptSpeedlClient>();
            }

            velocityTeleop.speedlClient = speedlClient;
        }
    }

    private bool IsRealRobotOutputEnabled()
    {
        Ur5UrScriptSpeedlClient speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
        return speedlClient != null && speedlClient.enableRealRobotOutput;
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

    private void ConfigureControllerVisualizer()
    {
        Quest3ControllerVisualizer controllerVisualizer = GetComponent<Quest3ControllerVisualizer>();
        if (controllerVisualizer == null)
        {
            controllerVisualizer = gameObject.AddComponent<Quest3ControllerVisualizer>();
        }

        controllerVisualizer.xrOrigin = FindXrOrigin();
    }

    private void ConfigureRecorder(
        Ur5ArticulationJointController jointController,
        Ur5JointTrajectoryPlayer trajectoryPlayer,
        Ur5TcpTargetFollower follower)
    {
        Ur5PoseCsvRecorder recorder = GetComponent<Ur5PoseCsvRecorder>();
        if (recorder == null)
        {
            recorder = gameObject.AddComponent<Ur5PoseCsvRecorder>();
        }

        recorder.tcpTarget = tcpTarget;
        recorder.jointController = jointController;
        recorder.trajectoryPlayer = trajectoryPlayer;
        recorder.tcpFollower = follower != null
            ? follower
            : FindObjectOfType<Ur5TcpTargetFollower>();
        recorder.questController = tcpTarget != null ? tcpTarget.GetComponent<Quest3TcpTargetController>() : null;
        recorder.velocityTeleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        recorder.speedlClient = GetComponent<Ur5UrScriptSpeedlClient>();
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

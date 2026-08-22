using System;
using System.IO;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.XR;

public class Ur5Continuous6DofBootstrapTests
{
    [Test]
    public void DefaultQuestProfile_UsesUr10StyleRightHandAnchoredPoseClutch()
    {
        GameObject owner = new GameObject("bootstrap-profile-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);

            Assert.IsTrue(teleop.enableUr10StyleAnchoredPoseClutch);
            Assert.IsFalse(teleop.enableContinuous6DofClutch);
            Assert.IsTrue(teleop.UsesUr10StyleAnchoredPoseClutch);
            Assert.AreEqual(XRNode.RightHand, teleop.positionControllerNode);
            Assert.AreEqual(XRNode.RightHand, teleop.rotationControllerNode);
            Assert.AreEqual(XRNode.LeftHand, teleop.safetyControllerNode);
            Assert.IsTrue(teleop.usePositionGripAsDeadman);
            Assert.IsTrue(teleop.useRotationGripAsDeadman);
            Assert.IsTrue(teleop.useAnchoredPoseTeleopStrategy);
            Assert.AreEqual(
                Ur5CartesianVelocityTeleopController.RotationInputMode.Locked,
                teleop.rotationInputMode);
            Assert.AreEqual(0.70f, teleop.anchoredPoseSmoothingStep, 0.000001f);
            Assert.AreEqual(0.70f, teleop.anchoredPosePrecisionSmoothingStep, 0.000001f);
            Assert.AreEqual(1.00f, teleop.relativePreviewPositionScale, 0.000001f);
            Assert.AreEqual(1.00f, teleop.normalPositionScale, 0.000001f);
            Assert.AreEqual(1.0f, teleop.relativePreviewRotationScale, 0.000001f);

            Assert.IsFalse(teleop.enableThreeModeController);
            Assert.IsFalse(teleop.useRightSecondaryButtonForInsertMode);
            Assert.IsFalse(teleop.enableAPrecisionModifier);
            Assert.IsFalse(teleop.enableFineControlButton);
            Assert.IsFalse(teleop.enableLeftSecondaryPoseRotation);
            Assert.IsFalse(teleop.enableLeftSecondaryOrientationHold);
            Assert.IsFalse(teleop.snapGraspApproachToVertical);

            Assert.IsTrue(teleop.filterControllerPosition);
            Assert.IsFalse(teleop.useAdaptiveControllerPositionFilter);
            Assert.AreEqual(0.0025f, teleop.controllerPositionJitterDeadbandMeters, 0.000001f);
            Assert.AreEqual(1.0f, teleop.angularDeadbandDegrees, 0.000001f);
            Assert.AreEqual(0.0f, teleop.previewTargetDeadbandMeters, 0.000001f);
            Assert.AreEqual(0.0f, teleop.finePreviewTargetDeadbandMeters, 0.000001f);
            Assert.IsFalse(teleop.useAccelerationLimitedPreviewTrajectory);
            Assert.IsTrue(teleop.limitPreviewLeadToActualTcp);
            Assert.AreEqual(0.040f, teleop.maximumPreviewLeadMeters, 0.000001f);
            Assert.AreEqual(0.160f, teleop.movingPreviewLeadMeters, 0.000001f);
            Assert.IsTrue(teleop.freezeRobotWhenPositionHandStops);
            Assert.AreEqual(0.0025f, teleop.controllerMotionEpsilonMeters, 0.000001f);
            Assert.AreEqual(0.04f, teleop.controllerStopHoldSeconds, 0.000001f);
            Assert.IsFalse(teleop.useRelativePoseCommandFilter);
            Assert.IsFalse(teleop.useProgressivePositionResponse);

            Assert.IsFalse(teleop.enableLeftPrimarySnapDown);
            Assert.IsFalse(teleop.enableLeftPrimaryReadyPose);
            Assert.AreEqual(0.45f, teleop.leftPrimaryReadyPoseHoldSeconds, 0.000001f);
            Assert.AreEqual(3.0f, teleop.leftPrimarySnapTimeoutSeconds, 0.000001f);
            Assert.AreEqual(0.003f, teleop.leftPrimarySnapPositionToleranceMeters, 0.000001f);
            Assert.AreEqual(0.50f, teleop.leftPrimarySnapRotationToleranceDegrees, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void ControlBootstrap_DefaultFollowerUsesMeasuredStateFastProfileDuringGripHold()
    {
        GameObject owner = new GameObject("follower-profile-test");
        GameObject robot = new GameObject("follower-profile-robot");
        GameObject target = new GameObject("TcpTarget");
        owner.SetActive(false);
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            bootstrap.robotRoot = robot.transform;
            bootstrap.tcpTarget = target.transform;
            bootstrap.enableCartesianVelocityTeleop = true;
            bootstrap.enableTcpTargetFollower = true;
            bootstrap.enableQuest3Control = false;
            bootstrap.enableKeyboardControl = false;
            bootstrap.addUrScriptSpeedlClient = false;

            InvokeNonPublic(bootstrap, "Awake");

            var follower = robot.GetComponent<Ur5TcpTargetFollower>();
            Assert.IsNotNull(follower);
            Assert.IsFalse(follower.enablePrecisionAssemblyTracking);
            Assert.IsFalse(follower.holdJointPoseWhenTargetSettled);
            Assert.IsTrue(follower.suppressRotationOnlyIkDuringPositionControl);
            Assert.IsTrue(follower.useMeasuredStateTeleopSolve);
            Assert.AreEqual(4.0f, follower.maximumCommandLeadDegrees, 0.000001f);
            Assert.AreEqual(8.0f, follower.maximumWristCommandLeadDegrees, 0.000001f);
            Assert.AreEqual(3.85f * Ur5AutoSceneBootstrap.TeleopControlRateHz, follower.maxJointSpeedDegreesPerSecond, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void ControlBootstrap_DefaultWorkspaceAllowsLowTabletopTcpWithoutGoingBelowGround()
    {
        GameObject owner = new GameObject("workspace-floor-profile-test");
        GameObject robot = new GameObject("workspace-floor-profile-robot");
        GameObject target = new GameObject("TcpTarget");
        owner.SetActive(false);
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            bootstrap.robotRoot = robot.transform;
            bootstrap.tcpTarget = target.transform;
            bootstrap.enableCartesianVelocityTeleop = true;
            bootstrap.enableTcpTargetFollower = false;
            bootstrap.enableQuest3Control = false;
            bootstrap.enableKeyboardControl = false;
            bootstrap.addUrScriptSpeedlClient = false;

            InvokeNonPublic(bootstrap, "Awake");

            var limiter = target.GetComponent<TcpTargetWorkspaceLimiter>();
            Assert.IsNotNull(limiter);
            Assert.AreEqual(0.015f, limiter.minimumLocalPosition.y, 0.000001f);
            Assert.GreaterOrEqual(limiter.minimumLocalPosition.y, 0.0f);
            Assert.AreEqual(0.015f, bootstrap.workspaceMinimumLocalHeightMeters, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void ControlBootstrap_DefaultFollowerUsesWristPriorityForGripARotationAdjust()
    {
        GameObject robot = new GameObject("follower-grip-a-wrist-priority-test");
        try
        {
            var follower = robot.AddComponent<Ur5TcpTargetFollower>();

            Ur5ControlBootstrap.ApplyDefaultFollowerProfile(follower);

            Assert.IsTrue(
                (bool)GetPublicFieldValue(follower, "wristPriorityDuringRotationAdjust"),
                "Grip+A is explicit end-effector attitude control; the follower must bias the solve toward wrist joints instead of letting shoulder/elbow dominate.");
            Assert.AreEqual(
                0.02f,
                (float)GetPublicFieldValue(follower, "rotationAdjustProximalJointWeight"),
                0.000001f);
            Assert.AreEqual(
                0.15f,
                (float)GetPublicFieldValue(follower, "rotationAdjustPositionTaskWeight"),
                0.000001f);
            Assert.AreEqual(
                1.60f,
                (float)GetPublicFieldValue(follower, "rotationAdjustDlsDampingMultiplier"),
                0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(robot);
        }
    }

    [Test]
    public void ControlBootstrap_StableJointDefaultsAllowResponsiveWristPoseTracking()
    {
        GameObject owner = new GameObject("joint-defaults-test");
        try
        {
            var jointController = owner.AddComponent<Ur5ArticulationJointController>();

            Ur5ControlBootstrap.ApplyStableJointDefaults(jointController);

            Assert.IsTrue(jointController.limitDriveTargetLeadFromMeasuredJoint);
            Assert.AreEqual(6.0f, jointController.maximumDriveTargetLeadDegrees, 0.000001f);
            Assert.AreEqual(6.0f, jointController.maximumWristDriveTargetLeadDegrees, 0.000001f);
            Assert.IsTrue(jointController.useDirectJointStateForMeasuredTeleop);
            Assert.AreEqual(3.85f, jointController.measuredStateTeleopMaximumJointStepDegrees, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void Ur10StylePreview_DefaultQuestMappingTracksControllerTranslationOneToOne()
    {
        GameObject owner = new GameObject("ur10-style-default-mapping-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            target.transform.SetPositionAndRotation(startPosition, Quaternion.identity);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, new Vector3(0.04f, 0.0f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            AssertVectorNear(startPosition + new Vector3(0.04f, 0.0f, 0.0f), target.transform.position, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StylePreview_RuntimeQuestSpeedProfileScalesAnchoredTranslation()
    {
        GameObject owner = new GameObject("ur10-style-runtime-speed-profile-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            bootstrap.questTranslationScale = 1.50f;
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);
            bootstrap.ApplyQuestTeleopSpeedProfile(teleop);
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            target.transform.SetPositionAndRotation(startPosition, Quaternion.identity);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, new Vector3(0.04f, 0.0f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            Assert.AreEqual(1.50f, bootstrap.EffectiveQuestTranslationScale, 0.000001f);
            Assert.AreEqual(bootstrap.EffectiveQuestTranslationScale, teleop.normalPositionScale, 0.000001f);
            AssertVectorNear(
                startPosition + new Vector3(0.04f * bootstrap.EffectiveQuestTranslationScale, 0.0f, 0.0f),
                target.transform.position,
                0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void ControlBootstrap_DefaultQuestSpeedProfileUsesResponsiveStableTranslation()
    {
        GameObject owner = new GameObject("quest-responsive-speed-profile-test");
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();

            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);
            bootstrap.ApplyQuestTeleopSpeedProfile(teleop);

            Assert.AreEqual(2.10f, bootstrap.EffectiveQuestTranslationScale, 0.000001f);
            Assert.AreEqual(2.10f, teleop.normalPositionScale, 0.000001f);
            Assert.AreEqual(0.160f, bootstrap.EffectiveQuestMovingPreviewLeadMeters, 0.000001f);
            Assert.AreEqual(0.160f, teleop.movingPreviewLeadMeters, 0.000001f);
            Assert.AreEqual(0.32f, teleop.maxLinearSpeed, 0.000001f);
            Assert.AreEqual(1.60f, teleop.maxLinearAcceleration, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void SampleScene_SerializesTheSameQuestSpeedProfileUsedByRuntimeBootstrap()
    {
        string scenePath = Path.Combine(Application.dataPath, "Scenes/SampleScene.scene");
        string sceneText = File.ReadAllText(scenePath);

        StringAssert.Contains("questTranslationScale: 2.1", sceneText);
        StringAssert.Contains("questMaximumPreviewLeadMeters: 0.04", sceneText);
        StringAssert.Contains("questMovingPreviewLeadMeters: 0.16", sceneText);
        StringAssert.Contains("questCommandMaxLinearSpeed: 0.32", sceneText);
        StringAssert.Contains("questCommandMaxLinearAcceleration: 1.6", sceneText);
        StringAssert.Contains("questMaxJointStepDegrees: 3.85", sceneText);
        StringAssert.Contains("workspaceMinimumLocalHeightMeters: 0.015", sceneText);
        StringAssert.Contains("realRobotMaxLinearSpeed: 0.035", sceneText);
        StringAssert.Contains("realRobotMaxAngularSpeedRadiansPerSecond: 0.25", sceneText);
        StringAssert.Contains("realRobotCommandAcceleration: 0.12", sceneText);
        StringAssert.Contains("realRobotStopAcceleration: 0.35", sceneText);
        StringAssert.Contains("realRobotMaxOutputLinearAcceleration: 0.1", sceneText);
        StringAssert.Contains("realRobotMaxOutputAngularAcceleration: 0.7", sceneText);
    }

    [Test]
    public void ControlBootstrap_RealRobotSpeedlSafetyProfileIsIndependentFromQuestPreviewSpeed()
    {
        GameObject owner = new GameObject("real-speedl-safety-profile-test");
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            bootstrap.questCommandMaxLinearSpeed = 0.32f;
            bootstrap.realRobotMaxLinearSpeed = 0.035f;
            bootstrap.realRobotMaxAngularSpeedRadiansPerSecond = 0.25f;
            bootstrap.realRobotCommandAcceleration = 0.12f;
            bootstrap.realRobotStopAcceleration = 0.35f;
            bootstrap.realRobotMaxOutputLinearAcceleration = 0.10f;
            bootstrap.realRobotMaxOutputAngularAcceleration = 0.70f;

            var speedlClient = owner.AddComponent<Ur5UrScriptSpeedlClient>();
            speedlClient.maxLinearSpeed = 0.50f;
            speedlClient.maxAngularSpeedRadiansPerSecond = 2.00f;
            speedlClient.armOnStart = true;

            bootstrap.ApplyRealRobotSpeedlSafetyProfile(speedlClient);

            Assert.IsTrue(speedlClient.requireMotionArmed);
            Assert.IsFalse(speedlClient.armOnStart);
            Assert.AreEqual(0.035f, speedlClient.maxLinearSpeed, 0.000001f);
            Assert.AreEqual(0.25f, speedlClient.maxAngularSpeedRadiansPerSecond, 0.000001f);
            Assert.AreEqual(0.12f, speedlClient.acceleration, 0.000001f);
            Assert.AreEqual(0.35f, speedlClient.stopAcceleration, 0.000001f);
            Assert.IsTrue(speedlClient.limitOutputAcceleration);
            Assert.AreEqual(0.10f, speedlClient.maxOutputLinearAcceleration, 0.000001f);
            Assert.AreEqual(0.70f, speedlClient.maxOutputAngularAcceleration, 0.000001f);
            Assert.IsTrue(speedlClient.sendStoplOnStop);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void Ur10StylePreview_WhenGripIsReleased_DoesNotWriteTheTcpTarget()
    {
        GameObject owner = new GameObject("ur10-style-release-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.tcpPreviewTarget = target.transform;
            Vector3 expectedPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Quaternion expectedRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
            target.transform.SetPositionAndRotation(expectedPosition, expectedRotation);

            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            Assert.LessOrEqual(Vector3.Distance(expectedPosition, target.transform.position), 0.000001f);
            Assert.LessOrEqual(Quaternion.Angle(expectedRotation, target.transform.rotation), 0.0001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StylePreview_WhenHandTargetRunsAhead_ClampsCommandLeadToActualTcp()
    {
        GameObject owner = new GameObject("ur10-style-lead-clamp-test");
        GameObject robot = new GameObject("ur10-style-lead-clamp-robot");
        GameObject target = new GameObject("TcpTarget");
        GameObject endEffector = new GameObject("tool0");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.normalPositionScale = 2.10f;
            teleop.limitPreviewLeadToActualTcp = true;
            teleop.maximumPreviewLeadMeters = 0.040f;
            teleop.movingPreviewLeadMeters = 0.040f;

            var follower = robot.AddComponent<Ur5TcpTargetFollower>();
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;
            SetNonPublicField(teleop, "tcpFollower", follower);

            Vector3 actualPosition = new Vector3(0.35f, 0.30f, 0.22f);
            target.transform.SetPositionAndRotation(actualPosition, Quaternion.identity);
            endEffector.transform.SetPositionAndRotation(actualPosition, Quaternion.identity);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, new Vector3(0.50f, 0.0f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            Assert.LessOrEqual(
                Vector3.Distance(actualPosition, target.transform.position),
                0.0401f,
                "Quest hand motion must not leave TcpTarget hundreds of millimetres ahead of the simulated arm; otherwise the arm keeps chasing after the hand stops.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
            UnityEngine.Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void Ur10StylePreview_WhenHandIsMoving_UsesResponsiveMovingLeadWindow()
    {
        GameObject owner = new GameObject("ur10-style-moving-lead-test");
        GameObject robot = new GameObject("ur10-style-moving-lead-robot");
        GameObject target = new GameObject("TcpTarget");
        GameObject endEffector = new GameObject("tool0");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.normalPositionScale = 2.10f;
            teleop.limitPreviewLeadToActualTcp = true;
            teleop.maximumPreviewLeadMeters = 0.040f;
            teleop.movingPreviewLeadMeters = 0.160f;
            teleop.freezeRobotWhenPositionHandStops = true;
            teleop.controllerMotionEpsilonMeters = 0.001f;

            var follower = robot.AddComponent<Ur5TcpTargetFollower>();
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;
            SetNonPublicField(teleop, "tcpFollower", follower);

            Vector3 actualPosition = new Vector3(0.35f, 0.30f, 0.22f);
            target.transform.SetPositionAndRotation(actualPosition, Quaternion.identity);
            endEffector.transform.SetPositionAndRotation(actualPosition, Quaternion.identity);

            SetUr10StyleLatestPose(teleop, Vector3.zero, Quaternion.identity);
            SetUr10StyleLatestPose(teleop, new Vector3(0.50f, 0.0f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            float commandLead = Vector3.Distance(actualPosition, target.transform.position);
            Assert.Greater(
                commandLead,
                0.120f,
                "While the hand is intentionally moving, the preview target should not be squeezed into the stop-hold safety window; otherwise large hand motion produces only tiny robot motion.");
            Assert.LessOrEqual(commandLead, 0.1601f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
            UnityEngine.Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void Ur10StylePreview_WhenStopHoldIsActive_ReturnsToSafetyLeadWindow()
    {
        GameObject owner = new GameObject("ur10-style-stop-hold-lead-test");
        GameObject robot = new GameObject("ur10-style-stop-hold-lead-robot");
        GameObject target = new GameObject("TcpTarget");
        GameObject endEffector = new GameObject("tool0");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.normalPositionScale = 2.10f;
            teleop.limitPreviewLeadToActualTcp = true;
            teleop.maximumPreviewLeadMeters = 0.040f;
            teleop.movingPreviewLeadMeters = 0.160f;

            var follower = robot.AddComponent<Ur5TcpTargetFollower>();
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;
            SetNonPublicField(teleop, "tcpFollower", follower);

            Vector3 actualPosition = new Vector3(0.35f, 0.30f, 0.22f);
            target.transform.SetPositionAndRotation(actualPosition, Quaternion.identity);
            endEffector.transform.SetPositionAndRotation(actualPosition, Quaternion.identity);

            SetUr10StyleLatestPose(teleop, Vector3.zero, Quaternion.identity);
            SetNonPublicField(teleop, "positionHandStopHoldActive", true);
            SetUr10StyleLatestPose(teleop, new Vector3(0.50f, 0.0f, 0.0f), Quaternion.identity);
            SetNonPublicField(teleop, "positionHandStopHoldActive", true);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            Assert.LessOrEqual(
                Vector3.Distance(actualPosition, target.transform.position),
                0.0401f,
                "Once stop-hold is active, the stale target must collapse back to the safety lead window even if the hand remains displaced.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
            UnityEngine.Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void Ur10StylePreview_UsesConfiguredSmoothingStepForResponsiveTracking()
    {
        GameObject owner = new GameObject("ur10-style-smoothing-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.angularDeadbandDegrees = 0.0f;
            teleop.normalPositionScale = 0.50f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Quaternion startRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
            target.transform.SetPositionAndRotation(startPosition, startRotation);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, new Vector3(0.10f, 0.0f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            AssertVectorNear(startPosition + new Vector3(0.05f, 0.0f, 0.0f), target.transform.position, 0.000001f);
            Assert.LessOrEqual(Quaternion.Angle(startRotation, target.transform.rotation), 0.0001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StylePreview_RebasesAtWorkspaceFloorSoReverseMotionLeavesBoundaryImmediately()
    {
        GameObject owner = new GameObject("ur10-style-workspace-rebase-test");
        GameObject robotRoot = new GameObject("ur10-style-workspace-rebase-robot");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.normalPositionScale = 1.0f;

            var limiter = target.AddComponent<TcpTargetWorkspaceLimiter>();
            limiter.robotRoot = robotRoot.transform;
            limiter.minimumLocalPosition = new Vector3(-1.0f, 0.05f, -1.0f);
            limiter.maximumLocalPosition = Vector3.one;
            limiter.keepAwayFromBase = false;
            teleop.workspaceLimiter = limiter;

            target.transform.SetPositionAndRotation(
                new Vector3(0.0f, 0.20f, 0.0f),
                Quaternion.identity);

            SetUr10StyleLatestPose(teleop, Vector3.zero, Quaternion.identity);
            SetUr10StyleLatestPose(teleop, new Vector3(0.0f, -0.30f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");
            Assert.AreEqual(0.05f, target.transform.position.y, 0.000001f);

            SetUr10StyleLatestPose(teleop, new Vector3(0.0f, -0.29f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            Assert.AreEqual(
                0.06f,
                target.transform.position.y,
                0.000001f,
                "Once the constrained target is re-anchored, a 1 cm reverse hand motion must leave the workspace floor without waiting to cancel the prior overtravel.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robotRoot);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StylePreview_LockedRotationModeTracksTranslationWithoutChangingGripperAttitude()
    {
        GameObject owner = new GameObject("ur10-style-locked-rotation-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.angularDeadbandDegrees = 0.0f;
            teleop.normalPositionScale = 0.50f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Quaternion startRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
            Quaternion incidentalControllerRotation = Quaternion.Euler(0.0f, 35.0f, 0.0f);
            target.transform.SetPositionAndRotation(startPosition, startRotation);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, new Vector3(0.10f, 0.0f, 0.0f), incidentalControllerRotation);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            AssertVectorNear(startPosition + new Vector3(0.05f, 0.0f, 0.0f), target.transform.position, 0.000001f);
            Assert.LessOrEqual(
                Quaternion.Angle(startRotation, target.transform.rotation),
                0.0001f,
                "Grip-only translation must not turn the gripper when the user has not explicitly requested orientation control.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StylePreview_GripOnlyReportsPositionOrientationLockForFollower()
    {
        GameObject owner = new GameObject("ur10-style-follower-orientation-lock-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            target.transform.SetPositionAndRotation(
                new Vector3(0.40f, 0.50f, 0.60f),
                Quaternion.Euler(5.0f, 10.0f, 15.0f));

            SetUr10StyleLatestPose(teleop, Vector3.zero, Quaternion.identity);

            Assert.IsTrue(
                teleop.IsPositionOrientationLocked,
                "UR10-style Grip-only translation must expose a full orientation lock so the follower uses its strict wrist-stabilized IK path.");

            SetNonPublicField(teleop, "ur10StyleRotationAdjustActive", true);
            Assert.IsFalse(
                teleop.IsPositionOrientationLocked,
                "Grip+A is explicit orientation adjustment, not Grip-only locked-orientation translation.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StyleRotationAdjustment_LockedModeUsesRightPrimaryButtonNotSecondary()
    {
        Assert.IsTrue(Ur5CartesianVelocityTeleopController.ShouldAdjustUr10StyleRotation(
            Ur5CartesianVelocityTeleopController.RotationInputMode.Locked,
            primaryButtonPressed: true,
            secondaryButtonPressed: false));
        Assert.IsFalse(Ur5CartesianVelocityTeleopController.ShouldAdjustUr10StyleRotation(
            Ur5CartesianVelocityTeleopController.RotationInputMode.Locked,
            primaryButtonPressed: false,
            secondaryButtonPressed: true));
        Assert.IsTrue(Ur5CartesianVelocityTeleopController.ShouldAdjustUr10StyleRotation(
            Ur5CartesianVelocityTeleopController.RotationInputMode.ControllerPoseDelta,
            primaryButtonPressed: false,
            secondaryButtonPressed: false));
    }

    [Test]
    public void Ur10StyleRotationAdjustment_LockedModeKeepsDownwardGraspWhileYawing()
    {
        GameObject owner = new GameObject("ur10-style-downward-yaw-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.angularDeadbandDegrees = 0.0f;
            teleop.normalPositionScale = 0.50f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Quaternion startRotation = Quaternion.LookRotation(Vector3.down, Vector3.forward);
            Quaternion controllerRotation = Quaternion.AngleAxis(-45.0f, Vector3.down)
                * Quaternion.AngleAxis(35.0f, Vector3.right);
            target.transform.SetPositionAndRotation(startPosition, startRotation);

            SetUr10StyleLatestPose(teleop, Vector3.zero, Quaternion.identity);
            SetNonPublicField(teleop, "ur10StyleRotationAdjustActive", true);
            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleCommandPose",
                Vector3.zero,
                Quaternion.identity,
                true);
            InvokeNonPublic(teleop, "CaptureUr10StyleRotationAdjustReference");
            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleCommandPose",
                new Vector3(0.10f, 0.0f, 0.0f),
                controllerRotation,
                false);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            AssertVectorNear(
                startPosition,
                target.transform.position,
                0.000001f);
            Assert.LessOrEqual(
                Vector3.Angle(Vector3.down, target.transform.rotation * Vector3.forward),
                0.001f,
                "Locked Grip+A must keep the grasp approach vertical instead of copying controller pitch/roll into a backward lean.");

            Vector3 startJawReference = Vector3.ProjectOnPlane(startRotation * Vector3.up, Vector3.down).normalized;
            Vector3 adjustedJawReference = Vector3.ProjectOnPlane(target.transform.rotation * Vector3.up, Vector3.down).normalized;
            Assert.Greater(
                Vector3.Angle(startJawReference, adjustedJawReference),
                5.0f,
                "Locked Grip+A should still yaw the gripper around the downward grasp axis.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StylePreview_GripStartImmediatelyReanchorsTcpTargetToActualTool()
    {
        GameObject owner = new GameObject("ur10-style-grip-start-reanchor-test");
        GameObject robot = new GameObject("ur10-style-grip-start-reanchor-robot");
        GameObject target = new GameObject("TcpTarget");
        GameObject endEffector = new GameObject("tool0");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;

            var follower = robot.AddComponent<Ur5TcpTargetFollower>();
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;
            SetNonPublicField(teleop, "tcpFollower", follower);

            Vector3 staleTargetPosition = new Vector3(0.52f, 0.42f, 0.28f);
            Quaternion staleTargetRotation = Quaternion.Euler(30.0f, -20.0f, 10.0f);
            Vector3 actualToolPosition = new Vector3(0.35f, 0.30f, 0.22f);
            Quaternion actualToolRotation = Quaternion.Euler(5.0f, 15.0f, -25.0f);
            target.transform.SetPositionAndRotation(staleTargetPosition, staleTargetRotation);
            endEffector.transform.SetPositionAndRotation(actualToolPosition, actualToolRotation);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);

            AssertVectorNear(
                actualToolPosition,
                target.transform.position,
                0.000001f);
            Assert.LessOrEqual(
                Quaternion.Angle(actualToolRotation, target.transform.rotation),
                0.0001f,
                "Grip press must discard stale TcpTarget lead immediately, before the next IK frame can chase it.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
            UnityEngine.Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void Ur10StyleRotationAdjustment_WhenEnded_RebasesTranslationAnchorWithoutPositionJump()
    {
        GameObject owner = new GameObject("ur10-style-rotation-end-rebase-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.angularDeadbandDegrees = 0.0f;
            teleop.normalPositionScale = 0.50f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Quaternion startRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
            Quaternion adjustedControllerRotation = Quaternion.Euler(0.0f, 30.0f, 0.0f);
            target.transform.SetPositionAndRotation(startPosition, startRotation);

            SetUr10StyleLatestPose(teleop, Vector3.zero, Quaternion.identity);
            SetNonPublicField(teleop, "ur10StyleRotationAdjustActive", true);
            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleCommandPose",
                new Vector3(0.10f, 0.0f, 0.0f),
                adjustedControllerRotation,
                true);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            SetNonPublicField(teleop, "ur10StyleRotationAdjustActive", false);
            InvokeNonPublic(
                teleop,
                "RebaseUr10StyleAtCommandPose",
                new Vector3(0.10f, 0.0f, 0.0f),
                adjustedControllerRotation);
            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleCommandPose",
                new Vector3(0.12f, 0.0f, 0.0f),
                adjustedControllerRotation,
                false);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            AssertVectorNear(
                startPosition + new Vector3(0.01f, 0.0f, 0.0f),
                target.transform.position,
                0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StylePreview_MapsControllerRotationToTcpRotation()
    {
        GameObject owner = new GameObject("ur10-style-rotation-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.ControllerPoseDelta;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.angularDeadbandDegrees = 0.0f;
            teleop.normalPositionScale = 0.50f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Quaternion startRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
            Quaternion controllerRotation = Quaternion.Euler(0.0f, 30.0f, 0.0f);
            target.transform.SetPositionAndRotation(startPosition, startRotation);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, Vector3.zero, controllerRotation);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            Quaternion expectedRotation = controllerRotation * startRotation;
            AssertVectorNear(startPosition, target.transform.position, 0.000001f);
            Assert.LessOrEqual(Quaternion.Angle(expectedRotation, target.transform.rotation), 0.0001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StyleFreeze_RebasesAnchorBeforeNextDeliberateMove()
    {
        GameObject owner = new GameObject("ur10-style-freeze-rebase-test");
        GameObject target = new GameObject("TcpTarget");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.normalPositionScale = 0.50f;

            Vector3 startPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Quaternion startRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
            Vector3 heldHandPosition = new Vector3(0.10f, 0.0f, 0.0f);
            target.transform.SetPositionAndRotation(startPosition, startRotation);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, heldHandPosition, Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");
            Vector3 frozenTarget = target.transform.position;

            InvokeNonPublic(
                teleop,
                "FreezeUr10StyleAtCurrentPose",
                heldHandPosition,
                Quaternion.identity);
            SetUr10StyleLatestPose(teleop, heldHandPosition + new Vector3(0.02f, 0.0f, 0.0f), Quaternion.identity);
            InvokeNonPublic(teleop, "ApplyUr10StyleAnchoredPosePreview");

            AssertVectorNear(frozenTarget + new Vector3(0.01f, 0.0f, 0.0f), target.transform.position, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void Ur10StyleFreeze_WhenActualToolLags_FreezesPositionAtActualToolAndPreservesLockedRotation()
    {
        GameObject owner = new GameObject("ur10-style-freeze-command-rotation-test");
        GameObject target = new GameObject("TcpTarget");
        GameObject robot = new GameObject("ur10-style-freeze-command-rotation-robot");
        GameObject endEffector = new GameObject("tool0");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            teleop.anchoredPoseSmoothingStep = 1.0f;
            teleop.normalPositionScale = 0.50f;

            var follower = robot.AddComponent<Ur5TcpTargetFollower>();
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;
            SetNonPublicField(teleop, "tcpFollower", follower);

            Vector3 commandPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Vector3 actualPosition = commandPosition + new Vector3(0.03f, -0.02f, 0.01f);
            Quaternion commandRotation = Quaternion.Euler(25.0f, 35.0f, 45.0f);
            target.transform.SetPositionAndRotation(commandPosition, commandRotation);
            endEffector.transform.SetPositionAndRotation(
                actualPosition,
                Quaternion.identity);
            SetNonPublicField(teleop, "persistentOrientationTarget", commandRotation);
            SetNonPublicField(teleop, "hasPersistentOrientationTarget", true);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);
            InvokeNonPublic(
                teleop,
                "FreezeUr10StyleAtCurrentPose",
                Vector3.zero,
                Quaternion.identity);

            AssertVectorNear(actualPosition, target.transform.position, 0.000001f);
            Assert.LessOrEqual(
                Quaternion.Angle(commandRotation, target.transform.rotation),
                0.0001f,
                "Hand-stop re-anchoring must stop position chase without discarding the locked gripper attitude.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(target);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void Ur10StyleFollowerRelease_SnapsLedTcpTargetBackToActualPose()
    {
        GameObject owner = new GameObject("ur10-style-follower-release-test");
        GameObject robot = new GameObject("ur10-style-follower-release-robot");
        GameObject target = new GameObject("TcpTarget");
        GameObject endEffector = new GameObject("tool0");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            var follower = robot.AddComponent<Ur5TcpTargetFollower>();
            follower.velocityTeleop = teleop;
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;
            Vector3 ledTargetPosition = new Vector3(0.40f, 0.50f, 0.60f);
            Vector3 actualPosition = new Vector3(0.35f, 0.30f, 0.22f);
            Quaternion targetRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
            Quaternion actualRotation = Quaternion.Euler(-5.0f, 20.0f, 10.0f);
            target.transform.SetPositionAndRotation(ledTargetPosition, targetRotation);
            endEffector.transform.SetPositionAndRotation(actualPosition, actualRotation);

            InvokeNonPublic(follower, "BeginSafeRelease");

            AssertVectorNear(actualPosition, target.transform.position, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
            UnityEngine.Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void ControlBootstrap_QuestUdpShadowTelemetryKeepsLocalControlByDefault()
    {
        GameObject owner = new GameObject("udp-shadow-bootstrap-test");
        GameObject robot = new GameObject("udp-shadow-robot");
        GameObject target = new GameObject("TcpTarget");
        owner.SetActive(false);
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            bootstrap.robotRoot = robot.transform;
            bootstrap.tcpTarget = target.transform;
            bootstrap.enableQuestUdpShadowTelemetry = true;
            bootstrap.questShadowReceiverHost = "192.168.10.37";
            bootstrap.questShadowReceiverPort = 8080;
            bootstrap.enableCartesianVelocityTeleop = true;
            bootstrap.enableKeyboardControl = true;
            bootstrap.enableQuest3Control = true;
            bootstrap.enableTcpTargetFollower = true;
            bootstrap.addUrScriptSpeedlClient = true;

            InvokeNonPublic(bootstrap, "Awake");

            var sender = owner.GetComponent<Quest3UdpTeleopSender>();
            Assert.IsNotNull(sender);
            Assert.IsTrue(sender.sendPackets);
            Assert.AreEqual("192.168.10.37", sender.receiverHost);
            Assert.AreEqual(8080, sender.receiverPort);
            var stabilizer = owner.GetComponent<Ur5PhysicsStabilizer>();
            Assert.IsNotNull(stabilizer, "UDP bridge mode must still stabilize the Unity robot model.");
            Assert.AreSame(robot.transform, stabilizer.robotRoot);
            Assert.IsNotNull(owner.GetComponent<Ur5CartesianVelocityTeleopController>());
            Assert.IsNotNull(owner.GetComponent<Ur5UrScriptSpeedlClient>());
            Assert.IsNotNull(robot.GetComponent<Quest3RobotiqGripperController>());
            Assert.IsNotNull(robot.GetComponent<Ur5TcpTargetFollower>());
            Assert.IsNull(target.GetComponent<TcpTargetKeyboardController>());
            Assert.IsNull(target.GetComponent<Quest3TcpTargetController>());
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void QuestUdpTelemetryProtocol_IncludesRotationAdjustButton()
    {
        Type buttonsPayload = typeof(Quest3UdpTeleopSender).GetNestedType(
            "ButtonsPayload",
            BindingFlags.NonPublic);

        Assert.IsNotNull(buttonsPayload, "Quest UDP telemetry must keep an explicit buttons payload schema.");
        Assert.IsNotNull(
            buttonsPayload.GetField("rotation_adjust", BindingFlags.Instance | BindingFlags.Public),
            "PC-side 9.3 Task 8 requires A / primaryButton as an explicit Grip+A rotation-adjust signal, not recenter overloading.");
    }

    [Test]
    public void ControlBootstrap_ExplicitQuestUdpTelemetryOnlyDisablesLocalWriters()
    {
        GameObject owner = new GameObject("udp-only-bootstrap-test");
        GameObject robot = new GameObject("udp-only-robot");
        GameObject target = new GameObject("TcpTarget");
        owner.SetActive(false);
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            bootstrap.robotRoot = robot.transform;
            bootstrap.tcpTarget = target.transform;
            bootstrap.enableQuestUdpShadowTelemetry = true;
            bootstrap.disableLocalTeleopWhenQuestUdpShadowTelemetry = true;
            bootstrap.questShadowReceiverHost = "192.168.10.37";
            bootstrap.questShadowReceiverPort = 8080;
            bootstrap.enableCartesianVelocityTeleop = true;
            bootstrap.enableKeyboardControl = true;
            bootstrap.enableQuest3Control = true;
            bootstrap.enableTcpTargetFollower = true;
            bootstrap.addUrScriptSpeedlClient = true;

            InvokeNonPublic(bootstrap, "Awake");

            Assert.IsNotNull(owner.GetComponent<Quest3UdpTeleopSender>());
            Assert.IsNotNull(owner.GetComponent<Ur5PhysicsStabilizer>());
            Assert.IsNull(owner.GetComponent<Ur5CartesianVelocityTeleopController>());
            Assert.IsNull(owner.GetComponent<Ur5UrScriptSpeedlClient>());
            Assert.IsNull(robot.GetComponent<Quest3RobotiqGripperController>());
            Assert.IsNull(robot.GetComponent<Ur5TcpTargetFollower>());
            Assert.IsNull(target.GetComponent<TcpTargetKeyboardController>());
            Assert.IsNull(target.GetComponent<Quest3TcpTargetController>());
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [Test]
    public void LeftSafetyPoseOwnership_IsIncludedInCommandActivity()
    {
        GameObject owner = new GameObject("left-safety-owner-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            FieldInfo field = typeof(Ur5CartesianVelocityTeleopController).GetField(
                "leftSafetyPoseController",
                BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.IsNotNull(field, "主控制器必须持有独立的左手安全姿态状态机。");

            var controller = (Ur5LeftSafetyPoseController)field.GetValue(teleop);
            if (controller == null)
            {
                // executeMethod 的 EditMode 直接回归不会自动触发 Awake，显式初始化可保持两种测试入口一致。
                InvokeNonPublic(teleop, "Awake");
                controller = (Ur5LeftSafetyPoseController)field.GetValue(teleop);
            }

            Assert.IsNotNull(controller);
            controller.Step(new Ur5LeftSafetyPoseStepInput(
                rightGripHeld: false,
                leftPoseValid: true,
                leftGripHeld: true,
                primaryPressed: true,
                snapTargetReached: false,
                readyPoseActive: false,
                deltaTimeSeconds: 0.10f));

            Assert.IsTrue(teleop.IsSafetyPoseCommandActive);
            Assert.IsTrue(teleop.IsCommandActive);
            Assert.AreEqual(Ur5LeftSafetyPoseState.ButtonHeld, teleop.SafetyPoseState);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void LeftSafetySnapReach_RequiresBothPositionAndRotationTolerance()
    {
        Assert.IsTrue(Ur5CartesianVelocityTeleopController.IsSafetySnapTargetReached(
            positionErrorMeters: 0.0029f,
            rotationErrorDegrees: 0.49f,
            positionToleranceMeters: 0.003f,
            rotationToleranceDegrees: 0.50f));
        Assert.IsFalse(Ur5CartesianVelocityTeleopController.IsSafetySnapTargetReached(
            positionErrorMeters: 0.0031f,
            rotationErrorDegrees: 0.49f,
            positionToleranceMeters: 0.003f,
            rotationToleranceDegrees: 0.50f));
        Assert.IsFalse(Ur5CartesianVelocityTeleopController.IsSafetySnapTargetReached(
            positionErrorMeters: 0.0029f,
            rotationErrorDegrees: 0.51f,
            positionToleranceMeters: 0.003f,
            rotationToleranceDegrees: 0.50f));
    }

    [Test]
    public void LegacyContinuousConfig_RemainsAvailableButIsNotTheDefaultQuestPath()
    {
        GameObject owner = new GameObject("bootstrap-parameter-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);
            Ur5Continuous6DofConfig config = teleop.continuous6DofConfig;

            Assert.AreEqual(0.15f, config.TranslationNearGain, 0.000001f);
            Assert.AreEqual(0.80f, config.TranslationFarGain, 0.000001f);
            Assert.AreEqual(0.040f, config.TranslationTransitionMeters, 0.000001f);
            Assert.AreEqual(0.25f, config.RotationNearGain, 0.000001f);
            Assert.AreEqual(0.80f, config.RotationFarGain, 0.000001f);
            Assert.AreEqual(20.0f, config.RotationTransitionDegrees, 0.000001f);
            Assert.AreEqual(0.055f, config.PositionTimeConstantSeconds, 0.000001f);
            Assert.AreEqual(0.055f, config.RotationTimeConstantSeconds, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void DiagnosticsLog_ReportsContinuousFieldsAndLegacyModeLabel()
    {
        GameObject owner = new GameObject("diagnostics-test");
        string capturedLog = string.Empty;
        Application.LogCallback callback = (condition, stackTrace, type) => capturedLog = condition;

        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableContinuous6DofClutch = true;
            var diagnostics = owner.AddComponent<Ur5TeleopDiagnostics>();
            diagnostics.enableDiagnostics = true;
            diagnostics.teleop = teleop;

            Application.logMessageReceived += callback;
            InvokeNonPublic(diagnostics, "Update");
        }
        finally
        {
            Application.logMessageReceived -= callback;
            UnityEngine.Object.DestroyImmediate(owner);
        }

        Assert.That(capturedLog, Does.Contain("control=continuous6dof"));
        Assert.That(capturedLog, Does.Contain("mode=legacy-free"));
        Assert.That(capturedLog, Does.Contain("fault="));
        Assert.That(capturedLog, Does.Contain("handDistanceM="));
        Assert.That(capturedLog, Does.Contain("handAngleDeg="));
        Assert.That(capturedLog, Does.Contain("translationGain="));
        Assert.That(capturedLog, Does.Contain("rotationGain="));
        Assert.That(capturedLog, Does.Contain("rotationFilterGapDeg="));
        Assert.That(capturedLog, Does.Contain("rawHandRotation="));
        Assert.That(capturedLog, Does.Contain("stationarySeconds="));
        Assert.That(capturedLog, Does.Contain("settledHold="));
        Assert.That(capturedLog, Does.Contain("nearSingularity="));
    }

    [Test]
    public void FollowerRuntimeDiagnostics_AreReadOnlyProperties()
    {
        PropertyInfo stationarySeconds = typeof(Ur5TcpTargetFollower).GetProperty(
            "TargetStationarySeconds",
            BindingFlags.Instance | BindingFlags.Public);
        PropertyInfo settledHold = typeof(Ur5TcpTargetFollower).GetProperty(
            "IsSettledTargetHoldActive",
            BindingFlags.Instance | BindingFlags.Public);

        Assert.IsNotNull(stationarySeconds);
        Assert.IsNotNull(settledHold);
        Assert.IsTrue(stationarySeconds.CanRead);
        Assert.IsFalse(stationarySeconds.CanWrite);
        Assert.IsTrue(settledHold.CanRead);
        Assert.IsFalse(settledHold.CanWrite);
    }

    [Test]
    public void PoseCsvRecorder_AppendsContinuousColumnsAtTail()
    {
        GameObject owner = new GameObject("csv-recorder-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableContinuous6DofClutch = true;
            var recorder = owner.AddComponent<Ur5PoseCsvRecorder>();
            recorder.velocityTeleop = teleop;

            InvokeNonPublic(recorder, "Start");
            InvokeNonPublic(recorder, "AppendSample");

            string csv = GetRecorderCsv(recorder).ToString();
            string[] lines = csv.Trim().Split('\n');
            string expectedTail =
                "continuous_6dof_enabled,continuous_state,continuous_fault,"
                + "controller_distance_m,controller_angle_deg,translation_gain,"
                + "rotation_gain,logical_to_filtered_rotation_deg,"
                + "raw_hand_pos_x,raw_hand_pos_y,raw_hand_pos_z,"
                + "raw_hand_rot_x,raw_hand_rot_y,raw_hand_rot_z,raw_hand_rot_w,"
                + "logical_pos_x,logical_pos_y,logical_pos_z,"
                + "logical_rot_x,logical_rot_y,logical_rot_z,logical_rot_w,"
                + "constrained_pos_x,constrained_pos_y,constrained_pos_z,"
                + "filtered_pos_x,filtered_pos_y,filtered_pos_z,"
                + "filtered_rot_x,filtered_rot_y,filtered_rot_z,filtered_rot_w,"
                + "actual_grasp_rot_x,actual_grasp_rot_y,actual_grasp_rot_z,actual_grasp_rot_w,"
                + "target_stationary_s,settled_hold,dls_min_pivot,near_singularity,"
                + "ik_failure_count,ik_lead_limited,"
                + "j1_drive_deg,j2_drive_deg,j3_drive_deg,j4_drive_deg,j5_drive_deg,j6_drive_deg,"
                + "j1_measured_deg,j2_measured_deg,j3_measured_deg,"
                + "j4_measured_deg,j5_measured_deg,j6_measured_deg,"
                + "ur10_rotation_adjust_active,position_orientation_locked,"
                + "controller_position_gate_holding,preview_lead_limited,"
                + "active_preview_lead_limit_m,moving_preview_lead_active,"
                + "direct_joint_state_servo_enabled,direct_joint_state_servo_active,"
                + "measured_teleop_max_joint_step_deg";

            Assert.That(lines[0], Does.EndWith(expectedTail));
            Assert.AreEqual(
                lines[0].Split(',').Length,
                lines[1].Split(',').Length,
                "CSV 头和采样行必须保持同样列数，避免后续分析脚本错位。");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void PoseCsvRecorder_AutoStartsOnlyOnAndroid()
    {
        Assert.IsTrue(Ur5PoseCsvRecorder.ShouldAutoStartRecording(RuntimePlatform.Android));
        Assert.IsFalse(Ur5PoseCsvRecorder.ShouldAutoStartRecording(RuntimePlatform.OSXEditor));
        Assert.IsFalse(Ur5PoseCsvRecorder.ShouldAutoStartRecording(RuntimePlatform.WindowsEditor));
    }

    [Test]
    public void ControlBootstrap_EnforcesQuestRecorderDefaults()
    {
        GameObject owner = new GameObject("recorder-bootstrap-test");
        owner.SetActive(false);
        try
        {
            var bootstrap = owner.AddComponent<Ur5ControlBootstrap>();
            var recorder = owner.AddComponent<Ur5PoseCsvRecorder>();
            recorder.autoRecordOnAndroid = false;
            recorder.flushIntervalSeconds = 7.0f;
            recorder.sampleInterval = 0.50f;

            InvokeNonPublic(bootstrap, "ConfigureRecorder", null, null, null, null);

            Assert.IsTrue(recorder.autoRecordOnAndroid);
            Assert.AreEqual(1.0f, recorder.flushIntervalSeconds, 0.000001f);
            Assert.AreEqual(0.02f, recorder.sampleInterval, 0.000001f);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void PoseCsvRecorder_PauseAndRepeatedSaveKeepOneCompleteSnapshot()
    {
        string outputPath = Path.Combine(
            Path.GetTempPath(),
            "ur5-task7-recorder-" + Guid.NewGuid().ToString("N") + ".csv");
        GameObject owner = new GameObject("csv-persistence-test");
        try
        {
            var recorder = owner.AddComponent<Ur5PoseCsvRecorder>();
            InvokeNonPublic(recorder, "Start");
            recorder.StartRecording();
            InvokeNonPublic(recorder, "AppendSample");
            SetNonPublicField(recorder, "outputPath", outputPath);

            InvokeNonPublic(recorder, "OnApplicationPause", true);
            string firstSnapshot = File.ReadAllText(outputPath);
            recorder.SaveRecordingSnapshot();
            string secondSnapshot = File.ReadAllText(outputPath);

            Assert.AreEqual(firstSnapshot, secondSnapshot);
            string header = GetRecorderCsv(recorder).ToString().Split('\n')[0];
            Assert.AreEqual(
                1,
                CountOccurrences(secondSnapshot, header),
                "重复生命周期保存不能追加第二个表头。");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
            if (File.Exists(outputPath))
            {
                File.Delete(outputPath);
            }
        }
    }

    [Test]
    public void PoseCsvRecorder_RepeatedSaveFailureLogsOnlyOnce()
    {
        int saveFailureLogCount = 0;
        GameObject owner = new GameObject("csv-save-failure-test");
        Application.LogCallback callback = (condition, stackTrace, type) =>
        {
            if (type == LogType.Error && condition.Contains("UR5 recording save failed"))
            {
                saveFailureLogCount++;
            }
        };

        try
        {
            var recorder = owner.AddComponent<Ur5PoseCsvRecorder>();
            InvokeNonPublic(recorder, "Start");
            SetNonPublicField(recorder, "outputPath", Path.GetTempPath());

            Application.logMessageReceived += callback;
            LogAssert.Expect(LogType.Error, new Regex("^UR5 recording save failed:"));
            recorder.SaveRecordingSnapshot();
            recorder.SaveRecordingSnapshot();

            Assert.AreEqual(1, saveFailureLogCount);
            SetNonPublicField(recorder, "outputPath", string.Empty);
        }
        finally
        {
            Application.logMessageReceived -= callback;
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    private static void InvokeNonPublic(object target, string methodName)
    {
        MethodInfo method = target.GetType().GetMethod(
            methodName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(method, methodName + " should exist.");
        method.Invoke(target, null);
    }

    private static void InvokeNonPublic(object target, string methodName, params object[] arguments)
    {
        MethodInfo method = target.GetType().GetMethod(
            methodName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(method, methodName + " should exist.");
        method.Invoke(target, arguments);
    }

    private static void SetNonPublicField(object target, string fieldName, object value)
    {
        FieldInfo field = target.GetType().GetField(
            fieldName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, fieldName + " should exist.");
        field.SetValue(target, value);
    }

    private static object GetPublicFieldValue(object target, string fieldName)
    {
        FieldInfo field = target.GetType().GetField(
            fieldName,
            BindingFlags.Instance | BindingFlags.Public);
        Assert.IsNotNull(field, fieldName + " should exist.");
        return field.GetValue(target);
    }

    private static void SetUr10StyleLatestPose(
        Ur5CartesianVelocityTeleopController teleop,
        Vector3 position,
        Quaternion rotation)
    {
        SetNonPublicField(teleop, "latestRawPositionWorld", position);
        SetNonPublicField(teleop, "latestPositionWorld", position);
        SetNonPublicField(teleop, "latestRotationWorld", rotation);
        SetNonPublicField(teleop, "latestRawPositionValid", true);
        SetNonPublicField(teleop, "latestPositionValid", true);
        SetNonPublicField(teleop, "latestRotationValid", true);
        InvokeNonPublic(
            teleop,
            "UpdateUr10StyleAnchoredPoseInput",
            true,
            position,
            true,
            rotation);
    }

    private static void AssertVectorNear(Vector3 expected, Vector3 actual, float tolerance)
    {
        Assert.LessOrEqual(Vector3.Distance(expected, actual), tolerance);
    }

    private static int CountOccurrences(string value, string search)
    {
        int count = 0;
        int index = 0;
        while ((index = value.IndexOf(search, index, StringComparison.Ordinal)) >= 0)
        {
            count++;
            index += search.Length;
        }

        return count;
    }

    private static StringBuilder GetRecorderCsv(Ur5PoseCsvRecorder recorder)
    {
        FieldInfo field = typeof(Ur5PoseCsvRecorder).GetField(
            "csv",
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, "csv field should exist.");
        return (StringBuilder)field.GetValue(recorder);
    }
}

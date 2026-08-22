using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public class Ur5JointCommandAntiWindupTests
{
    private GameObject rootOwner;
    private GameObject controllerOwner;
    private GameObject jointOwner;
    private Ur5ArticulationJointController controller;
    private ArticulationBody joint;

    [SetUp]
    public void SetUp()
    {
        rootOwner = new GameObject("anti-windup-root");
        rootOwner.AddComponent<ArticulationBody>();

        controllerOwner = new GameObject("anti-windup-controller");
        controllerOwner.SetActive(false);
        jointOwner = new GameObject("anti-windup-joint");
        jointOwner.transform.SetParent(rootOwner.transform, false);
        joint = jointOwner.AddComponent<ArticulationBody>();
        joint.jointType = ArticulationJointType.RevoluteJoint;
        joint.jointPosition = new ArticulationReducedSpace(0.0f);
        controller = controllerOwner.AddComponent<Ur5ArticulationJointController>();
        controller.limitDriveTargetLeadFromMeasuredJoint = false;

        GetPrivateList<ArticulationBody>(controller, "joints").Add(joint);
        GetPrivateList<float>(controller, "jointTargets").Add(120.0f);
        GetPrivateList<float>(controller, "appliedJointTargets").Add(120.0f);
        GetPrivateList<float>(controller, "appliedJointVelocities").Add(0.0f);
    }

    [TearDown]
    public void TearDown()
    {
        Object.DestroyImmediate(controllerOwner);
        Object.DestroyImmediate(rootOwner);
    }

    [Test]
    public void ApplyDriveTarget_WhenMeasuredRevoluteJointIsAtZero_StoresConstrainedTarget()
    {
        Assert.Greater(joint.jointPosition.dofCount, 0);
        Assert.AreEqual(0.0f, joint.jointPosition[0] * Mathf.Rad2Deg, 0.0001f);

        controller.limitDriveTargetLeadFromMeasuredJoint = true;
        controller.maximumDriveTargetLeadDegrees = 6.0f;
        InvokeNonPublic(controller, "ApplyDriveTarget", 0, 120.0f);

        Assert.AreEqual(6.0f, controller.GetDriveTargetDegrees(0), 0.0001f);
        Assert.AreEqual(6.0f, controller.GetAppliedJointTargetDegrees(0), 0.0001f);
    }

    [Test]
    public void ApplyDriveTarget_WhenAppliedStateDiverged_SynchronizesToAssignedDriveTarget()
    {
        InvokeNonPublic(controller, "ApplyDriveTarget", 0, 6.0f);

        Assert.AreEqual(6.0f, controller.GetDriveTargetDegrees(0), 0.0001f);
        Assert.AreEqual(
            controller.GetDriveTargetDegrees(0),
            controller.GetAppliedJointTargetDegrees(0),
            0.0001f,
            "The applied cache must not remain at an unapplied 120-degree request.");
    }

    [Test]
    public void BeginJointWaypoint_WhenTrajectoryStateDiverged_SeedsFromDriveTarget()
    {
        ArticulationDrive drive = joint.xDrive;
        drive.target = 6.0f;
        joint.xDrive = drive;
        GetPrivateList<float>(controller, "appliedJointTargets")[0] = 120.0f;

        var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
        follower.jointController = controller;
        InvokeNonPublic(follower, "BeginJointWaypoint", 1);

        float[] waypoint = GetPrivateField<float[]>(
            follower,
            "workingJointTargetsDegrees");
        Assert.AreEqual(6.0f, waypoint[0], 0.0001f);
    }

    [Test]
    public void BeginJointWaypoint_WhenMeasuredStateTeleopIsActive_SeedsFromMeasuredJoint()
    {
        ArticulationDrive drive = joint.xDrive;
        drive.target = 6.0f;
        joint.xDrive = drive;

        var teleop = controllerOwner.AddComponent<Ur5CartesianVelocityTeleopController>();
        SetNonPublicField(teleop, "<IsPositionClutched>k__BackingField", true);

        var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
        follower.jointController = controller;
        follower.velocityTeleop = teleop;
        follower.useMeasuredStateTeleopSolve = true;

        InvokeNonPublic(follower, "BeginJointWaypoint", 1);

        float[] waypoint = GetPrivateField<float[]>(follower, "workingJointTargetsDegrees");
        Assert.AreEqual(
            0.0f,
            waypoint[0],
            0.0001f,
            "Active teleoperation must solve from measured joint state, not a stale drive target that can create a lead/lag kick.");
    }

    [Test]
    public void LimitMeasuredStateJointStep_ClampsEachRequestWithoutHistoricalDelta()
    {
        MethodInfo method = typeof(Ur5TcpTargetFollower).GetMethod(
            "LimitMeasuredStateJointStep",
            BindingFlags.Static | BindingFlags.NonPublic);
        Assert.IsNotNull(method, "Measured-state teleop must expose a deterministic per-step limiter.");

        float first = (float)method.Invoke(null, new object[] { 8.0f, 2.0f });
        float second = (float)method.Invoke(null, new object[] { 8.0f, 2.0f });
        float reverse = (float)method.Invoke(null, new object[] { -8.0f, 2.0f });

        Assert.AreEqual(2.0f, first, 0.0001f);
        Assert.AreEqual(2.0f, second, 0.0001f);
        Assert.AreEqual(-2.0f, reverse, 0.0001f);
        Assert.GreaterOrEqual(first * second, 0.0f);
        Assert.LessOrEqual(Mathf.Abs(reverse), 2.0f);
    }

    [Test]
    public void CommitJointWaypoint_WhenMeasuredStateTeleopIsActive_BypassesStaleTrajectoryQueue()
    {
        var target = new GameObject("anti-windup-direct-target");
        var endEffector = new GameObject("anti-windup-direct-tool0");
        try
        {
            var teleop = controllerOwner.AddComponent<Ur5CartesianVelocityTeleopController>();
            SetNonPublicField(teleop, "<IsPositionClutched>k__BackingField", true);

            var trajectoryPlayer = controllerOwner.AddComponent<Ur5JointTrajectoryPlayer>();
            trajectoryPlayer.jointController = controller;
            trajectoryPlayer.EnqueueWaypointDegrees(new[] { 40.0f }, 1);

            var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
            follower.jointController = controller;
            follower.velocityTeleop = teleop;
            follower.trajectoryPlayer = trajectoryPlayer;
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;

            InvokeNonPublic(follower, "BeginJointWaypoint", 1);
            InvokeNonPublic(follower, "QueueJointDelta", 0, 1.0f);
            InvokeNonPublic(follower, "CommitJointWaypoint");

            Assert.IsFalse(
                trajectoryPlayer.HasPendingWaypoint,
                "Active teleoperation must not leave a stale trajectory waypoint to be applied after the current measured-state command.");
            Assert.AreEqual(
                1.0f,
                controller.GetAppliedJointTargetDegrees(0),
                0.0001f,
                "Measured-state teleoperation must commit the current one-step target directly to the drive cache.");
        }
        finally
        {
            Object.DestroyImmediate(target);
            Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void Ur10StyleGripStart_WhenOldWaypointExists_HoldsMeasuredJointPoseAndClearsQueue()
    {
        var target = new GameObject("anti-windup-tcp-target");
        var endEffector = new GameObject("anti-windup-tool0");
        try
        {
            var trajectoryPlayer = controllerOwner.AddComponent<Ur5JointTrajectoryPlayer>();
            trajectoryPlayer.jointController = controller;
            trajectoryPlayer.EnqueueWaypointDegrees(new[] { 45.0f }, 1);

            var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
            follower.jointController = controller;
            follower.trajectoryPlayer = trajectoryPlayer;
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;

            var teleop = controllerOwner.AddComponent<Ur5CartesianVelocityTeleopController>();
            teleop.enableUr10StyleAnchoredPoseClutch = true;
            teleop.enableThreeModeController = false;
            teleop.rotationInputMode = Ur5CartesianVelocityTeleopController.RotationInputMode.Locked;
            teleop.usePositionGripAsDeadman = false;
            teleop.tcpPreviewTarget = target.transform;
            SetNonPublicField(teleop, "tcpFollower", follower);

            target.transform.SetPositionAndRotation(
                new Vector3(0.40f, 0.50f, 0.60f),
                Quaternion.identity);
            endEffector.transform.SetPositionAndRotation(
                new Vector3(0.30f, 0.20f, 0.10f),
                Quaternion.identity);

            InvokeNonPublic(
                teleop,
                "UpdateUr10StyleAnchoredPoseInput",
                true,
                Vector3.zero,
                true,
                Quaternion.identity);

            Assert.AreEqual(0.0f, controller.GetJointTargetDegrees(0), 0.0001f);
            Assert.AreEqual(0.0f, controller.GetAppliedJointTargetDegrees(0), 0.0001f);
            Assert.IsFalse(
                trajectoryPlayer.HasPendingWaypoint,
                "Grip press must discard stale IK waypoints before the first held frame can chase them.");
        }
        finally
        {
            Object.DestroyImmediate(target);
            Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void StepTowardTarget_WhenTranslationLocksOrientation_UsesSingleDlsTask()
    {
        var target = new GameObject("anti-windup-dls-target");
        var endEffector = new GameObject("anti-windup-dls-tool0");
        try
        {
            var teleop = controllerOwner.AddComponent<Ur5CartesianVelocityTeleopController>();
            SetNonPublicField(teleop, "<IsPositionClutched>k__BackingField", true);
            SetNonPublicField(teleop, "hasPositionOrientationLock", true);

            var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
            follower.jointController = controller;
            follower.velocityTeleop = teleop;
            follower.tcpTarget = target.transform;
            follower.endEffector = endEffector.transform;
            follower.usePhysicalGraspFrameForOrientation = false;
            follower.followTargetRotation = true;

            target.transform.SetPositionAndRotation(
                new Vector3(0.20f, 0.10f, 0.00f),
                Quaternion.Euler(0.0f, 15.0f, 0.0f));
            endEffector.transform.SetPositionAndRotation(
                Vector3.zero,
                Quaternion.identity);

            PropertyInfo property = typeof(Ur5TcpTargetFollower).GetProperty("LastIkTaskMode");
            Assert.IsNotNull(
                property,
                "Follower diagnostics must expose which IK task path handled the current frame.");

            InvokeNonPublic(follower, "StepTowardTarget");

            Assert.AreEqual(
                "LockedTranslationSingleDls",
                property.GetValue(follower).ToString(),
                "Grip-only translation with a locked gripper attitude must solve position and orientation in one DLS task, not by alternating CCD position and wrist-only correction.");
        }
        finally
        {
            Object.DestroyImmediate(target);
            Object.DestroyImmediate(endEffector);
        }
    }

    [Test]
    public void CommitJointWaypoint_WhenWaypointExceedsCommandLead_ClampsLogicalTargetToDriveTarget()
    {
        ArticulationDrive drive = joint.xDrive;
        drive.target = 6.0f;
        joint.xDrive = drive;
        GetPrivateList<float>(controller, "appliedJointTargets")[0] = 6.0f;
        GetPrivateList<float>(controller, "jointTargets")[0] = 120.0f;

        var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
        follower.jointController = controller;
        follower.maximumCommandLeadDegrees = 2.0f;
        InvokeNonPublic(follower, "BeginJointWaypoint", 1);
        InvokeNonPublic(follower, "QueueJointDelta", 0, 120.0f);
        InvokeNonPublic(follower, "CommitJointWaypoint");

        Assert.AreEqual(8.0f, controller.GetJointTargetDegrees(0), 0.0001f);
        Assert.AreEqual(6.0f, controller.GetDriveTargetDegrees(0), 0.0001f);
        Assert.IsTrue(follower.WasIkCommandLeadLimited);
    }

    [Test]
    public void CommitJointWaypoint_WhenWristWaypointExceedsCommandLead_UsesWristLeadWindow()
    {
        var joints = GetPrivateList<ArticulationBody>(controller, "joints");
        var targets = GetPrivateList<float>(controller, "jointTargets");
        var appliedTargets = GetPrivateList<float>(controller, "appliedJointTargets");
        var appliedVelocities = GetPrivateList<float>(controller, "appliedJointVelocities");
        joints.Clear();
        targets.Clear();
        appliedTargets.Clear();
        appliedVelocities.Clear();
        for (int index = 0; index < 6; index++)
        {
            GameObject owner = new GameObject("anti-windup-joint-" + index);
            owner.transform.SetParent(rootOwner.transform, false);
            ArticulationBody body = owner.AddComponent<ArticulationBody>();
            body.jointType = ArticulationJointType.RevoluteJoint;
            joints.Add(body);
            targets.Add(0.0f);
            appliedTargets.Add(0.0f);
            appliedVelocities.Add(0.0f);
        }

        var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
        follower.jointController = controller;
        follower.maximumCommandLeadDegrees = 4.0f;
        follower.maximumWristCommandLeadDegrees = 12.0f;
        InvokeNonPublic(follower, "BeginJointWaypoint", 6);
        InvokeNonPublic(follower, "QueueJointDelta", 0, 120.0f);
        InvokeNonPublic(follower, "QueueJointDelta", 5, 120.0f);
        InvokeNonPublic(follower, "CommitJointWaypoint");

        Assert.AreEqual(4.0f, controller.GetJointTargetDegrees(0), 0.0001f);
        Assert.AreEqual(12.0f, controller.GetJointTargetDegrees(5), 0.0001f);
        Assert.IsTrue(follower.WasIkCommandLeadLimited);
    }

    private static List<T> GetPrivateList<T>(object target, string fieldName)
    {
        FieldInfo field = target.GetType().GetField(
            fieldName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, fieldName + " should exist.");
        return (List<T>)field.GetValue(target);
    }

    private static T GetPrivateField<T>(object target, string fieldName)
    {
        FieldInfo field = target.GetType().GetField(
            fieldName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, fieldName + " should exist.");
        return (T)field.GetValue(target);
    }

    private static void SetNonPublicField(object target, string fieldName, object value)
    {
        FieldInfo field = target.GetType().GetField(
            fieldName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, fieldName + " should exist.");
        field.SetValue(target, value);
    }

    private static void InvokeNonPublic(object target, string methodName, params object[] arguments)
    {
        MethodInfo method = target.GetType().GetMethod(
            methodName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(method, methodName + " should exist.");
        method.Invoke(target, arguments);
    }
}

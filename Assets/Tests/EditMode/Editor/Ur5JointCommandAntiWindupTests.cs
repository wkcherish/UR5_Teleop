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

    private static void InvokeNonPublic(object target, string methodName, params object[] arguments)
    {
        MethodInfo method = target.GetType().GetMethod(
            methodName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(method, methodName + " should exist.");
        method.Invoke(target, arguments);
    }
}

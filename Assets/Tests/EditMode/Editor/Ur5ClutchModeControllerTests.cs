using NUnit.Framework;
using UnityEngine;

public class Ur5ClutchModeControllerTests
{
    [Test]
    public void GripPressCapturesControllerAndTcpBaseline_ReleaseStopsUpdates()
    {
        var controller = CreateController();
        var baseline = CreateInput(gripHeld: true, Ur5TeleopMode.Free, Vector3.zero, Quaternion.identity);

        Ur5TeleopStepResult atBaseline = controller.Step(baseline);

        Assert.AreEqual(Ur5TeleopControllerState.Clutched, atBaseline.State);
        Assert.IsTrue(atBaseline.IsMotionCommandActive);
        AssertVectorNear(baseline.ActualTcpPosition, atBaseline.TargetPosition, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(baseline.ActualTcpRotation, atBaseline.TargetRotation), 0.0001f);

        Ur5TeleopStepResult released = controller.Step(CreateInput(
            gripHeld: false,
            Ur5TeleopMode.Free,
            new Vector3(0.20f, 0.0f, 0.0f),
            Quaternion.AngleAxis(30.0f, Vector3.up)));

        Assert.AreEqual(Ur5TeleopControllerState.Idle, released.State);
        Assert.IsFalse(released.IsMotionCommandActive);
        AssertVectorNear(Vector3.zero, released.LinearDelta, 0.000001f);
        Assert.LessOrEqual(released.AngularDeltaDegrees, 0.0001f);
    }

    [Test]
    public void FreeFineAndInsertUseIndependentModeParameters()
    {
        var freeController = CreateController();
        freeController.Step(CreateInput(gripHeld: true, Ur5TeleopMode.Free, Vector3.zero, Quaternion.identity));
        var fineController = CreateController();
        fineController.Step(CreateInput(gripHeld: true, Ur5TeleopMode.Fine, Vector3.zero, Quaternion.identity));

        Ur5TeleopStepResult free = freeController.Step(CreateInput(
            gripHeld: true,
            Ur5TeleopMode.Free,
            new Vector3(0.10f, 0.0f, 0.0f),
            Quaternion.AngleAxis(20.0f, Vector3.up)));
        Ur5TeleopStepResult fine = fineController.Step(CreateInput(
            gripHeld: true,
            Ur5TeleopMode.Fine,
            new Vector3(0.10f, 0.0f, 0.0f),
            Quaternion.AngleAxis(20.0f, Vector3.up)));

        Assert.AreEqual(0.080f, free.LinearDelta.x, 0.000001f);
        Assert.AreEqual(0.55f, free.Config.CommandFilterRetention, 0.000001f);
        Assert.AreEqual(0.020f, fine.LinearDelta.x, 0.000001f);
        Assert.AreEqual(0.70f, fine.Config.CommandFilterRetention, 0.000001f);
        Assert.Less(fine.AngularDeltaDegrees, free.AngularDeltaDegrees);
    }

    [Test]
    public void InsertModeConstrainsTranslationToConfiguredAxis()
    {
        var controller = CreateController();
        controller.Step(CreateInput(gripHeld: true, Ur5TeleopMode.Insert, Vector3.zero, Quaternion.identity));

        Ur5TeleopStepResult result = controller.Step(CreateInput(
            gripHeld: true,
            Ur5TeleopMode.Insert,
            new Vector3(0.04f, 0.03f, 0.12f),
            Quaternion.identity));

        Assert.AreEqual(Ur5TeleopMode.Insert, result.ActiveMode);
        AssertVectorNear(new Vector3(0.0f, 0.0f, 0.06f), result.LinearDelta, 0.000001f);
        AssertVectorNear(new Vector3(0.40f, 0.50f, 0.66f), result.TargetPosition, 0.000001f);
    }

    [Test]
    public void ModeSwitchRebasesFromCurrentTargetWithoutJump()
    {
        var controller = CreateController();
        controller.Step(CreateInput(gripHeld: true, Ur5TeleopMode.Free, Vector3.zero, Quaternion.identity));
        Ur5TeleopStepResult beforeSwitch = controller.Step(CreateInput(
            gripHeld: true,
            Ur5TeleopMode.Free,
            new Vector3(0.10f, 0.0f, 0.0f),
            Quaternion.AngleAxis(20.0f, Vector3.up)));

        Ur5TeleopStepResult switched = controller.Step(CreateInput(
            gripHeld: true,
            Ur5TeleopMode.Fine,
            new Vector3(0.10f, 0.0f, 0.0f),
            Quaternion.AngleAxis(20.0f, Vector3.up)));

        Assert.AreEqual(Ur5TeleopControllerState.Clutched, switched.State);
        AssertVectorNear(beforeSwitch.TargetPosition, switched.TargetPosition, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(beforeSwitch.TargetRotation, switched.TargetRotation), 0.0001f);
        Assert.IsTrue(switched.WasRebasedThisStep);
    }

    [Test]
    public void InvalidPoseOrRejectedSafetyEntersFaultAndStopsMotion()
    {
        var controller = CreateController();

        Ur5TeleopStepResult invalidPose = controller.Step(CreateInput(
            gripHeld: true,
            Ur5TeleopMode.Free,
            new Vector3(float.NaN, 0.0f, 0.0f),
            Quaternion.identity));

        Assert.AreEqual(Ur5TeleopControllerState.Fault, invalidPose.State);
        Assert.IsFalse(invalidPose.IsMotionCommandActive);
        AssertVectorNear(Vector3.zero, invalidPose.LinearDelta, 0.000001f);

        Ur5TeleopStepInput rejectedSafety = CreateInput(
            gripHeld: true,
            Ur5TeleopMode.Free,
            Vector3.zero,
            Quaternion.identity);
        rejectedSafety.IsSafetyAccepted = false;

        Ur5TeleopStepResult safetyFault = controller.Step(rejectedSafety);

        Assert.AreEqual(Ur5TeleopControllerState.Fault, safetyFault.State);
        Assert.IsFalse(safetyFault.IsMotionCommandActive);
        AssertVectorNear(Vector3.zero, safetyFault.LinearDelta, 0.000001f);
    }

    private static Ur5ClutchModeController CreateController()
    {
        return new Ur5ClutchModeController(
            new Ur5TeleopModeConfig(
                version: 1,
                translationGain: 0.80f,
                rotationGain: 0.80f,
                poseSmoothingStep: 0.18f,
                commandFilterRetention: 0.55f,
                deadbandMeters: 0.0f,
                maxLinearDeltaMeters: 0.50f,
                maxAngularDeltaDegrees: 45.0f,
                insertAxis: Vector3.forward,
                constrainToInsertAxis: false),
            new Ur5TeleopModeConfig(
                version: 1,
                translationGain: 0.20f,
                rotationGain: 0.25f,
                poseSmoothingStep: 0.10f,
                commandFilterRetention: 0.70f,
                deadbandMeters: 0.0f,
                maxLinearDeltaMeters: 0.10f,
                maxAngularDeltaDegrees: 12.0f,
                insertAxis: Vector3.forward,
                constrainToInsertAxis: false),
            new Ur5TeleopModeConfig(
                version: 1,
                translationGain: 0.50f,
                rotationGain: 0.0f,
                poseSmoothingStep: 0.08f,
                commandFilterRetention: 0.80f,
                deadbandMeters: 0.0f,
                maxLinearDeltaMeters: 0.07f,
                maxAngularDeltaDegrees: 0.0f,
                insertAxis: Vector3.forward,
                constrainToInsertAxis: true));
    }

    private static Ur5TeleopStepInput CreateInput(
        bool gripHeld,
        Ur5TeleopMode mode,
        Vector3 controllerPosition,
        Quaternion controllerRotation)
    {
        return new Ur5TeleopStepInput(
            gripHeld: gripHeld,
            requestedMode: mode,
            controllerPosition: controllerPosition,
            controllerRotation: controllerRotation,
            actualTcpPosition: new Vector3(0.40f, 0.50f, 0.60f),
            actualTcpRotation: Quaternion.Euler(0.0f, 10.0f, 0.0f),
            isInputPoseValid: true,
            isRobotStateValid: true,
            isSafetyAccepted: true);
    }

    private static void AssertVectorNear(Vector3 expected, Vector3 actual, float tolerance)
    {
        Assert.LessOrEqual(Vector3.Distance(expected, actual), tolerance);
    }
}

using NUnit.Framework;
using UnityEngine;

public class Ur5Continuous6DofClutchControllerTests
{
    [Test]
    public void ContinuousController_DefaultsToSingleRightHandOwnership()
    {
        GameObject owner = new GameObject("teleop-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();

            Assert.IsTrue(teleop.enableContinuous6DofClutch);
            Assert.AreEqual(UnityEngine.XR.XRNode.RightHand, teleop.positionControllerNode);
            Assert.AreEqual(UnityEngine.XR.XRNode.RightHand, teleop.rotationControllerNode);
            Assert.IsTrue(teleop.usePositionGripAsDeadman);
            Assert.IsTrue(teleop.useRotationGripAsDeadman);
        }
        finally
        {
            Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void DefaultConfig_UsesApprovedSingleModeParameters()
    {
        Ur5Continuous6DofConfig config = Ur5Continuous6DofConfig.Default;

        Assert.AreEqual(1, config.Version);
        Assert.AreEqual(0.15f, config.TranslationNearGain, 0.000001f);
        Assert.AreEqual(0.80f, config.TranslationFarGain, 0.000001f);
        Assert.AreEqual(0.040f, config.TranslationTransitionMeters, 0.000001f);
        Assert.AreEqual(0.25f, config.RotationNearGain, 0.000001f);
        Assert.AreEqual(0.80f, config.RotationFarGain, 0.000001f);
        Assert.AreEqual(20.0f, config.RotationTransitionDegrees, 0.000001f);
        Assert.AreEqual(0.055f, config.PositionTimeConstantSeconds, 0.000001f);
        Assert.AreEqual(0.055f, config.RotationTimeConstantSeconds, 0.000001f);
    }

    [Test]
    public void ContinuousGains_AreMonotonicAndMatchBothEndpoints()
    {
        Ur5Continuous6DofConfig config = Ur5Continuous6DofConfig.Default;

        Assert.AreEqual(0.15f, config.EvaluateTranslationGain(0.0f), 0.000001f);
        Assert.AreEqual(0.80f, config.EvaluateTranslationGain(0.040f), 0.000001f);
        Assert.Less(
            config.EvaluateTranslationGain(0.010f),
            config.EvaluateTranslationGain(0.020f));
        Assert.Less(
            config.EvaluateTranslationGain(0.020f),
            config.EvaluateTranslationGain(0.030f));

        Assert.AreEqual(0.25f, config.EvaluateRotationGain(0.0f), 0.000001f);
        Assert.AreEqual(0.80f, config.EvaluateRotationGain(20.0f), 0.000001f);
        Assert.Less(
            config.EvaluateRotationGain(5.0f),
            config.EvaluateRotationGain(10.0f));
        Assert.Less(
            config.EvaluateRotationGain(10.0f),
            config.EvaluateRotationGain(15.0f));
    }

    [Test]
    public void SanitizedConfig_RejectsInvalidOrInvertedRanges()
    {
        Ur5Continuous6DofConfig config = Ur5Continuous6DofConfig.Default;
        config.TranslationNearGain = -1.0f;
        config.TranslationFarGain = 0.10f;
        config.RotationNearGain = 0.70f;
        config.RotationFarGain = 0.20f;
        config.TranslationTransitionMeters = float.NaN;
        config.PositionTimeConstantSeconds = -0.5f;

        Ur5Continuous6DofConfig sanitized = config.Sanitized();

        Assert.GreaterOrEqual(sanitized.TranslationNearGain, 0.0f);
        Assert.GreaterOrEqual(sanitized.TranslationFarGain, sanitized.TranslationNearGain);
        Assert.GreaterOrEqual(sanitized.RotationFarGain, sanitized.RotationNearGain);
        Assert.Greater(sanitized.TranslationTransitionMeters, 0.0f);
        Assert.Greater(sanitized.PositionTimeConstantSeconds, 0.0f);
    }

    [Test]
    public void GripPressAnchorsWithoutJump_ReleaseStopsMotion()
    {
        var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
        Ur5Continuous6DofStepResult anchored = controller.Step(CreateInput(
            gripHeld: true, Vector3.zero, Quaternion.identity));

        Assert.AreEqual(Ur5TeleopControllerState.Clutched, anchored.State);
        Assert.IsTrue(anchored.WasAnchoredThisStep);
        AssertVectorNear(new Vector3(0.40f, 0.50f, 0.60f), anchored.TargetPosition, 0.000001f);

        Ur5Continuous6DofStepResult released = controller.Step(CreateInput(
            gripHeld: false,
            new Vector3(0.20f, 0.0f, 0.0f),
            Quaternion.AngleAxis(30.0f, Vector3.up)));

        Assert.AreEqual(Ur5TeleopControllerState.Idle, released.State);
        Assert.IsFalse(released.IsMotionCommandActive);
    }

    [TestCase(1.0f, 0.0f, 0.0f)]
    [TestCase(0.0f, 1.0f, 0.0f)]
    [TestCase(0.0f, 0.0f, 1.0f)]
    public void TranslationSingleAxis_DoesNotCouple(float x, float y, float z)
    {
        var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
        controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));

        Vector3 handDelta = new Vector3(x, y, z) * 0.001f;
        Ur5Continuous6DofStepResult result =
            controller.Step(CreateInput(true, handDelta, Quaternion.identity));

        Vector3 tcpDelta = result.TargetPosition - new Vector3(0.40f, 0.50f, 0.60f);
        AssertVectorNear(handDelta * 0.15f, tcpDelta, 0.000001f);
    }

    [TestCase(1.0f, 0.0f, 0.0f)]
    [TestCase(0.0f, 1.0f, 0.0f)]
    [TestCase(0.0f, 0.0f, 1.0f)]
    public void RotationSingleAxis_MapsFullQuaternion(float x, float y, float z)
    {
        var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
        controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
        Vector3 axis = new Vector3(x, y, z);

        Ur5Continuous6DofStepResult result = controller.Step(CreateInput(
            true, Vector3.zero, Quaternion.AngleAxis(1.0f, axis)));

        Assert.AreEqual(1.0f, result.ControllerAngleDegrees, 0.001f);
        Assert.AreEqual(0.25f, result.MappedAngleDegrees, 0.001f);
        Quaternion expected =
            Quaternion.AngleAxis(0.25f, axis) * Quaternion.Euler(5.0f, 10.0f, 15.0f);
        Assert.LessOrEqual(Quaternion.Angle(expected, result.TargetRotation), 0.001f);
    }

    [Test]
    public void RotationAcrossQuaternionSign_UsesShortestPath()
    {
        var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
        Quaternion anchor = Quaternion.AngleAxis(179.0f, Vector3.up);
        controller.Step(CreateInput(true, Vector3.zero, anchor));
        Quaternion current = Quaternion.AngleAxis(181.0f, Vector3.up);

        Ur5Continuous6DofStepResult result =
            controller.Step(CreateInput(true, Vector3.zero, current));

        Assert.AreEqual(2.0f, result.ControllerAngleDegrees, 0.01f);
        Assert.Less(result.MappedAngleDegrees, 1.0f);
    }

    [Test]
    public void InvalidInputFault_LatchesUntilGripRelease()
    {
        var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
        Ur5Continuous6DofStepInput invalid =
            CreateInput(true, new Vector3(float.NaN, 0.0f, 0.0f), Quaternion.identity);

        Ur5Continuous6DofStepResult fault = controller.Step(invalid);
        Assert.AreEqual(Ur5TeleopControllerState.Fault, fault.State);
        Assert.AreEqual(Ur5Continuous6DofFaultReason.InvalidControllerPose, fault.FaultReason);

        Ur5Continuous6DofStepResult stillFaulted =
            controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
        Assert.AreEqual(Ur5TeleopControllerState.Fault, stillFaulted.State);

        controller.Step(CreateInput(false, Vector3.zero, Quaternion.identity));
        Ur5Continuous6DofStepResult recovered =
            controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
        Assert.AreEqual(Ur5TeleopControllerState.Clutched, recovered.State);
    }

    [Test]
    public void Pause_RequiresGripReleaseBeforeReclutch()
    {
        var controller = new Ur5Continuous6DofClutchController(Ur5Continuous6DofConfig.Default);
        controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity));
        controller.Pause();

        Assert.AreEqual(
            Ur5TeleopControllerState.Paused,
            controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity)).State);

        controller.Step(CreateInput(false, Vector3.zero, Quaternion.identity));
        Assert.AreEqual(
            Ur5TeleopControllerState.Clutched,
            controller.Step(CreateInput(true, Vector3.zero, Quaternion.identity)).State);
    }

    private static Ur5Continuous6DofStepInput CreateInput(
        bool gripHeld,
        Vector3 controllerPosition,
        Quaternion controllerRotation)
    {
        return new Ur5Continuous6DofStepInput(
            gripHeld,
            controllerPosition,
            controllerRotation,
            new Vector3(0.40f, 0.50f, 0.60f),
            Quaternion.Euler(5.0f, 10.0f, 15.0f),
            isInputPoseValid: true,
            isRobotStateValid: true,
            isSafetyAccepted: true);
    }

    private static void AssertVectorNear(Vector3 expected, Vector3 actual, float tolerance)
    {
        Assert.LessOrEqual(Vector3.Distance(expected, actual), tolerance);
    }
}

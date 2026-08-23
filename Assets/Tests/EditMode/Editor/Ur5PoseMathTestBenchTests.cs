using NUnit.Framework;
using UnityEngine;

public class Ur5PoseMathTestBenchTests
{
    [Test]
    public void TranslationAlongRobotBaseAxes_ProducesOnlyTheExpectedTcpDelta()
    {
        Ur5PoseMathTestBench.ClutchSnapshot clutch = CreateDefaultClutch();

        AssertSingleAxisTranslation(clutch, new Vector3(0.10f, 0.0f, 0.0f));
        AssertSingleAxisTranslation(clutch, new Vector3(0.0f, 0.10f, 0.0f));
        AssertSingleAxisTranslation(clutch, new Vector3(0.0f, 0.0f, 0.10f));
    }

    [Test]
    public void RotationAroundSingleControllerAxes_DoesNotTranslateTcp()
    {
        Ur5PoseMathTestBench.ClutchSnapshot clutch = CreateDefaultClutch();

        AssertSingleAxisRotation(clutch, Vector3.right, 25.0f);
        AssertSingleAxisRotation(clutch, Vector3.up, 25.0f);
        AssertSingleAxisRotation(clutch, Vector3.forward, 25.0f);
    }

    [Test]
    public void RightHandTranslationOnly_KeepsTcpRotationLocked()
    {
        Ur5PoseMathTestBench.ClutchSnapshot clutch = CreateDefaultClutch();
        Ur5PoseMathTestBench.MappingSettings settings = Ur5PoseMathTestBench.MappingSettings.Default;
        settings.RotationScale = 0.0f;

        Ur5PoseMathTestBench.FramePose controller = CreateControllerPose(new Vector3(0.12f, -0.04f, 0.08f));
        Ur5PoseMathTestBench.Sample sample = Ur5PoseMathTestBench.Evaluate(clutch, controller, settings);

        Assert.LessOrEqual(Quaternion.Angle(clutch.TcpWorld.Rotation, sample.FinalTargetWorld.Rotation), 0.0001f);
        Assert.LessOrEqual(Quaternion.Angle(Quaternion.identity, sample.MappedTcpRotationDelta), 0.0001f);
    }

    [Test]
    public void QuaternionDelta_UsesWorldLeftMultiplyConvention()
    {
        Ur5PoseMathTestBench.ClutchSnapshot clutch = CreateDefaultClutch();
        Quaternion worldYaw = Quaternion.AngleAxis(90.0f, Vector3.up);
        Ur5PoseMathTestBench.FramePose controller = CreateControllerPose(Vector3.zero, worldYaw);

        Ur5PoseMathTestBench.Sample sample = Ur5PoseMathTestBench.Evaluate(
            clutch,
            controller,
            Ur5PoseMathTestBench.MappingSettings.Default);

        Quaternion expected = worldYaw * clutch.TcpWorld.Rotation;
        Assert.LessOrEqual(Quaternion.Angle(expected, sample.FinalTargetWorld.Rotation), 0.0001f);
    }

    [Test]
    public void Sample_ExposesControllerDeltaMappedTcpDeltaAndFinalTargetPose()
    {
        Ur5PoseMathTestBench.ClutchSnapshot clutch = CreateDefaultClutch();
        Ur5PoseMathTestBench.FramePose controller = CreateControllerPose(new Vector3(0.04f, 0.02f, -0.03f));

        Ur5PoseMathTestBench.Sample sample = Ur5PoseMathTestBench.Evaluate(
            clutch,
            controller,
            Ur5PoseMathTestBench.MappingSettings.Default);

        AssertVectorNear(new Vector3(0.04f, 0.02f, -0.03f), sample.ControllerRobotBaseDelta, 0.000001f);
        AssertVectorNear(new Vector3(0.04f, 0.02f, -0.03f), sample.ControllerXrOriginDelta, 0.000001f);
        AssertVectorNear(sample.MappedTcpWorldDelta, sample.FinalTargetWorld.Position - clutch.TcpWorld.Position, 0.000001f);
        Assert.IsTrue(Ur5PoseMathTestBench.IsFinite(sample.FinalTargetWorld));
    }

    private static Ur5PoseMathTestBench.ClutchSnapshot CreateDefaultClutch()
    {
        Quaternion sharedFrameRotation = Quaternion.Euler(0.0f, 30.0f, 0.0f);
        return new Ur5PoseMathTestBench.ClutchSnapshot(
            xrOriginWorld: new Ur5PoseMathTestBench.FramePose(
                new Vector3(1.0f, 0.2f, -0.4f),
                sharedFrameRotation),
            robotBaseWorld: new Ur5PoseMathTestBench.FramePose(
                new Vector3(-0.2f, 0.0f, 0.5f),
                sharedFrameRotation),
            controllerWorld: CreateControllerPose(Vector3.zero),
            tcpWorld: new Ur5PoseMathTestBench.FramePose(
                new Vector3(0.45f, 0.32f, 0.18f),
                Quaternion.Euler(12.0f, -20.0f, 8.0f)));
    }

    private static Ur5PoseMathTestBench.FramePose CreateControllerPose(Vector3 baseLocalOffset)
    {
        return CreateControllerPose(baseLocalOffset, Quaternion.identity);
    }

    private static Ur5PoseMathTestBench.FramePose CreateControllerPose(Vector3 baseLocalOffset, Quaternion worldRotationDelta)
    {
        Quaternion baseRotation = Quaternion.Euler(0.0f, 30.0f, 0.0f);
        Vector3 baseOrigin = new Vector3(-0.2f, 0.0f, 0.5f);
        Vector3 startPosition = baseOrigin + baseRotation * new Vector3(0.2f, 1.0f, -0.3f);
        Quaternion startRotation = Quaternion.Euler(5.0f, 15.0f, -10.0f);

        return new Ur5PoseMathTestBench.FramePose(
            startPosition + baseRotation * baseLocalOffset,
            worldRotationDelta * startRotation);
    }

    private static void AssertSingleAxisTranslation(
        Ur5PoseMathTestBench.ClutchSnapshot clutch,
        Vector3 baseLocalDelta)
    {
        Ur5PoseMathTestBench.FramePose controller = CreateControllerPose(baseLocalDelta);
        Ur5PoseMathTestBench.Sample sample = Ur5PoseMathTestBench.Evaluate(
            clutch,
            controller,
            Ur5PoseMathTestBench.MappingSettings.Default);

        AssertVectorNear(baseLocalDelta, sample.ControllerRobotBaseDelta, 0.000001f);
        AssertVectorNear(baseLocalDelta, sample.MappedTcpRobotBaseDelta, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(clutch.TcpWorld.Rotation, sample.FinalTargetWorld.Rotation), 0.0001f);
    }

    private static void AssertSingleAxisRotation(
        Ur5PoseMathTestBench.ClutchSnapshot clutch,
        Vector3 worldAxis,
        float degrees)
    {
        Ur5PoseMathTestBench.FramePose controller = CreateControllerPose(
            Vector3.zero,
            Quaternion.AngleAxis(degrees, worldAxis));
        Ur5PoseMathTestBench.Sample sample = Ur5PoseMathTestBench.Evaluate(
            clutch,
            controller,
            Ur5PoseMathTestBench.MappingSettings.Default);

        AssertVectorNear(Vector3.zero, sample.ControllerWorldDelta, 0.000001f);
        AssertVectorNear(Vector3.zero, sample.MappedTcpWorldDelta, 0.000001f);
        Assert.AreEqual(degrees, sample.ControllerRotationDeltaDegrees, 0.0001f);
    }

    private static void AssertVectorNear(Vector3 expected, Vector3 actual, float tolerance)
    {
        Assert.LessOrEqual(Vector3.Distance(expected, actual), tolerance);
    }
}

using NUnit.Framework;
using UnityEngine;

public class Ur5RelativePoseClutchMapperTests
{
    [Test]
    public void BeginAndMapAtBaseline_DoNotMoveTheTcpTarget()
    {
        var mapper = new Ur5RelativePoseClutchMapper();
        Vector3 handPosition = new Vector3(0.2f, 1.1f, -0.3f);
        Quaternion handRotation = Quaternion.Euler(10.0f, 20.0f, 30.0f);
        Vector3 targetPosition = new Vector3(0.4f, 0.5f, 0.6f);
        Quaternion targetRotation = Quaternion.Euler(-5.0f, 35.0f, 0.0f);

        Assert.IsTrue(mapper.Begin(handPosition, handRotation, targetPosition, targetRotation));
        Assert.IsTrue(mapper.TryMap(handPosition, handRotation, 0.8f, 0.8f,
            out Vector3 mappedPosition, out Quaternion mappedRotation));

        AssertVectorNear(targetPosition, mappedPosition, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(targetRotation, mappedRotation), 0.0001f);
    }

    [Test]
    public void End_StopsAcceptingFurtherHandPoseChanges()
    {
        var mapper = new Ur5RelativePoseClutchMapper();
        Assert.IsTrue(mapper.Begin(Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity));
        mapper.End();

        Assert.IsFalse(mapper.TryMap(Vector3.one, Quaternion.Euler(0.0f, 45.0f, 0.0f), 1.0f, 1.0f,
            out _, out _));
    }

    [Test]
    public void RebaseForPrecisionModifier_DoesNotCauseATargetJump()
    {
        var mapper = new Ur5RelativePoseClutchMapper();
        Assert.IsTrue(mapper.Begin(Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity));
        Assert.IsTrue(mapper.TryMap(new Vector3(0.1f, 0.0f, 0.0f), Quaternion.Euler(0.0f, 20.0f, 0.0f),
            0.8f, 0.8f, out Vector3 beforePosition, out Quaternion beforeRotation));

        Assert.IsTrue(mapper.Rebase(new Vector3(0.1f, 0.0f, 0.0f), Quaternion.Euler(0.0f, 20.0f, 0.0f),
            beforePosition, beforeRotation));
        Assert.IsTrue(mapper.TryMap(new Vector3(0.1f, 0.0f, 0.0f), Quaternion.Euler(0.0f, 20.0f, 0.0f),
            0.15f, 0.15f, out Vector3 afterPosition, out Quaternion afterRotation));

        AssertVectorNear(beforePosition, afterPosition, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(beforeRotation, afterRotation), 0.0001f);
    }

    [Test]
    public void PositionMapping_IsNotMultipliedByDeltaTime()
    {
        var mapper = new Ur5RelativePoseClutchMapper();
        Assert.IsTrue(mapper.Begin(Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity));

        Assert.IsTrue(mapper.TryMap(new Vector3(0.1f, 0.0f, 0.0f), Quaternion.identity, 0.8f, 1.0f,
            out Vector3 position, out _));

        Assert.AreEqual(0.08f, position.x, 0.000001f);
    }

    [Test]
    public void FilterResponse_IsConsistentAcrossUpdateRates()
    {
        Vector3 at60 = SimulateFilter(60.0f);
        Vector3 at72 = SimulateFilter(72.0f);
        Vector3 at90 = SimulateFilter(90.0f);
        Vector3 at120 = SimulateFilter(120.0f);

        AssertVectorNear(at60, at72, 0.0005f);
        AssertVectorNear(at60, at90, 0.0005f);
        AssertVectorNear(at60, at120, 0.0005f);
    }

    [Test]
    public void TimeConstantFilter_IsConsistentAt72_90And120Hz()
    {
        (Vector3 position, Quaternion rotation) at72 = SimulateTimeConstantFilter(72.0f);
        (Vector3 position, Quaternion rotation) at90 = SimulateTimeConstantFilter(90.0f);
        (Vector3 position, Quaternion rotation) at120 = SimulateTimeConstantFilter(120.0f);

        Assert.LessOrEqual(Vector3.Distance(at72.position, at90.position), 0.0005f);
        Assert.LessOrEqual(Vector3.Distance(at72.position, at120.position), 0.0005f);
        Assert.LessOrEqual(Quaternion.Angle(at72.rotation, at90.rotation), 0.1f);
        Assert.LessOrEqual(Quaternion.Angle(at72.rotation, at120.rotation), 0.1f);
    }

    [Test]
    public void TimeConstantFilter_EnforcesLinearSpeedLimit()
    {
        var filter = new Ur5RelativePoseCommandFilter();
        filter.Reset(Vector3.zero, Quaternion.identity);

        Assert.IsTrue(filter.FilterByTimeConstants(
            Vector3.right, Quaternion.identity,
            0.055f, 0.055f, 0.01f,
            0.26f, 4.0f * Mathf.Rad2Deg,
            out Vector3 position, out _));

        Assert.LessOrEqual(position.magnitude, 0.26f * 0.01f + 0.000001f);
    }

    [Test]
    public void TimeConstantFilter_EnforcesAngularSpeedLimit()
    {
        var filter = new Ur5RelativePoseCommandFilter();
        filter.Reset(Vector3.zero, Quaternion.identity);

        Assert.IsTrue(filter.FilterByTimeConstants(
            Vector3.zero, Quaternion.AngleAxis(90.0f, Vector3.up),
            0.055f, 0.055f, 0.01f,
            0.26f, 100.0f,
            out _, out Quaternion rotation));

        Assert.LessOrEqual(Quaternion.Angle(Quaternion.identity, rotation), 1.0001f);
    }

    [Test]
    public void TimeConstantFilter_RejectsInvalidInputAndPreservesLastOutput()
    {
        var filter = new Ur5RelativePoseCommandFilter();
        filter.Reset(new Vector3(0.1f, 0.2f, 0.3f), Quaternion.Euler(1.0f, 2.0f, 3.0f));

        Assert.IsFalse(filter.FilterByTimeConstants(
            new Vector3(float.NaN, 0.0f, 0.0f), Quaternion.identity,
            0.055f, 0.055f, 0.01f,
            0.26f, 100.0f,
            out Vector3 invalidPosition, out Quaternion invalidRotation));

        AssertVectorNear(new Vector3(0.1f, 0.2f, 0.3f), invalidPosition, 0.000001f);
        Assert.LessOrEqual(
            Quaternion.Angle(Quaternion.Euler(1.0f, 2.0f, 3.0f), invalidRotation),
            0.0001f);
    }

    [Test]
    public void PositionNoiseGate_HoldsIdleJitterWithoutTimeBasedLag()
    {
        var gate = new Ur5PositionNoiseGate();
        gate.Reset(Vector3.zero);

        Vector3 held = gate.Filter(new Vector3(0.0008f, 0.0f, 0.0f), 0.0010f);
        AssertVectorNear(Vector3.zero, held, 0.0000001f);

        Vector3 moved = gate.Filter(new Vector3(0.0040f, 0.0f, 0.0f), 0.0010f);
        Assert.AreEqual(0.0030f, moved.x, 0.000001f);

        Vector3 heldAgain = gate.Filter(new Vector3(0.0038f, 0.0f, 0.0f), 0.0010f);
        AssertVectorNear(moved, heldAgain, 0.0000001f);
    }

    [Test]
    public void PositionNoiseGate_RejectsNonFiniteInput()
    {
        var gate = new Ur5PositionNoiseGate();
        gate.Reset(new Vector3(0.1f, 0.2f, 0.3f));

        Vector3 result = gate.Filter(new Vector3(float.NaN, 0.0f, 0.0f), 0.0010f);

        AssertVectorNear(new Vector3(0.1f, 0.2f, 0.3f), result, 0.0000001f);
    }

    [Test]
    public void RotationNoiseGate_HoldsIdleQuaternionJitterWithoutTimeBasedLag()
    {
        var gate = new Ur5RotationNoiseGate();
        Quaternion start = Quaternion.Euler(0.0f, 10.0f, 0.0f);
        gate.Reset(start);

        Quaternion held = gate.Filter(Quaternion.Euler(0.0f, 10.6f, 0.0f), 1.0f);
        Assert.LessOrEqual(Quaternion.Angle(start, held), 0.0001f);

        Quaternion moved = gate.Filter(Quaternion.Euler(0.0f, 13.0f, 0.0f), 1.0f);
        Assert.LessOrEqual(Quaternion.Angle(Quaternion.Euler(0.0f, 12.0f, 0.0f), moved), 0.0001f);
    }

    [Test]
    public void RotationNoiseGate_RejectsInvalidInput()
    {
        var gate = new Ur5RotationNoiseGate();
        Quaternion start = Quaternion.Euler(1.0f, 2.0f, 3.0f);
        gate.Reset(start);

        Quaternion result = gate.Filter(new Quaternion(float.NaN, 0.0f, 0.0f, 1.0f), 1.0f);

        Assert.LessOrEqual(Quaternion.Angle(start, result), 0.0001f);
    }

    [Test]
    public void AnchoredStrategy_ResumesAtActualToolPoseWithoutJump()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();
        Vector3 hand = new Vector3(0.3f, 1.2f, -0.2f);
        Vector3 tool = new Vector3(0.6f, 0.4f, 0.1f);
        Quaternion rotation = Quaternion.Euler(0.0f, 30.0f, 0.0f);

        Assert.IsTrue(tracker.Resume(hand, Quaternion.identity, tool, rotation));
        Assert.IsTrue(tracker.TryGetRequestedPose(
            hand, Quaternion.identity, Vector3.one * 0.5f, rotation,
            out Vector3 requestedPosition, out Quaternion requestedRotation));
        Assert.IsTrue(tracker.FilterRequestedPose(
            requestedPosition, requestedRotation, 0.18f,
            out Vector3 filteredPosition, out Quaternion filteredRotation));

        AssertVectorNear(tool, filteredPosition, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(rotation, filteredRotation), 0.0001f);
    }

    [Test]
    public void AnchoredStrategy_MapsDisplacementOnceWithoutDeltaTime()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();
        Assert.IsTrue(tracker.Resume(Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity));

        Assert.IsTrue(tracker.TryGetRequestedPose(
            new Vector3(0.1f, -0.2f, 0.3f), Quaternion.identity,
            new Vector3(0.5f, 0.5f, 0.5f), Quaternion.identity,
            out Vector3 requestedPosition, out _));

        AssertVectorNear(new Vector3(0.05f, -0.1f, 0.15f), requestedPosition, 0.000001f);
    }

    [Test]
    public void AnchoredStrategy_MapsTwentyMillimetresToTenMillimetresAtFixedScale()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();
        Vector3 toolAnchor = new Vector3(0.40f, 0.50f, 0.60f);
        Assert.IsTrue(tracker.Resume(
            new Vector3(0.10f, 1.20f, -0.30f),
            Quaternion.Euler(10.0f, 20.0f, 30.0f),
            toolAnchor,
            Quaternion.Euler(-5.0f, 35.0f, 15.0f)));

        Assert.IsTrue(tracker.TryGetRequestedPose(
            new Vector3(0.12f, 1.20f, -0.30f),
            Quaternion.Euler(10.0f, 20.0f, 30.0f),
            Vector3.one * 0.5f,
            Quaternion.Euler(-5.0f, 35.0f, 15.0f),
            out Vector3 requestedPosition,
            out _));

        AssertVectorNear(toolAnchor + new Vector3(0.010f, 0.0f, 0.0f), requestedPosition, 0.000001f);
    }

    [Test]
    public void AnchoredStrategy_MapsFullRelativeControllerQuaternionOntoToolAnchor()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();
        Quaternion inputAnchor = Quaternion.Euler(20.0f, -35.0f, 15.0f);
        Quaternion toolAnchor = Quaternion.Euler(-10.0f, 50.0f, 25.0f);
        Quaternion currentInput = Quaternion.Euler(45.0f, 10.0f, -30.0f);
        Quaternion expectedToolRotation = currentInput * Quaternion.Inverse(inputAnchor) * toolAnchor;
        Assert.IsTrue(tracker.Resume(Vector3.zero, inputAnchor, Vector3.one, toolAnchor));

        Assert.IsTrue(tracker.TryGetRequestedPose(
            Vector3.zero,
            currentInput,
            Vector3.one * 0.5f,
            out _,
            out Quaternion requestedRotation));

        Assert.LessOrEqual(Quaternion.Angle(expectedToolRotation, requestedRotation), 0.0001f);
    }

    [Test]
    public void AnchoredStrategy_RepeatedUnchangedInputRetainsImmutableRequestedPoseAndConvergesMonotonically()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();
        Vector3 toolAnchor = new Vector3(0.40f, 0.50f, 0.60f);
        Vector3 handAnchor = new Vector3(0.10f, 1.20f, -0.30f);
        Vector3 currentHand = handAnchor + new Vector3(0.020f, 0.0f, 0.0f);
        Quaternion toolRotation = Quaternion.Euler(-5.0f, 35.0f, 15.0f);
        Assert.IsTrue(tracker.Resume(handAnchor, Quaternion.identity, toolAnchor, toolRotation));
        Assert.IsTrue(tracker.TryGetRequestedPose(
            currentHand,
            Quaternion.identity,
            Vector3.one * 0.5f,
            toolRotation,
            out Vector3 requestedPosition,
            out Quaternion requestedRotation));

        float previousError = Vector3.Distance(tracker.TargetPosition, requestedPosition);
        for (int index = 0; index < 5; index++)
        {
            Assert.IsTrue(tracker.FilterRequestedPose(
                requestedPosition,
                requestedRotation,
                0.05f,
                out Vector3 filteredPosition,
                out _));
            float error = Vector3.Distance(filteredPosition, requestedPosition);
            Assert.Less(error, previousError);
            previousError = error;

            Assert.IsTrue(tracker.TryGetRequestedPose(
                currentHand,
                Quaternion.identity,
                Vector3.one * 0.5f,
                toolRotation,
                out Vector3 repeatedRequestedPosition,
                out _));
            AssertVectorNear(requestedPosition, repeatedRequestedPosition, 0.000001f);
        }
    }

    [Test]
    public void AnchoredStrategy_RebaseForPrecisionDoesNotJump()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();
        Assert.IsTrue(tracker.Resume(Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity));
        Assert.IsTrue(tracker.TryGetRequestedPose(
            new Vector3(0.1f, 0.0f, 0.0f), Quaternion.identity,
            Vector3.one * 0.5f, Quaternion.identity,
            out Vector3 targetBeforeRebase, out Quaternion rotationBeforeRebase));
        Assert.IsTrue(tracker.FilterRequestedPose(
            targetBeforeRebase, rotationBeforeRebase, 1.0f,
            out Vector3 filteredBeforeRebase, out Quaternion filteredRotationBeforeRebase));

        Assert.IsTrue(tracker.Rebase(
            new Vector3(0.1f, 0.0f, 0.0f), Quaternion.identity,
            filteredBeforeRebase, filteredRotationBeforeRebase));
        Assert.IsTrue(tracker.TryGetRequestedPose(
            new Vector3(0.1f, 0.0f, 0.0f), Quaternion.identity,
            Vector3.one * 0.15f, filteredRotationBeforeRebase,
            out Vector3 targetAfterRebase, out Quaternion rotationAfterRebase));

        AssertVectorNear(filteredBeforeRebase, targetAfterRebase, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(filteredRotationBeforeRebase, rotationAfterRebase), 0.0001f);
    }

    [Test]
    public void AnchoredStrategy_PauseStopsAcceptingInputChanges()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();
        Vector3 toolPosition = new Vector3(0.4f, 0.5f, 0.6f);
        Quaternion toolRotation = Quaternion.Euler(5.0f, 10.0f, 15.0f);
        Assert.IsTrue(tracker.Resume(Vector3.zero, Quaternion.identity, toolPosition, toolRotation));
        Assert.IsTrue(tracker.FilterRequestedPose(
            toolPosition + Vector3.right,
            Quaternion.Euler(25.0f, 10.0f, 15.0f),
            0.05f,
            out Vector3 targetBeforeRelease,
            out Quaternion rotationBeforeRelease));
        tracker.Pause();

        Assert.IsFalse(tracker.TryGetRequestedPose(
            Vector3.one, Quaternion.identity, Vector3.one, Quaternion.identity,
            out _, out _));
        Assert.IsFalse(tracker.FilterRequestedPose(
            Vector3.zero, Quaternion.identity, 0.05f, out _, out _));
        AssertVectorNear(targetBeforeRelease, tracker.TargetPosition, 0.000001f);
        Assert.LessOrEqual(Quaternion.Angle(rotationBeforeRelease, tracker.TargetRotation), 0.0001f);
    }

    [Test]
    public void AnchoredStrategy_RejectsNonFinitePose()
    {
        var tracker = new Ur5AnchoredPoseTeleopStrategy();

        Assert.IsFalse(tracker.Resume(
            new Vector3(float.NaN, 0.0f, 0.0f), Quaternion.identity,
            Vector3.zero, Quaternion.identity));
        Assert.IsFalse(tracker.IsTracking);
    }

    private static Vector3 SimulateFilter(float updateRateHz)
    {
        var filter = new Ur5RelativePoseCommandFilter();
        filter.Reset(Vector3.zero, Quaternion.identity);
        float deltaTime = 1.0f / updateRateHz;
        int steps = Mathf.RoundToInt(updateRateHz * 0.5f);
        Vector3 position = Vector3.zero;
        for (int index = 0; index < steps; index++)
        {
            filter.Filter(Vector3.one, Quaternion.identity, 0.65f, deltaTime, 90.0f,
                out position, out _);
        }

        return position;
    }

    private static (Vector3 position, Quaternion rotation) SimulateTimeConstantFilter(float updateRateHz)
    {
        var filter = new Ur5RelativePoseCommandFilter();
        filter.Reset(Vector3.zero, Quaternion.identity);
        float deltaTime = 1.0f / updateRateHz;
        int steps = Mathf.RoundToInt(updateRateHz * 0.5f);
        Vector3 position = Vector3.zero;
        Quaternion rotation = Quaternion.identity;
        for (int index = 0; index < steps; index++)
        {
            filter.FilterByTimeConstants(
                Vector3.one * 0.1f,
                Quaternion.Euler(20.0f, 30.0f, 40.0f),
                0.055f,
                0.055f,
                deltaTime,
                10.0f,
                720.0f,
                out position,
                out rotation);
        }

        return (position, rotation);
    }

    private static void AssertVectorNear(Vector3 expected, Vector3 actual, float tolerance)
    {
        Assert.LessOrEqual(Vector3.Distance(expected, actual), tolerance);
    }
}

using NUnit.Framework;

public class Ur5LeftSafetyPoseControllerTests
{
    [Test]
    public void ShortPress_RequestsOneSnapAndBlocksRightGripUntilReached()
    {
        var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);

        Ur5LeftSafetyPoseStepResult pressed = controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.10f));
        Assert.AreEqual(Ur5LeftSafetyPoseState.ButtonHeld, pressed.State);
        Assert.IsTrue(pressed.BlocksRightGrip);

        Ur5LeftSafetyPoseStepResult released = controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: false,
            deltaTimeSeconds: 0.01f));
        Assert.AreEqual(Ur5LeftSafetyPoseState.SnapDownActive, released.State);
        Assert.IsTrue(released.RequestSnapDown);
        Assert.IsTrue(released.BlocksRightGrip);

        Ur5LeftSafetyPoseStepResult active = controller.Step(CreateInput(
            rightGripHeld: true,
            deltaTimeSeconds: 0.02f));
        Assert.AreEqual(Ur5LeftSafetyPoseState.SnapDownActive, active.State);
        Assert.IsFalse(active.RequestSnapDown);
        Assert.IsTrue(active.BlocksRightGrip);

        Ur5LeftSafetyPoseStepResult reached = controller.Step(CreateInput(
            rightGripHeld: true,
            snapTargetReached: true,
            deltaTimeSeconds: 0.02f));
        Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, reached.State);
        Assert.IsFalse(reached.BlocksRightGrip);
    }

    [Test]
    public void LongPress_RequestsReadyPoseInsteadOfSnapDown()
    {
        var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);
        controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.20f));
        controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.20f));

        Ur5LeftSafetyPoseStepResult result = controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.06f));

        Assert.IsTrue(result.RequestReadyPose);
        Assert.IsFalse(result.RequestSnapDown);
        Assert.AreEqual(Ur5LeftSafetyPoseState.ReadyPoseActive, result.State);
        Assert.IsTrue(result.BlocksRightGrip);
    }

    [Test]
    public void ReadyPose_ReleaseRequestsCancellationOnlyWhileFollowerIsActive()
    {
        var controller = CreateReadyPoseController();

        Ur5LeftSafetyPoseStepResult released = controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: false,
            readyPoseActive: true,
            deltaTimeSeconds: 0.01f));
        Assert.IsTrue(released.CancelReadyPose);
        Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, released.State);

        Ur5LeftSafetyPoseStepResult idle = controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: false,
            readyPoseActive: true,
            deltaTimeSeconds: 0.01f));
        Assert.IsFalse(idle.CancelReadyPose);
    }

    [Test]
    public void SnapDown_TimesOutOnceAndReturnsIdle()
    {
        var controller = CreateSnapDownController();

        Ur5LeftSafetyPoseStepResult beforeTimeout = controller.Step(CreateInput(
            deltaTimeSeconds: 2.90f));
        Assert.AreEqual(Ur5LeftSafetyPoseState.SnapDownActive, beforeTimeout.State);
        Assert.IsFalse(beforeTimeout.SnapTimedOut);

        Ur5LeftSafetyPoseStepResult timedOut = controller.Step(CreateInput(
            deltaTimeSeconds: 0.11f));
        Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, timedOut.State);
        Assert.IsTrue(timedOut.SnapTimedOut);

        Ur5LeftSafetyPoseStepResult idle = controller.Step(CreateInput(
            deltaTimeSeconds: 1.0f));
        Assert.IsFalse(idle.SnapTimedOut);
    }

    [Test]
    public void RightGripHeld_PreventsLeftSafetyPoseFromStarting()
    {
        var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);

        Ur5LeftSafetyPoseStepResult result = controller.Step(CreateInput(
            rightGripHeld: true,
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.50f));

        Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, result.State);
        Assert.IsFalse(result.BlocksRightGrip);
        Assert.IsFalse(result.RequestReadyPose);
    }

    [Test]
    public void InvalidLeftPoseOrReleasedGrip_CancelsButtonWithoutSnap()
    {
        var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);
        controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.10f));

        Ur5LeftSafetyPoseStepResult invalid = controller.Step(CreateInput(
            leftPoseValid: false,
            leftGripHeld: true,
            primaryPressed: false,
            deltaTimeSeconds: 0.01f));

        Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, invalid.State);
        Assert.IsFalse(invalid.RequestSnapDown);
    }

    [Test]
    public void Reset_ClearsActiveSafetyOwnership()
    {
        var controller = CreateSnapDownController();

        controller.Reset();

        Assert.AreEqual(Ur5LeftSafetyPoseState.Idle, controller.State);
        Assert.IsFalse(controller.BlocksRightGrip);
    }

    private static Ur5LeftSafetyPoseController CreateReadyPoseController()
    {
        var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);
        controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.45f));
        return controller;
    }

    private static Ur5LeftSafetyPoseController CreateSnapDownController()
    {
        var controller = new Ur5LeftSafetyPoseController(0.45f, 3.0f);
        controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: true,
            deltaTimeSeconds: 0.10f));
        controller.Step(CreateInput(
            leftGripHeld: true,
            primaryPressed: false,
            deltaTimeSeconds: 0.01f));
        return controller;
    }

    private static Ur5LeftSafetyPoseStepInput CreateInput(
        bool rightGripHeld = false,
        bool leftPoseValid = true,
        bool leftGripHeld = false,
        bool primaryPressed = false,
        bool snapTargetReached = false,
        bool readyPoseActive = false,
        float deltaTimeSeconds = 0.02f)
    {
        return new Ur5LeftSafetyPoseStepInput(
            rightGripHeld,
            leftPoseValid,
            leftGripHeld,
            primaryPressed,
            snapTargetReached,
            readyPoseActive,
            deltaTimeSeconds);
    }
}

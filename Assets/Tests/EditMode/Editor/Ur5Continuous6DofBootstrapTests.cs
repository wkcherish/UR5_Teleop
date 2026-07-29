using System;
using System.Reflection;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.XR;

public class Ur5Continuous6DofBootstrapTests
{
    [Test]
    public void DefaultQuestProfile_UsesOnlyRightHandContinuous6Dof()
    {
        GameObject owner = new GameObject("bootstrap-profile-test");
        try
        {
            var teleop = owner.AddComponent<Ur5CartesianVelocityTeleopController>();
            Ur5ControlBootstrap.ApplyDefaultQuestTeleopProfile(teleop);

            Assert.IsTrue(teleop.enableContinuous6DofClutch);
            Assert.AreEqual(Ur5Continuous6DofConfig.Default.Version, teleop.continuous6DofConfig.Version);
            Assert.AreEqual(XRNode.RightHand, teleop.positionControllerNode);
            Assert.AreEqual(XRNode.RightHand, teleop.rotationControllerNode);
            Assert.IsTrue(teleop.usePositionGripAsDeadman);
            Assert.IsTrue(teleop.useRotationGripAsDeadman);
            Assert.AreEqual(
                Ur5CartesianVelocityTeleopController.RotationInputMode.ControllerPoseDelta,
                teleop.rotationInputMode);

            Assert.IsFalse(teleop.enableThreeModeController);
            Assert.IsFalse(teleop.useRightSecondaryButtonForInsertMode);
            Assert.IsFalse(teleop.enableAPrecisionModifier);
            Assert.IsFalse(teleop.enableFineControlButton);
            Assert.IsFalse(teleop.enableLeftSecondaryPoseRotation);
            Assert.IsFalse(teleop.enableLeftSecondaryOrientationHold);
            Assert.IsFalse(teleop.snapGraspApproachToVertical);

            Assert.IsFalse(teleop.filterControllerPosition);
            Assert.IsFalse(teleop.useAdaptiveControllerPositionFilter);
            Assert.AreEqual(0.0f, teleop.previewTargetDeadbandMeters, 0.000001f);
            Assert.AreEqual(0.0f, teleop.finePreviewTargetDeadbandMeters, 0.000001f);
            Assert.IsFalse(teleop.useAccelerationLimitedPreviewTrajectory);
            Assert.IsFalse(teleop.limitPreviewLeadToActualTcp);

            Assert.IsTrue(teleop.enableLeftPrimarySnapDown);
            Assert.IsTrue(teleop.enableLeftPrimaryReadyPose);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(owner);
        }
    }

    [Test]
    public void DefaultQuestProfile_UsesApprovedContinuousParameters()
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
                + "rotation_gain,logical_to_filtered_rotation_deg";

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

    private static void InvokeNonPublic(object target, string methodName)
    {
        MethodInfo method = target.GetType().GetMethod(
            methodName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(method, methodName + " should exist.");
        method.Invoke(target, null);
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

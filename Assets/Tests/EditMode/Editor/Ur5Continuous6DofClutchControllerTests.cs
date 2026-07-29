using NUnit.Framework;
using UnityEngine;

public class Ur5Continuous6DofClutchControllerTests
{
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
}

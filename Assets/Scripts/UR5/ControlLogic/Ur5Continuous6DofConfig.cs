using System;
using UnityEngine;

/// <summary>
/// 右手 Grip 单模式 6DoF 离合控制的版本化参数。
/// 字段名和 Tooltip 均显式标注单位，避免后续调参时混用米、度或秒。
/// </summary>
[Serializable]
public struct Ur5Continuous6DofConfig
{
    [Tooltip("配置版本号；用于后续单模式参数迁移。")]
    public int Version;
    [Tooltip("平移近端增益；小位移时手柄位移映射到 TCP 位移的倍率。")]
    public float TranslationNearGain;
    [Tooltip("平移远端增益；大位移时手柄位移映射到 TCP 位移的倍率。")]
    public float TranslationFarGain;
    [Tooltip("平移增益由近端过渡到远端的手柄距离，单位米。")]
    public float TranslationTransitionMeters;
    [Tooltip("旋转近端增益；小角度时手柄旋转映射到 TCP 旋转的倍率。")]
    public float RotationNearGain;
    [Tooltip("旋转远端增益；大角度时手柄旋转映射到 TCP 旋转的倍率。")]
    public float RotationFarGain;
    [Tooltip("旋转增益由近端过渡到远端的手柄角度，单位度。")]
    public float RotationTransitionDegrees;
    [Tooltip("最终目标位置时间常数，单位秒。")]
    public float PositionTimeConstantSeconds;
    [Tooltip("最终目标旋转时间常数，单位秒。")]
    public float RotationTimeConstantSeconds;
    [Tooltip("单次离合允许的最大 TCP 位移，单位米。")]
    public float MaxClutchTranslationMeters;
    [Tooltip("单次离合允许的最大 TCP 旋转，单位度。")]
    public float MaxClutchRotationDegrees;
    [Tooltip("滤波后 TCP 目标最大线速度，单位米每秒。")]
    public float MaxLinearSpeedMetersPerSecond;
    [Tooltip("滤波后 TCP 目标最大角速度，单位度每秒。")]
    public float MaxAngularSpeedDegreesPerSecond;

    public static Ur5Continuous6DofConfig Default => new Ur5Continuous6DofConfig
    {
        Version = 1,
        TranslationNearGain = 0.15f,
        TranslationFarGain = 0.80f,
        TranslationTransitionMeters = 0.040f,
        RotationNearGain = 0.25f,
        RotationFarGain = 0.80f,
        RotationTransitionDegrees = 20.0f,
        PositionTimeConstantSeconds = 0.055f,
        RotationTimeConstantSeconds = 0.055f,
        MaxClutchTranslationMeters = 0.35f,
        MaxClutchRotationDegrees = 45.0f,
        MaxLinearSpeedMetersPerSecond = 0.26f,
        MaxAngularSpeedDegreesPerSecond = 4.0f * Mathf.Rad2Deg
    };

    public float EvaluateTranslationGain(float controllerDistanceMeters)
    {
        Ur5Continuous6DofConfig value = Sanitized();
        float t = Mathf.Clamp01(
            Mathf.Max(0.0f, controllerDistanceMeters) / value.TranslationTransitionMeters);
        return Mathf.Lerp(value.TranslationNearGain, value.TranslationFarGain, SmoothStep01(t));
    }

    public float EvaluateRotationGain(float controllerAngleDegrees)
    {
        Ur5Continuous6DofConfig value = Sanitized();
        float t = Mathf.Clamp01(
            Mathf.Abs(controllerAngleDegrees) / value.RotationTransitionDegrees);
        return Mathf.Lerp(value.RotationNearGain, value.RotationFarGain, SmoothStep01(t));
    }

    public Ur5Continuous6DofConfig Sanitized()
    {
        Ur5Continuous6DofConfig defaults = Default;
        float translationNear = NonNegativeOrDefault(
            TranslationNearGain, defaults.TranslationNearGain);
        float rotationNear = NonNegativeOrDefault(
            RotationNearGain, defaults.RotationNearGain);

        return new Ur5Continuous6DofConfig
        {
            Version = Mathf.Max(1, Version),
            TranslationNearGain = translationNear,
            TranslationFarGain = Mathf.Max(
                translationNear,
                NonNegativeOrDefault(TranslationFarGain, defaults.TranslationFarGain)),
            TranslationTransitionMeters = PositiveOrDefault(
                TranslationTransitionMeters, defaults.TranslationTransitionMeters, 0.0001f),
            RotationNearGain = rotationNear,
            RotationFarGain = Mathf.Max(
                rotationNear,
                NonNegativeOrDefault(RotationFarGain, defaults.RotationFarGain)),
            RotationTransitionDegrees = PositiveOrDefault(
                RotationTransitionDegrees, defaults.RotationTransitionDegrees, 0.01f),
            PositionTimeConstantSeconds = PositiveOrDefault(
                PositionTimeConstantSeconds, defaults.PositionTimeConstantSeconds, 0.001f),
            RotationTimeConstantSeconds = PositiveOrDefault(
                RotationTimeConstantSeconds, defaults.RotationTimeConstantSeconds, 0.001f),
            MaxClutchTranslationMeters = NonNegativeOrDefault(
                MaxClutchTranslationMeters, defaults.MaxClutchTranslationMeters),
            MaxClutchRotationDegrees = NonNegativeOrDefault(
                MaxClutchRotationDegrees, defaults.MaxClutchRotationDegrees),
            MaxLinearSpeedMetersPerSecond = NonNegativeOrDefault(
                MaxLinearSpeedMetersPerSecond, defaults.MaxLinearSpeedMetersPerSecond),
            MaxAngularSpeedDegreesPerSecond = NonNegativeOrDefault(
                MaxAngularSpeedDegreesPerSecond, defaults.MaxAngularSpeedDegreesPerSecond)
        };
    }

    private static float SmoothStep01(float value)
    {
        // 只在同一模式内连续改变增益，不引入 free/fine/insert 档位跳变。
        value = Mathf.Clamp01(value);
        return value * value * (3.0f - 2.0f * value);
    }

    private static float NonNegativeOrDefault(float value, float fallback)
    {
        return IsFinite(value) ? Mathf.Max(0.0f, value) : fallback;
    }

    private static float PositiveOrDefault(float value, float fallback, float minimum)
    {
        return IsFinite(value) ? Mathf.Max(minimum, value) : fallback;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}

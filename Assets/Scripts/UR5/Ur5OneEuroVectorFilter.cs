using UnityEngine;

/// <summary>
/// 一欧元（One Euro）三维滤波器。
/// 静止时使用较低截止频率抑制 Quest 微抖；手部快速移动时自动提高截止频率，
/// 因此不会像固定低通滤波那样明显拖慢正常遥操作。
/// </summary>
public sealed class Ur5OneEuroVectorFilter
{
    private Vector3 lastRawValue;
    private Vector3 filteredValue;
    private Vector3 filteredDerivative;
    private bool isInitialized;

    public void Reset(Vector3 value)
    {
        lastRawValue = value;
        filteredValue = value;
        filteredDerivative = Vector3.zero;
        isInitialized = true;
    }

    public Vector3 Filter(
        Vector3 rawValue,
        float deltaTime,
        float minimumCutoffHz,
        float beta,
        float derivativeCutoffHz)
    {
        if (!isInitialized)
        {
            Reset(rawValue);
            return rawValue;
        }

        float safeDeltaTime = Mathf.Max(0.0001f, deltaTime);
        Vector3 rawDerivative = (rawValue - lastRawValue) / safeDeltaTime;
        float derivativeAlpha = CalculateAlpha(derivativeCutoffHz, safeDeltaTime);
        filteredDerivative = Vector3.Lerp(filteredDerivative, rawDerivative, derivativeAlpha);

        // 只以速度大小调节截止频率，保证 XYZ 三轴具有一致的手感。
        float adaptiveCutoff = Mathf.Max(0.01f, minimumCutoffHz)
            + Mathf.Max(0.0f, beta) * filteredDerivative.magnitude;
        float valueAlpha = CalculateAlpha(adaptiveCutoff, safeDeltaTime);
        filteredValue = Vector3.Lerp(filteredValue, rawValue, valueAlpha);
        lastRawValue = rawValue;
        return filteredValue;
    }

    private static float CalculateAlpha(float cutoffHz, float deltaTime)
    {
        float safeCutoff = Mathf.Max(0.01f, cutoffHz);
        float tau = 1.0f / (2.0f * Mathf.PI * safeCutoff);
        return 1.0f / (1.0f + tau / Mathf.Max(0.0001f, deltaTime));
    }
}

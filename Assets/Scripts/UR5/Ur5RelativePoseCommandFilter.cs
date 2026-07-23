using UnityEngine;

/// <summary>
/// 相对位姿遥操作的单级互补滤波器。
///
/// 该实现采用 Open-Teach 遥操作管线中“相对位姿 + 单级位置 Lerp / 姿态 Slerp”
/// 的同类结构：每次离合时重新建立手柄与 TCP 的共同零点，之后仅对最终 TCP
/// 命令滤波。这样不会把手柄滤波、目标滤波和关节滤波叠成多重迟滞。
/// </summary>
public sealed class Ur5RelativePoseCommandFilter
{
    private bool isInitialized;
    private Vector3 filteredPosition;
    private Quaternion filteredRotation = Quaternion.identity;

    public void Reset(Vector3 position, Quaternion rotation)
    {
        filteredPosition = position;
        filteredRotation = rotation;
        isInitialized = true;
    }

    public void Clear()
    {
        isInitialized = false;
    }

    /// <summary>
    /// 以固定保留比例平滑最终命令。retention 为 0 时完全跟随，越接近 1 越平稳。
    /// </summary>
    public void Filter(
        Vector3 targetPosition,
        Quaternion targetRotation,
        float retention,
        out Vector3 position,
        out Quaternion rotation)
    {
        if (!isInitialized)
        {
            Reset(targetPosition, targetRotation);
        }

        float clampedRetention = Mathf.Clamp01(retention);
        float updateWeight = 1.0f - clampedRetention;
        filteredPosition = Vector3.Lerp(filteredPosition, targetPosition, updateWeight);
        filteredRotation = Quaternion.Slerp(filteredRotation, targetRotation, updateWeight);
        position = filteredPosition;
        rotation = filteredRotation;
    }
}

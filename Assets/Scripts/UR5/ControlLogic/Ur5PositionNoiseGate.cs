using UnityEngine;

/// <summary>
/// Rejects stationary controller jitter without adding time-based follow lag.
/// The accepted point remains inside a radial deadband of the raw input and
/// moves immediately once deliberate motion leaves that band.
/// </summary>
public sealed class Ur5PositionNoiseGate
{
    private bool isInitialized;
    private Vector3 acceptedPosition;

    public bool IsInitialized => isInitialized;
    public Vector3 AcceptedPosition => acceptedPosition;

    public void Reset(Vector3 position)
    {
        acceptedPosition = position;
        isInitialized = true;
    }

    public void Clear()
    {
        isInitialized = false;
    }

    public Vector3 Filter(Vector3 rawPosition, float deadbandMeters)
    {
        if (!Ur5RelativePoseClutchMapper.IsFinite(rawPosition))
        {
            return acceptedPosition;
        }

        if (!isInitialized)
        {
            Reset(rawPosition);
            return acceptedPosition;
        }

        float deadband = Mathf.Max(0.0f, deadbandMeters);
        Vector3 difference = rawPosition - acceptedPosition;
        float distance = difference.magnitude;
        if (distance <= deadband || distance < 0.0000001f)
        {
            return acceptedPosition;
        }

        acceptedPosition += difference * ((distance - deadband) / distance);
        return acceptedPosition;
    }
}

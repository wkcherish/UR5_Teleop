using UnityEngine;

/// <summary>Optional, throttled TcpTarget ownership monitor.</summary>
public class TcpTargetWriteMonitor : MonoBehaviour
{
    public bool enableDiagnostics;
    [Min(0.1f)] public float warningIntervalSeconds = 1.0f;
    public string LastWriter { get; private set; } = "None";
    public int WriteConflictCount { get; private set; }
    public int TotalWriteCount { get; private set; }
    public bool HadWriteConflictThisFrame { get; private set; }

    private int lastWriteFrame = -1;
    private float nextWarningTime;

    public void RecordWrite(string writer)
    {
        writer = string.IsNullOrEmpty(writer) ? "Unknown" : writer;
        if (lastWriteFrame != Time.frameCount)
        {
            HadWriteConflictThisFrame = false;
        }

        if (lastWriteFrame == Time.frameCount && LastWriter != writer)
        {
            WriteConflictCount++;
            HadWriteConflictThisFrame = true;
            if (enableDiagnostics && Time.unscaledTime >= nextWarningTime)
            {
                Debug.LogWarning("UR5 TcpTarget write conflict: " + LastWriter + " -> " + writer);
                nextWarningTime = Time.unscaledTime + Mathf.Max(0.1f, warningIntervalSeconds);
            }
        }

        LastWriter = writer;
        lastWriteFrame = Time.frameCount;
        TotalWriteCount++;
    }
}

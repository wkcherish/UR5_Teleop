using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

public class Ur5PoseCsvRecorder : MonoBehaviour
{
    public Transform tcpTarget;
    public Ur5ArticulationJointController jointController;
    public Quest3TcpTargetController questController;
    public KeyCode toggleRecordingKey = KeyCode.R;
    public bool recordOnStart = false;
    public float sampleInterval = 0.02f;

    private StringBuilder csv;
    private float nextSampleTime;
    private bool isRecording;
    private string outputPath;

    private void Start()
    {
        csv = new StringBuilder();
        csv.AppendLine("time,quest_device_valid,quest_clutched,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg");
        outputPath = Path.Combine(Application.persistentDataPath, "ur5_pose_log.csv");

        if (recordOnStart)
        {
            StartRecording();
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(toggleRecordingKey))
        {
            if (isRecording)
            {
                StopRecording();
            }
            else
            {
                StartRecording();
            }
        }

        if (!isRecording || Time.time < nextSampleTime)
        {
            return;
        }

        nextSampleTime = Time.time + sampleInterval;
        AppendSample();
    }

    public void StartRecording()
    {
        isRecording = true;
        nextSampleTime = Time.time;
        Debug.Log("UR5 recording started.");
    }

    public void StopRecording()
    {
        isRecording = false;
        File.WriteAllText(outputPath, csv.ToString());
        Debug.Log("UR5 recording saved to: " + outputPath);
    }

    private void AppendSample()
    {
        Vector3 position = tcpTarget != null ? tcpTarget.position : Vector3.zero;
        Quaternion rotation = tcpTarget != null ? tcpTarget.rotation : Quaternion.identity;
        bool questDeviceValid = questController != null && questController.IsDeviceValid;
        bool questClutched = questController != null && questController.IsClutched;

        csv.Append(Format(Time.time)).Append(',');
        csv.Append(questDeviceValid ? "1" : "0").Append(',');
        csv.Append(questClutched ? "1" : "0").Append(',');
        csv.Append(Format(position.x)).Append(',');
        csv.Append(Format(position.y)).Append(',');
        csv.Append(Format(position.z)).Append(',');
        csv.Append(Format(rotation.x)).Append(',');
        csv.Append(Format(rotation.y)).Append(',');
        csv.Append(Format(rotation.z)).Append(',');
        csv.Append(Format(rotation.w));

        for (int i = 0; i < 6; i++)
        {
            float target = jointController != null ? jointController.GetJointTargetDegrees(i) : 0.0f;
            csv.Append(',').Append(Format(target));
        }

        csv.AppendLine();
    }

    private string Format(float value)
    {
        return value.ToString("F6", CultureInfo.InvariantCulture);
    }
}

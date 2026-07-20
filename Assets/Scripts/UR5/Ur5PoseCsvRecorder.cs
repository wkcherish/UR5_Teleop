using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

public class Ur5PoseCsvRecorder : MonoBehaviour
{
    public Transform tcpTarget;
    public Ur5ArticulationJointController jointController;
    public Ur5TcpTargetFollower tcpFollower;
    public Ur5JointTrajectoryPlayer trajectoryPlayer;
    public Ur5GraspAssistController graspAssist;
    public Quest3TcpTargetController questController;
    public Ur5CartesianVelocityTeleopController velocityTeleop;
    public Ur5UrScriptSpeedlClient speedlClient;
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
        ResolveControlReferences();
        csv.AppendLine("time,quest_device_valid,quest_clutched,target_pos_x,target_pos_y,target_pos_z,actual_pos_x,actual_pos_y,actual_pos_z,position_error_m,rotation_error_deg,target_rot_x,target_rot_y,target_rot_z,target_rot_w,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,trajectory_pending_waypoints,last_joint_assignment_time,grasp_assist_active,velocity_active,position_clutched,rotation_clutched,fine_control_active,rotation_input_mode,rotation_joystick_valid,rotation_joystick_x,rotation_joystick_y,joystick_roll_modifier,base_vx_mps,base_vy_mps,base_vz_mps,base_wx_radps,base_wy_radps,base_wz_radps,raw_base_vx_mps,raw_base_vy_mps,raw_base_vz_mps,raw_base_wx_radps,raw_base_wy_radps,raw_base_wz_radps,workspace_limited,input_pose_valid,real_output_enabled,speedl_connected,motion_armed,sent_vx_mps,sent_vy_mps,sent_vz_mps,sent_wx_radps,sent_wy_radps,sent_wz_radps");
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
        ResolveControlReferences();
        Vector3 targetPosition = tcpTarget != null ? tcpTarget.position : Vector3.zero;
        Quaternion targetRotation = tcpTarget != null ? tcpTarget.rotation : Quaternion.identity;
        Vector3 actualPosition = tcpFollower != null ? tcpFollower.ControlPointPosition : targetPosition;
        float positionError = tcpFollower != null ? tcpFollower.PositionError : Vector3.Distance(targetPosition, actualPosition);
        float rotationError = tcpFollower != null ? tcpFollower.RotationErrorDegrees : 0.0f;
        bool questDeviceValid = questController != null && questController.IsDeviceValid;
        bool questClutched = questController != null && questController.IsClutched;
        bool velocityActive = velocityTeleop != null && velocityTeleop.IsCommandActive;
        bool positionClutched = velocityTeleop != null && velocityTeleop.IsPositionClutched;
        bool rotationClutched = velocityTeleop != null && velocityTeleop.IsRotationClutched;
        bool fineControlActive = velocityTeleop != null && velocityTeleop.IsFineControlActive;
        string rotationInputMode = velocityTeleop != null ? velocityTeleop.CurrentRotationInputMode.ToString() : string.Empty;
        bool rotationJoystickValid = velocityTeleop != null && velocityTeleop.IsRotationJoystickValid;
        Vector2 rotationJoystickInput = velocityTeleop != null ? velocityTeleop.RotationJoystickInput : Vector2.zero;
        bool joystickRollModifier = velocityTeleop != null && velocityTeleop.IsJoystickRollModifierActive;
        Vector3 baseLinearVelocity = velocityTeleop != null ? velocityTeleop.BaseLinearVelocity : Vector3.zero;
        Vector3 baseAngularVelocity = velocityTeleop != null ? velocityTeleop.BaseAngularVelocity : Vector3.zero;
        Vector3 rawBaseLinearVelocity = velocityTeleop != null ? velocityTeleop.RawBaseLinearVelocity : Vector3.zero;
        Vector3 rawBaseAngularVelocity = velocityTeleop != null ? velocityTeleop.RawBaseAngularVelocity : Vector3.zero;
        bool workspaceLimited = velocityTeleop != null && velocityTeleop.IsWorkspaceLimited;
        bool inputPoseValid = velocityTeleop != null && velocityTeleop.IsInputPoseValid;
        bool realOutputEnabled = speedlClient != null && speedlClient.enableRealRobotOutput;
        bool speedlConnected = speedlClient != null && speedlClient.IsConnected;
        bool motionArmed = speedlClient != null && speedlClient.IsMotionArmed;
        Vector3 sentLinearVelocity = speedlClient != null ? speedlClient.LastSentLinearVelocity : Vector3.zero;
        Vector3 sentAngularVelocity = speedlClient != null ? speedlClient.LastSentAngularVelocity : Vector3.zero;

        if (velocityTeleop != null)
        {
            questDeviceValid = velocityTeleop.IsDeviceValid;
            questClutched = velocityTeleop.IsCommandActive;
        }

        csv.Append(Format(Time.time)).Append(',');
        csv.Append(questDeviceValid ? "1" : "0").Append(',');
        csv.Append(questClutched ? "1" : "0").Append(',');
        csv.Append(Format(targetPosition.x)).Append(',');
        csv.Append(Format(targetPosition.y)).Append(',');
        csv.Append(Format(targetPosition.z)).Append(',');
        csv.Append(Format(actualPosition.x)).Append(',');
        csv.Append(Format(actualPosition.y)).Append(',');
        csv.Append(Format(actualPosition.z)).Append(',');
        csv.Append(Format(positionError)).Append(',');
        csv.Append(Format(rotationError)).Append(',');
        csv.Append(Format(targetRotation.x)).Append(',');
        csv.Append(Format(targetRotation.y)).Append(',');
        csv.Append(Format(targetRotation.z)).Append(',');
        csv.Append(Format(targetRotation.w));

        for (int i = 0; i < 6; i++)
        {
            float target = jointController != null ? jointController.GetJointTargetDegrees(i) : 0.0f;
            csv.Append(',').Append(Format(target));
        }

        csv.Append(',').Append(trajectoryPlayer != null ? trajectoryPlayer.PendingWaypointCount.ToString(CultureInfo.InvariantCulture) : "0");
        csv.Append(',').Append(Format(trajectoryPlayer != null ? trajectoryPlayer.LastAssignmentTime : 0.0f));
        csv.Append(',').Append(graspAssist != null && graspAssist.IsAssistActive ? "1" : "0");
        csv.Append(',').Append(velocityActive ? "1" : "0");
        csv.Append(',').Append(positionClutched ? "1" : "0");
        csv.Append(',').Append(rotationClutched ? "1" : "0");
        csv.Append(',').Append(fineControlActive ? "1" : "0");
        csv.Append(',').Append(rotationInputMode);
        csv.Append(',').Append(rotationJoystickValid ? "1" : "0");
        csv.Append(',').Append(Format(rotationJoystickInput.x));
        csv.Append(',').Append(Format(rotationJoystickInput.y));
        csv.Append(',').Append(joystickRollModifier ? "1" : "0");
        csv.Append(',').Append(Format(baseLinearVelocity.x));
        csv.Append(',').Append(Format(baseLinearVelocity.y));
        csv.Append(',').Append(Format(baseLinearVelocity.z));
        csv.Append(',').Append(Format(baseAngularVelocity.x));
        csv.Append(',').Append(Format(baseAngularVelocity.y));
        csv.Append(',').Append(Format(baseAngularVelocity.z));
        csv.Append(',').Append(Format(rawBaseLinearVelocity.x));
        csv.Append(',').Append(Format(rawBaseLinearVelocity.y));
        csv.Append(',').Append(Format(rawBaseLinearVelocity.z));
        csv.Append(',').Append(Format(rawBaseAngularVelocity.x));
        csv.Append(',').Append(Format(rawBaseAngularVelocity.y));
        csv.Append(',').Append(Format(rawBaseAngularVelocity.z));
        csv.Append(',').Append(workspaceLimited ? "1" : "0");
        csv.Append(',').Append(inputPoseValid ? "1" : "0");
        csv.Append(',').Append(realOutputEnabled ? "1" : "0");
        csv.Append(',').Append(speedlConnected ? "1" : "0");
        csv.Append(',').Append(motionArmed ? "1" : "0");
        csv.Append(',').Append(Format(sentLinearVelocity.x));
        csv.Append(',').Append(Format(sentLinearVelocity.y));
        csv.Append(',').Append(Format(sentLinearVelocity.z));
        csv.Append(',').Append(Format(sentAngularVelocity.x));
        csv.Append(',').Append(Format(sentAngularVelocity.y));
        csv.Append(',').Append(Format(sentAngularVelocity.z));

        csv.AppendLine();
    }

    private void ResolveControlReferences()
    {
        if (tcpFollower == null && jointController != null)
        {
            tcpFollower = jointController.GetComponent<Ur5TcpTargetFollower>();
        }

        if (tcpFollower == null)
        {
            tcpFollower = FindObjectOfType<Ur5TcpTargetFollower>();
        }

        if (velocityTeleop == null)
        {
            velocityTeleop = FindObjectOfType<Ur5CartesianVelocityTeleopController>();
        }

        if (trajectoryPlayer == null)
        {
            trajectoryPlayer = FindObjectOfType<Ur5JointTrajectoryPlayer>();
        }

        if (graspAssist == null)
        {
            graspAssist = FindObjectOfType<Ur5GraspAssistController>();
        }

        if (speedlClient == null)
        {
            speedlClient = FindObjectOfType<Ur5UrScriptSpeedlClient>();
        }
    }

    private string Format(float value)
    {
        return value.ToString("F6", CultureInfo.InvariantCulture);
    }
}

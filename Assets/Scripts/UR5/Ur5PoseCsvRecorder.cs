using System;
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
    [Tooltip("Quest Android 运行时自动开始记录；Editor 不会自动开启。")]
    public bool autoRecordOnAndroid = true;
    public float sampleInterval = 0.02f;
    [Tooltip("录制期间覆盖保存完整 CSV 快照的间隔，单位秒。")]
    public float flushIntervalSeconds = 1.0f;

    private StringBuilder csv;
    private float nextSampleTime;
    private float nextFlushTime;
    private bool isRecording;
    private string outputPath;
    private bool hasLoggedSaveFailure;

    private void Start()
    {
        csv = new StringBuilder();
        ResolveControlReferences();
        csv.Append("time,quest_device_valid,quest_clutched,target_pos_x,target_pos_y,target_pos_z,actual_pos_x,actual_pos_y,actual_pos_z,position_error_m,rotation_error_deg,target_rot_x,target_rot_y,target_rot_z,target_rot_w,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,trajectory_pending_waypoints,last_joint_assignment_time,grasp_assist_active,velocity_active,position_clutched,rotation_clutched,fine_control_active,rotation_input_mode,rotation_joystick_valid,rotation_joystick_x,rotation_joystick_y,joystick_roll_modifier,base_vx_mps,base_vy_mps,base_vz_mps,base_wx_radps,base_wy_radps,base_wz_radps,raw_base_vx_mps,raw_base_vy_mps,raw_base_vz_mps,raw_base_wx_radps,raw_base_wy_radps,raw_base_wz_radps,workspace_limited,input_pose_valid,real_output_enabled,speedl_connected,motion_armed,sent_vx_mps,sent_vy_mps,sent_vz_mps,sent_wx_radps,sent_wy_radps,sent_wz_radps,continuous_6dof_enabled,continuous_state,continuous_fault,controller_distance_m,controller_angle_deg,translation_gain,rotation_gain,logical_to_filtered_rotation_deg");
        csv.AppendLine(",raw_hand_pos_x,raw_hand_pos_y,raw_hand_pos_z,raw_hand_rot_x,raw_hand_rot_y,raw_hand_rot_z,raw_hand_rot_w,logical_pos_x,logical_pos_y,logical_pos_z,logical_rot_x,logical_rot_y,logical_rot_z,logical_rot_w,constrained_pos_x,constrained_pos_y,constrained_pos_z,filtered_pos_x,filtered_pos_y,filtered_pos_z,filtered_rot_x,filtered_rot_y,filtered_rot_z,filtered_rot_w,actual_grasp_rot_x,actual_grasp_rot_y,actual_grasp_rot_z,actual_grasp_rot_w,target_stationary_s,settled_hold,dls_min_pivot,near_singularity,ik_failure_count,ik_lead_limited,j1_drive_deg,j2_drive_deg,j3_drive_deg,j4_drive_deg,j5_drive_deg,j6_drive_deg,j1_measured_deg,j2_measured_deg,j3_measured_deg,j4_measured_deg,j5_measured_deg,j6_measured_deg,ur10_rotation_adjust_active,position_orientation_locked,controller_position_gate_holding");
        outputPath = Path.Combine(Application.persistentDataPath, "ur5_pose_log.csv");

        if (recordOnStart || (autoRecordOnAndroid && ShouldAutoStartRecording(Application.platform)))
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
        if (Time.unscaledTime >= nextFlushTime)
        {
            SaveRecordingSnapshot();
            nextFlushTime = Time.unscaledTime + Mathf.Max(0.1f, flushIntervalSeconds);
        }
    }

    public void StartRecording()
    {
        isRecording = true;
        nextSampleTime = Time.time;
        nextFlushTime = Time.unscaledTime + Mathf.Max(0.1f, flushIntervalSeconds);
        Debug.Log("UR5 recording started.");
    }

    public void StopRecording()
    {
        SaveRecordingSnapshot();
        isRecording = false;
    }

    public static bool ShouldAutoStartRecording(RuntimePlatform platform)
    {
        return platform == RuntimePlatform.Android;
    }

    public void SaveRecordingSnapshot()
    {
        if (csv == null || csv.Length == 0 || string.IsNullOrEmpty(outputPath))
        {
            return;
        }

        try
        {
            // 每次覆盖完整快照，不追加表头；生命周期回调重复触发也保持幂等。
            File.WriteAllText(outputPath, csv.ToString());
            hasLoggedSaveFailure = false;
            Debug.Log("UR5 recording saved to: " + outputPath);
        }
        catch (IOException exception)
        {
            LogSaveFailureOnce(exception);
        }
        catch (UnauthorizedAccessException exception)
        {
            LogSaveFailureOnce(exception);
        }
    }

    private void OnApplicationPause(bool paused)
    {
        if (paused)
        {
            SaveRecordingSnapshot();
        }
    }

    private void OnApplicationQuit()
    {
        SaveRecordingSnapshot();
    }

    private void OnDisable()
    {
        SaveRecordingSnapshot();
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
        bool continuous6DofEnabled = velocityTeleop != null && velocityTeleop.EnableContinuous6DofClutch;
        string continuousState = velocityTeleop != null ? velocityTeleop.TeleopControllerState.ToString() : string.Empty;
        string continuousFault = velocityTeleop != null ? velocityTeleop.Continuous6DofFaultReason.ToString() : string.Empty;
        float continuousDistanceMeters = velocityTeleop != null ? velocityTeleop.ContinuousControllerDistanceMeters : 0.0f;
        float continuousAngleDegrees = velocityTeleop != null ? velocityTeleop.ContinuousControllerAngleDegrees : 0.0f;
        float continuousTranslationGain = velocityTeleop != null ? velocityTeleop.ContinuousTranslationGain : 0.0f;
        float continuousRotationGain = velocityTeleop != null ? velocityTeleop.ContinuousRotationGain : 0.0f;
        float logicalToFilteredRotationDegrees = velocityTeleop != null
            ? Quaternion.Angle(velocityTeleop.LogicalCommandRotation, velocityTeleop.FilteredCommandRotation)
            : 0.0f;
        Vector3 rawHandPosition = velocityTeleop != null
            ? velocityTeleop.RawControllerPositionWorld
            : Vector3.zero;
        Quaternion rawHandRotation = velocityTeleop != null
            ? velocityTeleop.RawControllerRotationWorld
            : default(Quaternion);
        Vector3 logicalPosition = velocityTeleop != null
            ? velocityTeleop.LogicalCommandPosition
            : Vector3.zero;
        Quaternion logicalRotation = velocityTeleop != null
            ? velocityTeleop.LogicalCommandRotation
            : default(Quaternion);
        Vector3 constrainedPosition = velocityTeleop != null
            ? velocityTeleop.ConstrainedCommandPosition
            : Vector3.zero;
        Vector3 filteredPosition = velocityTeleop != null
            ? velocityTeleop.FilteredCommandPosition
            : Vector3.zero;
        Quaternion filteredRotation = velocityTeleop != null
            ? velocityTeleop.FilteredCommandRotation
            : default(Quaternion);
        Quaternion actualGraspRotation = tcpFollower != null
            ? tcpFollower.ActualGraspRotation
            : default(Quaternion);
        float targetStationarySeconds = tcpFollower != null
            ? tcpFollower.TargetStationarySeconds
            : 0.0f;
        bool settledHold = tcpFollower != null && tcpFollower.IsSettledTargetHoldActive;
        float dlsMinimumPivot = tcpFollower != null ? tcpFollower.LastDlsMinimumPivot : 0.0f;
        bool nearSingularity = tcpFollower != null && tcpFollower.IsNearSingularity;
        int ikFailureCount = tcpFollower != null ? tcpFollower.IkFailureCount : 0;
        bool ikLeadLimited = tcpFollower != null && tcpFollower.WasIkCommandLeadLimited;
        bool ur10RotationAdjustActive = velocityTeleop != null && velocityTeleop.IsUr10StyleRotationAdjustActive;
        bool positionOrientationLocked = velocityTeleop != null && velocityTeleop.IsPositionOrientationLocked;
        bool controllerPositionGateHolding = velocityTeleop != null && velocityTeleop.IsControllerPositionNoiseGateHolding;

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
        // 新增列只能追加在末尾，避免旧日志分析脚本按前缀 schema 读取时发生错位。
        csv.Append(',').Append(continuous6DofEnabled ? "1" : "0");
        csv.Append(',').Append(continuousState);
        csv.Append(',').Append(continuousFault);
        csv.Append(',').Append(Format(continuousDistanceMeters));
        csv.Append(',').Append(Format(continuousAngleDegrees));
        csv.Append(',').Append(Format(continuousTranslationGain));
        csv.Append(',').Append(Format(continuousRotationGain));
        csv.Append(',').Append(Format(logicalToFilteredRotationDegrees));

        // 以下字段按“输入 -> 命令 -> IK -> 物理关节”顺序记录，便于定位静止摆动最早出现在哪一层。
        AppendVector3(rawHandPosition);
        AppendQuaternion(rawHandRotation);
        AppendVector3(logicalPosition);
        AppendQuaternion(logicalRotation);
        AppendVector3(constrainedPosition);
        AppendVector3(filteredPosition);
        AppendQuaternion(filteredRotation);
        AppendQuaternion(actualGraspRotation);
        csv.Append(',').Append(Format(targetStationarySeconds));
        csv.Append(',').Append(settledHold ? "1" : "0");
        csv.Append(',').Append(Format(dlsMinimumPivot));
        csv.Append(',').Append(nearSingularity ? "1" : "0");
        csv.Append(',').Append(ikFailureCount.ToString(CultureInfo.InvariantCulture));
        csv.Append(',').Append(ikLeadLimited ? "1" : "0");

        for (int i = 0; i < 6; i++)
        {
            float driveTarget = jointController != null
                ? jointController.GetDriveTargetDegrees(i)
                : 0.0f;
            csv.Append(',').Append(Format(driveTarget));
        }

        for (int i = 0; i < 6; i++)
        {
            float measuredJoint = jointController != null
                ? jointController.GetMeasuredJointDegrees(i)
                : 0.0f;
            csv.Append(',').Append(Format(measuredJoint));
        }

        csv.Append(',').Append(ur10RotationAdjustActive ? "1" : "0");
        csv.Append(',').Append(positionOrientationLocked ? "1" : "0");
        csv.Append(',').Append(controllerPositionGateHolding ? "1" : "0");

        csv.AppendLine();
    }

    private void AppendVector3(Vector3 value)
    {
        csv.Append(',').Append(Format(value.x));
        csv.Append(',').Append(Format(value.y));
        csv.Append(',').Append(Format(value.z));
    }

    private void AppendQuaternion(Quaternion value)
    {
        csv.Append(',').Append(Format(value.x));
        csv.Append(',').Append(Format(value.y));
        csv.Append(',').Append(Format(value.z));
        csv.Append(',').Append(Format(value.w));
    }

    private void LogSaveFailureOnce(Exception exception)
    {
        if (hasLoggedSaveFailure)
        {
            return;
        }

        hasLoggedSaveFailure = true;
        Debug.LogError("UR5 recording save failed: " + exception.Message);
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

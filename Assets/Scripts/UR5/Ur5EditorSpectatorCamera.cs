using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class Ur5EditorSpectatorCamera : MonoBehaviour
{
    [Header("References")]
    public Transform robotRoot;
    public Transform xrOrigin;
    public Transform cameraTransform;
    public Transform tcpTarget;

    [Header("Spectator View")]
    public bool enableWhenNoHmd = true;
    public Vector3 viewOffset = new Vector3(-1.3f, 0.9f, -1.6f);
    public Vector3 lookAtOffset = new Vector3(0.0f, 0.45f, 0.0f);

    [Header("Debug")]
    public bool logStatus = true;

    private bool hasAppliedView;

    private void Start()
    {
        TryApplySpectatorView();
    }

    private void LateUpdate()
    {
        if (!hasAppliedView)
        {
            TryApplySpectatorView();
        }
    }

    private void TryApplySpectatorView()
    {
        ResolveReferences();

        if (!enableWhenNoHmd || robotRoot == null || xrOrigin == null || cameraTransform == null)
        {
            return;
        }

        if (HasActiveHmd())
        {
            return;
        }

        Vector3 focusPoint = GetFocusPoint();
        Vector3 desiredCameraPosition = robotRoot.position + viewOffset;
        Vector3 originDelta = desiredCameraPosition - cameraTransform.position;
        xrOrigin.position += originDelta;

        Vector3 lookDirection = focusPoint - cameraTransform.position;
        if (lookDirection.sqrMagnitude > 0.0001f)
        {
            xrOrigin.rotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
        }

        hasAppliedView = true;

        if (logStatus)
        {
            Debug.Log("UR5 spectator camera aligned for editor view.");
        }
    }

    private void ResolveReferences()
    {
        if (robotRoot == null)
        {
            robotRoot = FindNamedTransform("ur5_robot")
                ?? FindNamedTransform("ur5")
                ?? FindNamedTransform("UR5")
                ?? FindNamedTransform("base_link");
        }

        if (xrOrigin == null)
        {
            xrOrigin = FindNamedTransform("XR Origin (VR)");
        }

        if (cameraTransform == null)
        {
            Camera mainCamera = Camera.main;
            if (mainCamera != null)
            {
                cameraTransform = mainCamera.transform;
            }
        }

        if (tcpTarget == null)
        {
            tcpTarget = FindNamedTransform("TcpTarget");
        }
    }

    private Vector3 GetFocusPoint()
    {
        if (tcpTarget != null)
        {
            return tcpTarget.position;
        }

        return robotRoot.position + lookAtOffset;
    }

    private bool HasActiveHmd()
    {
        List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.Head, devices);
        foreach (InputDevice device in devices)
        {
            if (device.isValid)
            {
                return true;
            }
        }

        return false;
    }

    private Transform FindNamedTransform(string objectName)
    {
        GameObject found = GameObject.Find(objectName);
        return found != null ? found.transform : null;
    }
}

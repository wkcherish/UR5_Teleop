using UnityEngine;

public class TcpTargetKeyboardController : MonoBehaviour
{
    [Header("Move")]
    public bool requireClutch = false;
    public KeyCode clutchKey = KeyCode.Space;
    public float moveSpeed = 0.10f;
    public float fastMultiplier = 3.0f;
    public bool moveRelativeToCamera = true;
    public bool flattenCameraForward = true;

    [Header("Rotate")]
    public float rotateSpeed = 45.0f;

    private void Update()
    {
        if (requireClutch && !Input.GetKey(clutchKey))
        {
            return;
        }

        float speed = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)
            ? moveSpeed * fastMultiplier
            : moveSpeed;

        Transform referenceCamera = Camera.main != null ? Camera.main.transform : null;
        Vector3 forward = Vector3.forward;
        Vector3 right = Vector3.right;

        if (moveRelativeToCamera && referenceCamera != null)
        {
            forward = referenceCamera.forward;
            right = referenceCamera.right;

            if (flattenCameraForward)
            {
                forward = Vector3.ProjectOnPlane(forward, Vector3.up);
                right = Vector3.ProjectOnPlane(right, Vector3.up);
            }

            if (forward.sqrMagnitude > 0.0001f)
            {
                forward.Normalize();
            }

            if (right.sqrMagnitude > 0.0001f)
            {
                right.Normalize();
            }
        }

        Vector3 move = Vector3.zero;

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) move += forward;
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) move -= forward;
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) move -= right;
        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) move += right;
        if (Input.GetKey(KeyCode.E)) move += Vector3.up;
        if (Input.GetKey(KeyCode.Q)) move += Vector3.down;

        if (move.sqrMagnitude > 0.0001f)
        {
            transform.position += move.normalized * speed * Time.deltaTime;
        }

        if (Input.GetKey(KeyCode.Z))
        {
            transform.Rotate(Vector3.up, -rotateSpeed * Time.deltaTime, Space.World);
        }

        if (Input.GetKey(KeyCode.C))
        {
            transform.Rotate(Vector3.up, rotateSpeed * Time.deltaTime, Space.World);
        }
    }
}

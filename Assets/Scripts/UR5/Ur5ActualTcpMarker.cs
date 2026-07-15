using UnityEngine;

[DefaultExecutionOrder(200)]
public class Ur5ActualTcpMarker : MonoBehaviour
{
    public Ur5TcpTargetFollower follower;
    public float markerDiameter = 0.025f;
    public Color markerColor = Color.green;

    private Transform marker;

    private void Awake()
    {
        CreateMarker();
    }

    private void LateUpdate()
    {
        if (follower == null)
        {
            follower = FindObjectOfType<Ur5TcpTargetFollower>();
        }

        if (follower != null && marker != null)
        {
            marker.position = follower.ControlPointPosition;
        }
    }

    private void CreateMarker()
    {
        Transform existing = transform.Find("ActualTcpMarker");
        if (existing != null)
        {
            marker = existing;
            return;
        }

        GameObject markerObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        markerObject.name = "ActualTcpMarker";
        markerObject.transform.SetParent(transform, false);
        markerObject.transform.localScale = Vector3.one * markerDiameter;

        Collider markerCollider = markerObject.GetComponent<Collider>();
        if (markerCollider != null)
        {
            Destroy(markerCollider);
        }

        Renderer renderer = markerObject.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = markerColor;
        }

        marker = markerObject.transform;
    }
}

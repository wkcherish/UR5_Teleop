using UnityEngine;

[DefaultExecutionOrder(200)]
public class Ur5ActualTcpMarker : MonoBehaviour
{
    public Ur5TcpTargetFollower follower;
    [Tooltip("Hide the command target sphere. TcpTarget is an IK command, while ActualTcp is the physical two-pad midpoint.")]
    public bool hideCommandTargetRenderer = true;
    public float markerDiameter = 0.035f;
    public Color markerColor = Color.green;

    private Transform marker;
    private bool ownsMarker;

    private void Awake()
    {
        SetCommandTargetRendererVisible(!hideCommandTargetRenderer);
        CreateMarker();
    }

    private void OnEnable()
    {
        SetCommandTargetRendererVisible(!hideCommandTargetRenderer);
        if (marker != null)
        {
            marker.gameObject.SetActive(true);
        }
    }

    private void OnDisable()
    {
        if (marker != null)
        {
            marker.gameObject.SetActive(false);
        }
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
            marker.localScale = Vector3.one * markerDiameter;
        }
    }

    private void OnDestroy()
    {
        if (ownsMarker && marker != null)
        {
            Destroy(marker.gameObject);
        }
    }

    private void CreateMarker()
    {
        // Older scenes created this marker as a child of TcpTarget. Detach it:
        // otherwise the hierarchy incorrectly suggests that the actual TCP is
        // an offset from the commanded target.
        Transform existing = transform.Find("ActualTcpMarker");
        if (existing != null)
        {
            marker = existing;
            marker.SetParent(null, true);
            return;
        }

        GameObject markerObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        markerObject.name = "ActualTcp";
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
        ownsMarker = true;
    }

    private void SetCommandTargetRendererVisible(bool visible)
    {
        Renderer commandRenderer = GetComponent<Renderer>();
        if (commandRenderer != null)
        {
            commandRenderer.enabled = visible;
        }
    }
}

using UnityEngine;
using UnityEngine.XR;

[DefaultExecutionOrder(150)]
public class Quest3ControllerVisualizer : MonoBehaviour
{
    [Header("XR Reference")]
    public Transform xrOrigin;
    public bool showLeftController = true;
    public bool showRightController = true;
    [Tooltip("关闭后隐藏 Unity 生成的虚拟手柄；Passthrough 模式下可直接观察真实手柄。")]
    public bool renderVirtualControllers = true;
    public bool useOfficialTouchPlusModels = true;
    public float officialTouchPlusModelScale = 0.01f;

    [Header("Appearance")]
    public Vector3 controllerSize = new Vector3(0.045f, 0.035f, 0.11f);
    public Color leftControllerColor = new Color(0.1f, 0.65f, 0.72f);
    public Color rightControllerColor = new Color(0.95f, 0.45f, 0.12f);

    private InputDevice leftDevice;
    private InputDevice rightDevice;
    private ControllerVisual leftVisual;
    private ControllerVisual rightVisual;

    private void Awake()
    {
        if (xrOrigin == null)
        {
            GameObject foundOrigin = GameObject.Find("XR Origin (VR)");
            if (foundOrigin != null)
            {
                xrOrigin = foundOrigin.transform;
            }
        }

        if (showLeftController)
        {
            leftVisual = CreateVisual("QuestLeftController", leftControllerColor, true);
        }

        if (showRightController)
        {
            rightVisual = CreateVisual("QuestRightController", rightControllerColor, false);
        }
    }

    private void LateUpdate()
    {
        if (!renderVirtualControllers)
        {
            SetVisualActive(leftVisual, false);
            SetVisualActive(rightVisual, false);
            return;
        }

        UpdateVisual(XRNode.LeftHand, ref leftDevice, leftVisual);
        UpdateVisual(XRNode.RightHand, ref rightDevice, rightVisual);
    }

    /// <summary>
    /// 显示或隐藏 Unity 中的手柄模型，不影响 Quest 控制器输入和数据采集。
    /// </summary>
    public void SetVirtualControllerVisibility(bool visible)
    {
        renderVirtualControllers = visible;
        if (!visible)
        {
            SetVisualActive(leftVisual, false);
            SetVisualActive(rightVisual, false);
        }
    }

    private void UpdateVisual(XRNode node, ref InputDevice device, ControllerVisual visual)
    {
        if (visual == null)
        {
            return;
        }

        if (!device.isValid)
        {
            device = InputDevices.GetDeviceAtXRNode(node);
        }

        Vector3 localPosition = Vector3.zero;
        Quaternion localRotation = Quaternion.identity;
        bool hasPosition = device.isValid
            && device.TryGetFeatureValue(CommonUsages.devicePosition, out localPosition);
        bool hasRotation = device.isValid
            && device.TryGetFeatureValue(CommonUsages.deviceRotation, out localRotation);

        visual.root.gameObject.SetActive(hasPosition && hasRotation);
        if (!hasPosition || !hasRotation)
        {
            return;
        }

        if (xrOrigin != null)
        {
            visual.root.SetPositionAndRotation(
                xrOrigin.TransformPoint(localPosition),
                xrOrigin.rotation * localRotation);
        }
        else
        {
            visual.root.SetPositionAndRotation(localPosition, localRotation);
        }

        float triggerAmount = 0.0f;
        device.TryGetFeatureValue(CommonUsages.trigger, out triggerAmount);
        if (visual.trigger != null)
        {
            visual.trigger.localPosition = new Vector3(0.0f, -0.013f, 0.042f - triggerAmount * 0.018f);
        }
    }

    private void SetVisualActive(ControllerVisual visual, bool active)
    {
        if (visual != null && visual.root != null)
        {
            visual.root.gameObject.SetActive(active);
        }
    }

    private ControllerVisual CreateVisual(string objectName, Color bodyColor, bool isLeftHand)
    {
        GameObject rootObject = new GameObject(objectName);
        rootObject.transform.SetParent(transform, false);

        if (useOfficialTouchPlusModels)
        {
            string resourcePath = isLeftHand
                ? "MetaXR/TouchPlus/MetaQuestTouchPlus_Left"
                : "MetaXR/TouchPlus/MetaQuestTouchPlus_Right";
            GameObject officialModel = Resources.Load<GameObject>(resourcePath);
            if (officialModel != null)
            {
                GameObject modelInstance = Instantiate(officialModel, rootObject.transform);
                modelInstance.name = "MetaQuestTouchPlusModel";
                modelInstance.transform.localPosition = Vector3.zero;
                modelInstance.transform.localRotation = Quaternion.identity;
                modelInstance.transform.localScale = Vector3.one * officialTouchPlusModelScale;
                return new ControllerVisual(rootObject.transform, null);
            }
        }

        Transform body = CreatePrimitiveChild(rootObject.transform, "Body", PrimitiveType.Cube, bodyColor);
        body.localScale = controllerSize;

        Transform trigger = CreatePrimitiveChild(rootObject.transform, "Trigger", PrimitiveType.Cube, Color.white);
        trigger.localScale = new Vector3(0.027f, 0.012f, 0.03f);

        Transform grip = CreatePrimitiveChild(rootObject.transform, "Grip", PrimitiveType.Capsule, bodyColor * 0.7f);
        grip.localPosition = new Vector3(0.0f, -0.055f, -0.01f);
        grip.localRotation = Quaternion.Euler(10.0f, 0.0f, 0.0f);
        grip.localScale = new Vector3(0.032f, 0.05f, 0.032f);

        return new ControllerVisual(rootObject.transform, trigger);
    }

    private Transform CreatePrimitiveChild(Transform parent, string objectName, PrimitiveType primitiveType, Color color)
    {
        GameObject child = GameObject.CreatePrimitive(primitiveType);
        child.name = objectName;
        child.transform.SetParent(parent, false);

        Collider collider = child.GetComponent<Collider>();
        if (collider != null)
        {
            Destroy(collider);
        }

        Renderer renderer = child.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = color;
        }

        return child.transform;
    }

    private sealed class ControllerVisual
    {
        public readonly Transform root;
        public readonly Transform trigger;

        public ControllerVisual(Transform root, Transform trigger)
        {
            this.root = root;
            this.trigger = trigger;
        }
    }
}

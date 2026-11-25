using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;

/// <summary>
/// Runtime Quest-side editor for the PC UDP receiver endpoint.
/// This component only changes Quest telemetry destination fields; it never
/// opens URScript/RTDE connections and never enables real robot output.
/// </summary>
[DefaultExecutionOrder(-70)]
public class Quest3UdpEndpointRuntimePanel : MonoBehaviour
{
    public enum EndpointShortcutAction
    {
        None,
        ResetEndpoint,
        ConfirmEditedEndpoint,
        ContinueWithSavedEndpoint
    }

    public const string ReceiverHostPrefsKey = "QuestUr5_UdpReceiverHost";
    public const string ReceiverPortPrefsKey = "QuestUr5_UdpReceiverPort";

    private const string DefaultReceiverHost = "192.168.1.100";
    private const int DefaultReceiverPort = 8080;
    private const int Ipv4OctetCount = 4;
    private const int DigitsPerOctet = 3;
    private const int Ipv4DigitCount = Ipv4OctetCount * DigitsPerOctet;
    private const float ControllerRepeatSeconds = 0.16f;
    private const float StickDeadZone = 0.65f;

    [Header("Endpoint")]
    public Quest3UdpTeleopSender sender;
    public string fallbackReceiverHost = "";
    public int fallbackReceiverPort = DefaultReceiverPort;
    public float fallbackSendRateHz = 100.0f;

    [Header("Runtime UI")]
    public bool createUiOnStart = true;
    public bool showOnStart = true;
    public bool hideAfterApply = true;
    public bool useControllerShortcuts = true;
    public float panelDistanceMeters = 1.35f;
    public Vector2 panelSizePixels = new Vector2(980.0f, 560.0f);

    private readonly int[] hostOctets = new int[Ipv4OctetCount];
    private readonly Text[] digitTexts = new Text[Ipv4DigitCount];
    private readonly Image[] digitBackgrounds = new Image[Ipv4DigitCount];

    private GameObject panelRoot;
    private Text activeEndpointText;
    private Text statusText;
    private Text portText;
    private Button continueButton;
    private Button resetButton;
    private int currentPort = DefaultReceiverPort;
    private string savedOrInitialHost = DefaultReceiverHost;
    private int savedOrInitialPort = DefaultReceiverPort;
    private int selectedDigitIndex;
    private float nextControllerRepeatTime;
    private float nextStatusRefreshTime;
    private InputDevice rightController;
    private InputDevice leftController;
    private bool previousRightSecondary;
    private bool previousLeftPrimary;
    private bool previousLeftSecondary;
    private bool previousLeftMenu;
    private bool editingEndpoint;

    public static bool HasSavedReceiverHost()
    {
        return IsValidIpv4Address(PlayerPrefs.GetString(ReceiverHostPrefsKey, ""));
    }

    public static bool IsValidIpv4Address(string host)
    {
        int[] octets = new int[Ipv4OctetCount];
        return TryParseIpv4Octets(host, octets);
    }

    public static bool TryParseIpv4Octets(string host, int[] octets)
    {
        if (octets == null || octets.Length < Ipv4OctetCount || string.IsNullOrWhiteSpace(host))
        {
            return false;
        }

        string[] parts = host.Trim().Split('.');
        if (parts.Length != Ipv4OctetCount)
        {
            return false;
        }

        int[] parsed = new int[Ipv4OctetCount];
        for (int i = 0; i < Ipv4OctetCount; i++)
        {
            string part = parts[i];
            if (part.Length < 1 || part.Length > 3)
            {
                return false;
            }

            int value;
            if (!int.TryParse(part, out value) || value < 0 || value > 255)
            {
                return false;
            }

            parsed[i] = value;
        }

        for (int i = 0; i < Ipv4OctetCount; i++)
        {
            octets[i] = parsed[i];
        }

        return true;
    }

    public static string FormatIpv4Octets(int[] octets)
    {
        if (octets == null || octets.Length < Ipv4OctetCount)
        {
            return DefaultReceiverHost;
        }

        return ClampOctet(octets[0])
            + "."
            + ClampOctet(octets[1])
            + "."
            + ClampOctet(octets[2])
            + "."
            + ClampOctet(octets[3]);
    }

    public static int AdjustIpv4Digit(int octetValue, int digitOffset, int delta)
    {
        int place = digitOffset == 0 ? 100 : digitOffset == 1 ? 10 : 1;
        return ClampOctet(octetValue + delta * place);
    }

    public static string ResolveInitialHost(string fallbackHost)
    {
        string savedHost = PlayerPrefs.GetString(ReceiverHostPrefsKey, "");
        if (IsValidIpv4Address(savedHost))
        {
            return savedHost.Trim();
        }

        if (IsValidIpv4Address(fallbackHost))
        {
            return fallbackHost.Trim();
        }

        return DefaultReceiverHost;
    }

    public static int ResolveInitialPort(int fallbackPort)
    {
        int savedPort = PlayerPrefs.GetInt(ReceiverPortPrefsKey, 0);
        if (savedPort >= 1 && savedPort <= 65535)
        {
            return savedPort;
        }

        if (fallbackPort >= 1 && fallbackPort <= 65535)
        {
            return fallbackPort;
        }

        return DefaultReceiverPort;
    }

    public static EndpointShortcutAction ResolveEndpointShortcutAction(
        bool leftXPressed,
        bool leftYPressed,
        bool rightBPressed)
    {
        if (leftXPressed)
        {
            return EndpointShortcutAction.ResetEndpoint;
        }

        if (rightBPressed)
        {
            return EndpointShortcutAction.ConfirmEditedEndpoint;
        }

        if (leftYPressed)
        {
            return EndpointShortcutAction.ContinueWithSavedEndpoint;
        }

        return EndpointShortcutAction.None;
    }

    private void Start()
    {
        ResolveSender();
        string initialHost = ResolveInitialHost(GetFallbackHost());
        currentPort = ResolveInitialPort(GetFallbackPort());
        savedOrInitialHost = initialHost;
        savedOrInitialPort = currentPort;
        SetHostOctets(initialHost);
        editingEndpoint = !HasSavedReceiverHost();

        ApplyEndpoint(initialHost, currentPort, false, false);

        if (createUiOnStart)
        {
            BuildPanel();
            RefreshEndpointDisplay();
            SetPanelVisible(showOnStart);
            RefreshStatusText(editingEndpoint ? "请输入IP" : "已恢复上次IP");
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F1))
        {
            SetPanelVisible(panelRoot == null || !panelRoot.activeSelf);
        }

        if (panelRoot != null && panelRoot.activeSelf)
        {
            HandleKeyboardShortcuts();
        }

        if (useControllerShortcuts)
        {
            HandleControllerShortcuts();
        }

        if (panelRoot != null && panelRoot.activeSelf && Time.unscaledTime >= nextStatusRefreshTime)
        {
            nextStatusRefreshTime = Time.unscaledTime + 0.25f;
            RefreshStatusText(null);
        }
    }

    public void Configure(
        Quest3UdpTeleopSender configuredSender,
        string configuredFallbackHost,
        int configuredFallbackPort,
        float configuredSendRateHz)
    {
        sender = configuredSender;
        fallbackReceiverHost = configuredFallbackHost;
        fallbackReceiverPort = configuredFallbackPort;
        fallbackSendRateHz = configuredSendRateHz;
    }

    public bool ApplyEndpoint(string host, int port, bool save, bool hidePanel)
    {
        string normalizedHost;
        int normalizedPort;
        string error;
        if (!TryNormalizeEndpoint(host, port, out normalizedHost, out normalizedPort, out error))
        {
            RefreshStatusText(error);
            return false;
        }

        ResolveSender();
        if (sender == null)
        {
            RefreshStatusText("No sender");
            return false;
        }

        sender.receiverHost = normalizedHost;
        sender.receiverPort = normalizedPort;
        sender.sendRateHz = Mathf.Clamp(fallbackSendRateHz, 1.0f, 120.0f);
        sender.sendPackets = true;
        currentPort = normalizedPort;
        SetHostOctets(normalizedHost);

        if (save)
        {
            PlayerPrefs.SetString(ReceiverHostPrefsKey, normalizedHost);
            PlayerPrefs.SetInt(ReceiverPortPrefsKey, normalizedPort);
            PlayerPrefs.Save();
            editingEndpoint = false;
            savedOrInitialHost = normalizedHost;
            savedOrInitialPort = normalizedPort;
        }

        RefreshEndpointDisplay();
        RefreshStatusText(save ? "Saved" : "Active");
        if (hidePanel)
        {
            SetPanelVisible(false);
        }

        return true;
    }

    private string GetFallbackHost()
    {
        if (sender != null && IsValidIpv4Address(sender.receiverHost))
        {
            return sender.receiverHost;
        }

        return fallbackReceiverHost;
    }

    private int GetFallbackPort()
    {
        if (sender != null && sender.receiverPort >= 1 && sender.receiverPort <= 65535)
        {
            return sender.receiverPort;
        }

        return fallbackReceiverPort;
    }

    private void ResolveSender()
    {
        if (sender == null)
        {
            sender = GetComponent<Quest3UdpTeleopSender>();
        }

        if (sender == null)
        {
            sender = gameObject.AddComponent<Quest3UdpTeleopSender>();
        }
    }

    private static bool TryNormalizeEndpoint(
        string host,
        int port,
        out string normalizedHost,
        out int normalizedPort,
        out string error)
    {
        int[] octets = new int[Ipv4OctetCount];
        normalizedHost = string.IsNullOrWhiteSpace(host) ? "" : host.Trim();
        normalizedPort = port;
        error = "";

        if (!TryParseIpv4Octets(normalizedHost, octets))
        {
            error = "Bad IP";
            return false;
        }

        if (normalizedPort < 1 || normalizedPort > 65535)
        {
            error = "Bad port";
            return false;
        }

        normalizedHost = FormatIpv4Octets(octets);
        return true;
    }

    private void BuildPanel()
    {
        if (panelRoot != null)
        {
            return;
        }

        Transform cameraTransform = ResolveCameraTransform();

        panelRoot = new GameObject("Quest UDP Endpoint Panel");
        if (cameraTransform != null)
        {
            panelRoot.transform.SetParent(cameraTransform, false);
            panelRoot.transform.localPosition = new Vector3(0.0f, -0.08f, panelDistanceMeters);
            panelRoot.transform.localRotation = Quaternion.identity;
        }

        Canvas canvas = panelRoot.AddComponent<Canvas>();
        canvas.renderMode = cameraTransform != null ? RenderMode.WorldSpace : RenderMode.ScreenSpaceOverlay;
        canvas.worldCamera = cameraTransform != null ? cameraTransform.GetComponent<Camera>() : null;
        canvas.sortingOrder = 500;
        panelRoot.AddComponent<GraphicRaycaster>();

        CanvasScaler scaler = panelRoot.AddComponent<CanvasScaler>();
        scaler.dynamicPixelsPerUnit = 12.0f;
        scaler.referencePixelsPerUnit = 100.0f;

        RectTransform rootRect = panelRoot.GetComponent<RectTransform>();
        rootRect.sizeDelta = panelSizePixels;
        if (cameraTransform != null)
        {
            panelRoot.transform.localScale = Vector3.one * 0.001f;
        }

        Image background = panelRoot.AddComponent<Image>();
        background.color = new Color(0.05f, 0.07f, 0.10f, 0.90f);

        CreateText(panelRoot.transform, "Quest UDP Receiver", 0.0f, 238.0f, 880.0f, 44.0f, 32, FontStyle.Bold, TextAnchor.MiddleCenter);
        CreateText(
            panelRoot.transform,
            "有历史IP时按左手Y继续；按左手X重设后用右摇杆修改IP",
            0.0f,
            188.0f,
            900.0f,
            34.0f,
            19,
            FontStyle.Bold,
            TextAnchor.MiddleCenter);
        CreateText(
            panelRoot.transform,
            "Left X: reset    Right stick: edit digits    Right B: confirm    Left Y: continue",
            0.0f,
            158.0f,
            900.0f,
            30.0f,
            18,
            FontStyle.Normal,
            TextAnchor.MiddleCenter);

        activeEndpointText = CreateText(panelRoot.transform, "Sending: --", -250.0f, 108.0f, 610.0f, 34.0f, 22, FontStyle.Normal, TextAnchor.MiddleLeft);
        statusText = CreateText(panelRoot.transform, "Ready", 320.0f, 108.0f, 260.0f, 34.0f, 22, FontStyle.Bold, TextAnchor.MiddleRight);

        CreateDigitStrip();
        portText = CreateText(panelRoot.transform, "Port: 8080", 0.0f, -32.0f, 420.0f, 32.0f, 22, FontStyle.Bold, TextAnchor.MiddleCenter);
        continueButton = CreateActionButton("继续 / 左Y", -190.0f, -218.0f, ContinueWithSavedEndpoint);
        resetButton = CreateActionButton("重设 / 左X", 190.0f, -218.0f, BeginEndpointReset);

        CreateText(panelRoot.transform, "Save once on Quest. Later builds reuse the saved IP until you change it.", 0.0f, -88.0f, 900.0f, 32.0f, 20, FontStyle.Normal, TextAnchor.MiddleCenter);
        CreateText(panelRoot.transform, "Enter the IP of the computer running the Quest UR5 control host.", 0.0f, -126.0f, 900.0f, 32.0f, 20, FontStyle.Normal, TextAnchor.MiddleCenter);
        CreateText(panelRoot.transform, "No URScript, RTDE, or real robot output is started here.", 0.0f, -164.0f, 900.0f, 32.0f, 20, FontStyle.Bold, TextAnchor.MiddleCenter);

        RefreshEndpointDisplay();
        HighlightSelectedDigit();
    }

    private Button CreateActionButton(
        string label,
        float x,
        float y,
        UnityEngine.Events.UnityAction action)
    {
        GameObject buttonObject = CreateRectObject(
            label + " Button",
            panelRoot.transform,
            x,
            y,
            260.0f,
            64.0f);
        Image image = buttonObject.AddComponent<Image>();
        image.color = new Color(0.16f, 0.38f, 0.72f, 1.0f);

        Button button = buttonObject.AddComponent<Button>();
        button.targetGraphic = image;
        button.onClick.AddListener(action);
        Text text = CreateText(
            buttonObject.transform,
            label,
            0.0f,
            0.0f,
            260.0f,
            64.0f,
            26,
            FontStyle.Bold,
            TextAnchor.MiddleCenter);
        text.raycastTarget = false;
        return button;
    }

    private void CreateDigitStrip()
    {
        const float digitWidth = 52.0f;
        const float digitHeight = 72.0f;
        const float digitGap = 6.0f;
        const float dotWidth = 24.0f;
        const float y = 36.0f;
        float totalWidth = Ipv4DigitCount * digitWidth
            + (Ipv4DigitCount - Ipv4OctetCount) * digitGap
            + (Ipv4OctetCount - 1) * dotWidth;
        float x = -totalWidth * 0.5f + digitWidth * 0.5f;
        int digitIndex = 0;

        for (int octet = 0; octet < Ipv4OctetCount; octet++)
        {
            for (int digit = 0; digit < DigitsPerOctet; digit++)
            {
                CreateDigitCell(digitIndex, x, y, digitWidth, digitHeight);
                digitIndex++;
                x += digitWidth + digitGap;
            }

            if (octet < Ipv4OctetCount - 1)
            {
                CreateText(panelRoot.transform, ".", x - digitGap * 0.5f + dotWidth * 0.5f, y - 4.0f, dotWidth, digitHeight, 42, FontStyle.Bold, TextAnchor.MiddleCenter);
                x += dotWidth;
            }
        }
    }

    private void CreateDigitCell(int index, float x, float y, float width, float height)
    {
        GameObject cellObject = CreateRectObject("IP Digit " + index, panelRoot.transform, x, y, width, height);
        Image image = cellObject.AddComponent<Image>();
        image.color = new Color(0.93f, 0.96f, 1.0f, 0.98f);

        Text text = CreateText(cellObject.transform, "0", 0.0f, 0.0f, width, height - 4.0f, 38, FontStyle.Bold, TextAnchor.MiddleCenter);
        text.color = Color.black;
        text.raycastTarget = false;

        digitBackgrounds[index] = image;
        digitTexts[index] = text;
    }

    private Text CreateText(
        Transform parent,
        string value,
        float x,
        float y,
        float width,
        float height,
        int fontSize,
        FontStyle fontStyle,
        TextAnchor alignment)
    {
        GameObject textObject = CreateRectObject("Text", parent, x, y, width, height);
        Text text = textObject.AddComponent<Text>();
        text.text = value;
        text.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        text.fontSize = fontSize;
        text.fontStyle = fontStyle;
        text.alignment = alignment;
        text.color = Color.white;
        text.raycastTarget = false;
        return text;
    }

    private static GameObject CreateRectObject(
        string name,
        Transform parent,
        float x,
        float y,
        float width,
        float height)
    {
        GameObject gameObject = new GameObject(name);
        gameObject.transform.SetParent(parent, false);
        RectTransform rectTransform = gameObject.AddComponent<RectTransform>();
        rectTransform.anchorMin = new Vector2(0.5f, 0.5f);
        rectTransform.anchorMax = new Vector2(0.5f, 0.5f);
        rectTransform.pivot = new Vector2(0.5f, 0.5f);
        rectTransform.anchoredPosition = new Vector2(x, y);
        rectTransform.sizeDelta = new Vector2(width, height);
        return gameObject;
    }

    private void ApplyEndpointFromDigits()
    {
        ApplyEndpoint(FormatIpv4Octets(hostOctets), currentPort, true, hideAfterApply);
    }

    private void ContinueWithSavedEndpoint()
    {
        ApplyEndpoint(savedOrInitialHost, savedOrInitialPort, false, false);
        ContinueWithActiveEndpoint();
    }

    private void ConfirmEditedEndpoint()
    {
        if (editingEndpoint)
        {
            ApplyEndpointFromDigits();
            return;
        }

        ContinueWithActiveEndpoint();
    }

    private void ContinueWithActiveEndpoint()
    {
        if (sender == null || !IsValidIpv4Address(sender.receiverHost))
        {
            editingEndpoint = true;
            RefreshStatusText("IP无效");
            RefreshEndpointDisplay();
            return;
        }

        editingEndpoint = false;
        RefreshStatusText("Sending");
        SetPanelVisible(false);
    }

    private void BeginEndpointReset()
    {
        editingEndpoint = true;
        selectedDigitIndex = 0;
        RefreshStatusText("右摇杆输入，右B确认");
        RefreshEndpointDisplay();
    }

    private void SetHostOctets(string host)
    {
        int[] parsed = new int[Ipv4OctetCount];
        if (!TryParseIpv4Octets(host, parsed))
        {
            TryParseIpv4Octets(DefaultReceiverHost, parsed);
        }

        for (int i = 0; i < Ipv4OctetCount; i++)
        {
            hostOctets[i] = parsed[i];
        }

        RefreshEndpointDisplay();
    }

    private void RefreshEndpointDisplay()
    {
        for (int octet = 0; octet < Ipv4OctetCount; octet++)
        {
            string padded = ClampOctet(hostOctets[octet]).ToString("D3");
            for (int digit = 0; digit < DigitsPerOctet; digit++)
            {
                int index = octet * DigitsPerOctet + digit;
                if (digitTexts[index] != null)
                {
                    digitTexts[index].text = padded[digit].ToString();
                }
            }
        }

        if (activeEndpointText != null)
        {
            string host = sender != null ? sender.receiverHost : FormatIpv4Octets(hostOctets);
            int port = sender != null ? sender.receiverPort : currentPort;
            activeEndpointText.text = "Sending: " + host + ":" + port;
        }

        if (portText != null)
        {
            portText.text = "Port: " + currentPort;
        }

        if (continueButton != null)
        {
            continueButton.gameObject.SetActive(true);
            // "继续" also commits a newly entered endpoint after "重设".
            // Keeping one primary action avoids requiring a separate keyboard
            // or controller confirmation step on the Quest runtime panel.
            continueButton.interactable = true;
        }
        if (resetButton != null)
        {
            resetButton.gameObject.SetActive(true);
            resetButton.interactable = true;
        }

        HighlightSelectedDigit();
    }

    private void SetPanelVisible(bool visible)
    {
        if (sender != null)
        {
            sender.InputBlocked = visible;
        }
        if (panelRoot == null && visible && createUiOnStart)
        {
            BuildPanel();
        }

        if (panelRoot != null)
        {
            panelRoot.SetActive(visible);
            if (visible)
            {
                RefreshEndpointDisplay();
            }
        }
    }

    private void RefreshStatusText(string status)
    {
        if (activeEndpointText != null && sender != null)
        {
            activeEndpointText.text = "Sending: " + sender.receiverHost + ":" + sender.receiverPort;
        }

        if (statusText == null)
        {
            return;
        }

        string prefix = string.IsNullOrEmpty(status) ? "" : status + " | ";
        long sequence = sender != null ? sender.LastSentSequence : -1;
        statusText.text = prefix + "Seq " + sequence;
    }

    private void HandleKeyboardShortcuts()
    {
        if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            MoveSelectedDigit(-1);
        }
        else if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            MoveSelectedDigit(1);
        }
        else if (Input.GetKeyDown(KeyCode.UpArrow))
        {
            AdjustSelectedDigit(1);
        }
        else if (Input.GetKeyDown(KeyCode.DownArrow))
        {
            AdjustSelectedDigit(-1);
        }
        else if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter))
        {
            if (editingEndpoint)
            {
                ApplyEndpointFromDigits();
            }
            else
            {
                ContinueWithSavedEndpoint();
            }
        }
        else if (Input.GetKeyDown(KeyCode.Escape))
        {
            SetPanelVisible(false);
        }
        else if (Input.GetKeyDown(KeyCode.H))
        {
            if (editingEndpoint)
            {
                ApplyPreset(DefaultReceiverHost, "Hotspot preset");
            }
        }
        else if (Input.GetKeyDown(KeyCode.S))
        {
            if (editingEndpoint)
            {
                ApplyScenePreset();
            }
        }
        else if (Input.GetKeyDown(KeyCode.R))
        {
            BeginEndpointReset();
        }
    }

    private void HandleControllerShortcuts()
    {
        RefreshControllerDevices();

        bool leftMenuPressed = ReadButtonDown(leftController, CommonUsages.menuButton, ref previousLeftMenu);
        bool leftXPressed = ReadButtonDown(leftController, CommonUsages.primaryButton, ref previousLeftPrimary);
        bool leftYPressed = ReadButtonDown(leftController, CommonUsages.secondaryButton, ref previousLeftSecondary);
        bool rightBPressed = ReadButtonDown(rightController, CommonUsages.secondaryButton, ref previousRightSecondary);

        if (leftMenuPressed)
        {
            SetPanelVisible(panelRoot == null || !panelRoot.activeSelf);
        }

        if (panelRoot == null || !panelRoot.activeSelf)
        {
            return;
        }

        EndpointShortcutAction action = ResolveEndpointShortcutAction(leftXPressed, leftYPressed, rightBPressed);
        if (action == EndpointShortcutAction.ResetEndpoint)
        {
            BeginEndpointReset();
            return;
        }

        if (action == EndpointShortcutAction.ConfirmEditedEndpoint)
        {
            ConfirmEditedEndpoint();
            return;
        }

        if (action == EndpointShortcutAction.ContinueWithSavedEndpoint)
        {
            if (editingEndpoint)
            {
                RefreshStatusText("右B确认");
            }
            else
            {
                ContinueWithSavedEndpoint();
            }
            return;
        }

        Vector2 axis;
        if (!rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out axis)
            || !editingEndpoint
            || Time.unscaledTime < nextControllerRepeatTime)
        {
            return;
        }

        if (axis.x > StickDeadZone)
        {
            MoveSelectedDigit(1);
        }
        else if (axis.x < -StickDeadZone)
        {
            MoveSelectedDigit(-1);
        }
        else if (axis.y > StickDeadZone)
        {
            AdjustSelectedDigit(1);
        }
        else if (axis.y < -StickDeadZone)
        {
            AdjustSelectedDigit(-1);
        }
    }

    private void RefreshControllerDevices()
    {
        if (!rightController.isValid)
        {
            rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        }

        if (!leftController.isValid)
        {
            leftController = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        }
    }

    private static bool ReadButtonDown(InputDevice device, InputFeatureUsage<bool> usage, ref bool previousValue)
    {
        bool currentValue = false;
        if (device.isValid)
        {
            device.TryGetFeatureValue(usage, out currentValue);
        }

        bool pressedThisFrame = currentValue && !previousValue;
        previousValue = currentValue;
        return pressedThisFrame;
    }

    private void MoveSelectedDigit(int delta)
    {
        if (!editingEndpoint)
        {
            return;
        }

        selectedDigitIndex = Mathf.Clamp(selectedDigitIndex + delta, 0, Ipv4DigitCount - 1);
        nextControllerRepeatTime = Time.unscaledTime + ControllerRepeatSeconds;
        HighlightSelectedDigit();
    }

    private void AdjustSelectedDigit(int delta)
    {
        if (!editingEndpoint)
        {
            return;
        }

        int octetIndex = selectedDigitIndex / DigitsPerOctet;
        int digitOffset = selectedDigitIndex % DigitsPerOctet;
        hostOctets[octetIndex] = AdjustIpv4Digit(hostOctets[octetIndex], digitOffset, delta);
        nextControllerRepeatTime = Time.unscaledTime + ControllerRepeatSeconds;
        RefreshEndpointDisplay();
    }

    private void ApplyPreset(string host, string status)
    {
        if (!IsValidIpv4Address(host))
        {
            RefreshStatusText("Bad preset");
            return;
        }

        SetHostOctets(host);
        ApplyEndpoint(host, currentPort, false, false);
        RefreshStatusText(status);
    }

    private void ApplyScenePreset()
    {
        if (IsValidIpv4Address(fallbackReceiverHost))
        {
            ApplyPreset(fallbackReceiverHost, "Scene preset");
            return;
        }

        ApplyPreset(GetFallbackHost(), "Scene preset");
    }

    private void HighlightSelectedDigit()
    {
        for (int i = 0; i < digitBackgrounds.Length; i++)
        {
            if (digitBackgrounds[i] == null)
            {
                continue;
            }

            bool selected = i == selectedDigitIndex;
            digitBackgrounds[i].color = selected
                ? new Color(0.18f, 0.42f, 0.85f, 1.0f)
                : new Color(0.93f, 0.96f, 1.0f, 0.98f);

            if (digitTexts[i] != null)
            {
                digitTexts[i].color = selected ? Color.white : Color.black;
            }
        }
    }

    private static int ClampOctet(int value)
    {
        return Mathf.Clamp(value, 0, 255);
    }

    private static Transform ResolveCameraTransform()
    {
        Camera mainCamera = Camera.main;
        if (mainCamera != null)
        {
            return mainCamera.transform;
        }

        Camera anyCamera = Object.FindObjectOfType<Camera>();
        return anyCamera != null ? anyCamera.transform : null;
    }
}

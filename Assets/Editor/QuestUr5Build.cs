using System;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEditor.XR.Management;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR.OpenXR;

/// <summary>One build entry point shared by macOS, Linux and Windows editors.</summary>
public static class QuestUr5Build
{
    [MenuItem("Quest UR5/Build Quest APK")]
    public static void BuildApk()
    {
        ValidateProject();
        string scene = GetMainScenePath();
        if (!BuildPipeline.IsBuildTargetSupported(BuildTargetGroup.Android, BuildTarget.Android))
        {
            throw new InvalidOperationException("Install Android Build Support, SDK/NDK and OpenJDK in Unity Hub.");
        }

        if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android)
        {
            if (Application.isBatchMode)
            {
                throw new InvalidOperationException("Start the editor with -buildTarget Android for batch APK builds.");
            }
            if (!EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, BuildTarget.Android))
            {
                throw new InvalidOperationException("Could not switch the active build target to Android.");
            }
        }
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
        PlayerSettings.Android.targetSdkVersion = AndroidSdkVersions.AndroidApiLevelAuto;
        PlayerSettings.Android.forceInternetPermission = true;
        PlayerSettings.SetApiCompatibilityLevel(BuildTargetGroup.Android, ApiCompatibilityLevel.NET_Standard);
        Directory.CreateDirectory("Builds");
        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = new[] { scene },
            locationPathName = "Builds/quest-ur5.apk",
            target = BuildTarget.Android,
            options = BuildOptions.None
        });
        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new InvalidOperationException("Quest APK build failed: " + report.summary.result);
        }
        Debug.Log("Quest APK: " + Path.GetFullPath("Builds/quest-ur5.apk"));
    }

    [MenuItem("Quest UR5/Validate Project")]
    public static void ValidateProject()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            throw new InvalidOperationException("Exit Play mode before validating or building the project.");
        }
        string path = GetMainScenePath();
        if (!File.Exists(path))
        {
            throw new FileNotFoundException("Quest UR5 main scene is missing", path);
        }
        var xrSettings = XRGeneralSettingsPerBuildTarget.XRGeneralSettingsForBuildTarget(BuildTargetGroup.Android);
        if (xrSettings == null || xrSettings.Manager == null || !xrSettings.InitManagerOnStart)
        {
            throw new InvalidOperationException("Restore Android XR settings and enable Initialize XR on Startup.");
        }
        bool hasOpenXrLoader = false;
        foreach (var loader in xrSettings.Manager.activeLoaders)
        {
            hasOpenXrLoader |= loader is OpenXRLoader;
        }
        if (!hasOpenXrLoader)
        {
            throw new InvalidOperationException("Enable the OpenXR loader for Android in XR Plug-in Management.");
        }

        Scene scene = SceneManager.GetSceneByPath(path);
        bool openedForValidation = !scene.isLoaded;
        if (openedForValidation)
        {
            scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Additive);
        }
        try
        {
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (Transform child in root.GetComponentsInChildren<Transform>(true))
                {
                    if (GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(child.gameObject) > 0)
                    {
                        throw new InvalidOperationException("Missing script in main scene: " + child.name
                            + ". Restore the locked packages and asset metadata before building.");
                    }
                }
            }
        }
        finally
        {
            if (openedForValidation)
            {
                EditorSceneManager.CloseScene(scene, true);
            }
        }
        Debug.Log("Quest UR5 validation passed: " + path + "; editor " + Application.unityVersion
            + "; host " + Application.platform);
    }

    private static string GetMainScenePath()
    {
        return File.Exists("Assets/Scenes/SampleScene.unity")
            ? "Assets/Scenes/SampleScene.unity"
            : "Assets/Scenes/SampleScene.scene";
    }

    [MenuItem("Quest UR5/Normalize Metadata for Unity Export")]
    public static void NormalizeMetadataForExport()
    {
        // Use the source editor's asset database to preserve the actual identity.
        // Generating new GUIDs would break scene, material and XR references.
        int changed = 0;
        foreach (string metaPath in Directory.GetFiles("Assets", "*.meta", SearchOption.AllDirectories))
        {
            string text = File.ReadAllText(metaPath);
            Match match = Regex.Match(text, @"^guid: (\S+)\r?$", RegexOptions.Multiline);
            if (!match.Success || Regex.IsMatch(match.Groups[1].Value, "^[a-fA-F0-9]{32}$"))
            {
                continue;
            }
            string assetPath = metaPath.Substring(0, metaPath.Length - ".meta".Length).Replace('\\', '/');
            string guid = AssetDatabase.AssetPathToGUID(assetPath);
            if (!Regex.IsMatch(guid, "^[a-fA-F0-9]{32}$"))
            {
                throw new InvalidOperationException("Cannot resolve original GUID for " + assetPath);
            }
            string normalized = text.Substring(0, match.Groups[1].Index) + guid
                + text.Substring(match.Groups[1].Index + match.Groups[1].Length);
            File.WriteAllText(metaPath, normalized, new UTF8Encoding(false));
            changed++;
        }
        Debug.Log("Normalized " + changed + " asset metadata GUIDs for standard Unity export.");
    }
}

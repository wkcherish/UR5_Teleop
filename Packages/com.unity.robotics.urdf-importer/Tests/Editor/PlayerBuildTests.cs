using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.TestTools;
using UnityEngine;
using UnityEngine.TestTools;

namespace BuildTests
{
    public class PlayerBuilder
    {
        const string BuildOutputRoot = "Temp/UrdfImporterBuildTests";
        List<EditorBuildSettingsScene> m_EditorBuildSettingsScenes = new List<EditorBuildSettingsScene>();
        BuildSummary m_Summary;

        [SetUp]
        public void SetUp()
        {
            m_EditorBuildSettingsScenes = EditorBuildSettings.scenes
                .Where(scene => scene.enabled)
                .ToList();

            Assert.IsNotEmpty(m_EditorBuildSettingsScenes, "At least one enabled build scene is required.");
            foreach (EditorBuildSettingsScene scene in m_EditorBuildSettingsScenes)
            {
                Assert.IsTrue(File.Exists(scene.path), $"Build scene does not exist: {scene.path}");
            }
        }

        [TearDown]
        public void TearDown()
        {
            if (Directory.Exists(BuildOutputRoot))
            {
                Directory.Delete(BuildOutputRoot, true);
            }
        }

        [UnityPlatform(RuntimePlatform.WindowsEditor)]
        [RequirePlatformSupport(BuildTarget.StandaloneWindows64)]
        [Test]
        public void BuildPlayerStandaloneWindows64()
        {
            BuildPlayer(BuildTargetGroup.Standalone, BuildTarget.StandaloneWindows64, BuildOptions.None, out _, out m_Summary);
            Assert.AreEqual(BuildResult.Succeeded, m_Summary.result, " BuildTarget.StandaloneWindows64 failed to build");
        }

        [RequirePlatformSupport(BuildTarget.StandaloneLinux64)]
        [Test]
        public void BuildPlayerLinux()
        {
            BuildPlayer(BuildTargetGroup.Standalone, BuildTarget.StandaloneLinux64, BuildOptions.None, out _, out m_Summary);
            Assert.AreEqual(BuildResult.Succeeded, m_Summary.result, "BuildTarget.StandaloneLinux64 failed to build");
        }

        [UnityPlatform(RuntimePlatform.OSXEditor)]
        [RequirePlatformSupport(BuildTarget.StandaloneOSX)]
        [Test]
        public void BuildPlayerOSX()
        {
            BuildPlayer(BuildTargetGroup.Standalone, BuildTarget.StandaloneOSX, BuildOptions.None, out _, out m_Summary);
            Assert.AreEqual(BuildResult.Succeeded, m_Summary.result, "BuildTarget.StandaloneOSX failed to build");
        }

        void BuildPlayer(BuildTargetGroup buildTargetGroup, BuildTarget buildTarget, BuildOptions buildOptions,
            out BuildReport buildReport, out BuildSummary buildSummary)
        {
            BuildPlayerOptions buildPlayerOptions = new BuildPlayerOptions();
            buildPlayerOptions.scenes = m_EditorBuildSettingsScenes
                .Select(scene => scene.path)
                .ToArray();
            buildPlayerOptions.locationPathName = GetBuildOutputPath(buildTarget);
            buildPlayerOptions.target = buildTarget;
            buildPlayerOptions.options = buildOptions;
            buildPlayerOptions.targetGroup = buildTargetGroup;

            // Batchmode 下当前场景可能是未保存的 Untitled Scene，必须显式传入保存过的构建场景。
            buildReport = BuildPipeline.BuildPlayer(buildPlayerOptions);
            buildSummary = buildReport.summary;
        }

        static string GetBuildOutputPath(BuildTarget buildTarget)
        {
            switch (buildTarget)
            {
                case BuildTarget.StandaloneOSX:
                    return Path.Combine(BuildOutputRoot, "UrdfImporterBuildTest.app");
                case BuildTarget.StandaloneWindows64:
                    return Path.Combine(BuildOutputRoot, "UrdfImporterBuildTest.exe");
                default:
                    return Path.Combine(BuildOutputRoot, "UrdfImporterBuildTest");
            }
        }
    }
}

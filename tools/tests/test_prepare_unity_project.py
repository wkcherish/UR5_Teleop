"""Filesystem and serialization cases that can break cross-platform Unity exports."""

import json
from pathlib import Path, PurePosixPath, PureWindowsPath

import pytest

from tools import prepare_unity_project as exporter


@pytest.fixture
def source(tmp_path, monkeypatch):
    root = tmp_path / "source"
    files = {
        "Assets/Scenes/SampleScene.scene": "%TAG !u! tag:yousandi.cn,2023:\nscene: 测试\n",
        "Assets/Scenes/SampleScene.scene.meta": "guid: 0123456789abcdef0123456789abcdef\n",
        "Assets/Scripts/Startup.cs": "SceneManager.sceneLoaded += HandleSceneLoaded;\n",
        "ProjectSettings/ProjectSettings.asset": "%TAG !u! tag:yousandi.cn,2023:\n",
        "ProjectSettings/EditorBuildSettings.asset": "path: Assets/Scenes/SampleScene.scene\n",
        "Packages/com.unity.robotics.urdf-importer/package.json": json.dumps(
            {"name": "com.unity.robotics.urdf-importer"}
        ),
        "tools/unity-manifest.json": json.dumps(
            {"dependencies": {"com.unity.robotics.urdf-importer": "file:com.unity.robotics.urdf-importer"}}
        ),
        "tools/unity-packages-lock.json": json.dumps(
            {
                "dependencies": {
                    "com.unity.robotics.urdf-importer": {
                        "version": "file:com.unity.robotics.urdf-importer",
                        "dependencies": {},
                    }
                }
            }
        ),
    }
    for name, content in files.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    monkeypatch.setattr(exporter, "ROOT", root)
    return root


def test_export_preserves_guids_and_scene_api_names(source, tmp_path):
    destination = exporter.export_project(tmp_path / "Unity project with spaces 中文")
    assert (destination / "Assets/Scenes/SampleScene.unity").is_file()
    assert not list(destination.rglob("*.scene"))
    scene_text = (destination / "Assets/Scenes/SampleScene.unity").read_text(encoding="utf-8")
    assert "tag:unity3d.com,2011:" in scene_text
    assert "scene: 测试" in scene_text
    assert "0123456789abcdef0123456789abcdef" in (
        destination / "Assets/Scenes/SampleScene.unity.meta"
    ).read_text()
    assert "SceneManager.sceneLoaded" in (destination / "Assets/Scripts/Startup.cs").read_text()
    assert "Assets/Scenes/SampleScene.unity" in (
        destination / "ProjectSettings/EditorBuildSettings.asset"
    ).read_text()
    assert "SampleScene.scene" in (source / "ProjectSettings/EditorBuildSettings.asset").read_text()


@pytest.mark.parametrize("name", ["CON.txt", "aux", "COM1.asset", "file.", "file ", "bad:name", "bad\\name"])
def test_rejects_windows_incompatible_names(name):
    with pytest.raises(ValueError, match="not portable"):
        exporter.audit_portable_paths([PurePosixPath("Assets") / name])


def test_rejects_case_collisions_without_needing_case_sensitive_disk():
    with pytest.raises(ValueError, match="filename collision"):
        exporter.audit_portable_paths([Path("Assets/Robot.prefab"), Path("Assets/robot.prefab")])


def test_rejects_collisions_when_path_objects_compare_case_insensitively():
    with pytest.raises(ValueError, match="filename collision"):
        exporter.audit_portable_paths([PureWindowsPath("Assets/Robot.prefab"), PureWindowsPath("Assets/robot.prefab")])


def test_refuses_existing_destination(source, tmp_path):
    destination = tmp_path / "existing"
    destination.mkdir()
    sentinel = destination / "keep.txt"
    sentinel.write_text("keep")
    with pytest.raises(FileExistsError):
        exporter.export_project(destination)
    assert sentinel.read_text() == "keep"


def test_refuses_output_inside_source(source):
    with pytest.raises(ValueError, match="outside"):
        exporter.export_project(source / "export")


def test_duplicate_guid_rolls_back_only_the_new_export(source, tmp_path):
    extra_meta = source / "Assets/Scripts/Startup.cs.meta"
    extra_meta.write_text((source / "Assets/Scenes/SampleScene.scene.meta").read_text())
    destination = tmp_path / "export"
    with pytest.raises(ValueError, match="duplicate asset GUID"):
        exporter.export_project(destination)
    assert not destination.exists()
    assert extra_meta.exists()


def test_rejects_encoded_guid_instead_of_skipping_it(source, tmp_path):
    meta = source / "Assets/Scenes/SampleScene.scene.meta"
    meta.write_text("guid: Wi4c5Hn7BilnQzLPx8Be1uwSvOxTbXWtICqao1piMM3mdU0QAVl4ZAQ=\n")
    with pytest.raises(ValueError, match="nonstandard asset GUID"):
        exporter.export_project(tmp_path / "export")


def test_export_does_not_copy_editor_or_sdk_generated_state(source, tmp_path):
    for name in (
        "ProjectSettings/PackageManagerSettings.asset",
        "Assets/RuntimeActionBindings.json",
        "Assets/RuntimeActionBindings.json.meta",
        "Assets/.DS_Store",
    ):
        (source / name).write_text("local state", encoding="utf-8")
    destination = exporter.export_project(tmp_path / "export")
    assert not (destination / "ProjectSettings/PackageManagerSettings.asset").exists()
    assert not list(destination.rglob("RuntimeActionBindings.json*"))
    assert not list(destination.rglob(".DS_Store"))


@pytest.mark.parametrize("dependency", ["file:/Users/example/importer", "file:C:/importer", "file:../importer"])
def test_rejects_machine_specific_embedded_package_paths(source, tmp_path, dependency):
    name = "com.unity.robotics.urdf-importer"
    (source / "tools/unity-manifest.json").write_text(json.dumps({"dependencies": {name: dependency}}))
    lock_path = source / "tools/unity-packages-lock.json"
    lock = json.loads(lock_path.read_text())
    lock["dependencies"][name]["version"] = dependency
    lock_path.write_text(json.dumps(lock))
    with pytest.raises(ValueError, match="package path must be relative"):
        exporter.export_project(tmp_path / "export")

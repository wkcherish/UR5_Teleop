#!/usr/bin/env python3
"""Export the Tuanjie project to standard Unity without modifying the source.

Only source assets, embedded packages and project settings are copied. No
editor, native build tool or network access is needed to perform the export.
"""

import argparse
import json
import re
import shutil
import unicodedata
from pathlib import Path, PurePath, PureWindowsPath

ROOT = Path(__file__).resolve().parents[1]
UNITY_VERSION = "2022.3.62f1"
TEXT_SUFFIXES = {".asset", ".prefab", ".mat", ".scene", ".unity", ".meta", ".cs", ".md", ".json"}
WINDOWS_RESERVED_NAMES = {"CON", "PRN", "AUX", "NUL"} | {
    f"{prefix}{number}" for prefix in ("COM", "LPT") for number in range(1, 10)
}


def audit_portable_paths(paths: list[PurePath]) -> None:
    """Reject names that differ between Windows, Linux and macOS filesystems."""
    seen: dict[str, PurePath] = {}
    for path in paths:
        for part in path.parts:
            if (
                re.search(r'[<>:"\\|?*\x00-\x1f]', part)
                or part.endswith((".", " "))
                or part.split(".")[0].upper() in WINDOWS_RESERVED_NAMES
            ):
                raise ValueError(f"filename is not portable to Windows: {path}")
        key = unicodedata.normalize("NFC", path.as_posix()).casefold()
        if key in seen and seen[key].as_posix() != path.as_posix():
            raise ValueError(f"case-insensitive filename collision: {seen[key]} and {path}")
        seen[key] = path


def export_project(destination: Path) -> Path:
    destination = destination.resolve()
    if destination == ROOT or ROOT in destination.parents or destination in ROOT.parents:
        raise ValueError("output must be outside the source repository")
    if destination.exists():
        raise FileExistsError("output already exists; choose a new directory")
    destination.mkdir(parents=True)
    try:
        scene_names = [p.name for p in (ROOT / "Assets").rglob("*.scene")]
        for name in ("Assets", "ProjectSettings"):
            # Editor registry UI caches and SDK-generated bindings are local state.
            # The destination editor and Meta XR recreate them from shared inputs.
            shutil.copytree(
                ROOT / name,
                destination / name,
                ignore=shutil.ignore_patterns(
                    ".DS_Store", "PackageManagerSettings.asset", "RuntimeActionBindings.json", "RuntimeActionBindings.json.meta"
                ),
            )
        packages = destination / "Packages"
        packages.mkdir()
        shutil.copytree(
            ROOT / "Packages/com.unity.robotics.urdf-importer", packages / "com.unity.robotics.urdf-importer"
        )
        shutil.copyfile(ROOT / "tools/unity-manifest.json", packages / "manifest.json")
        shutil.copyfile(ROOT / "tools/unity-packages-lock.json", packages / "packages-lock.json")
        for path in list(destination.rglob("*")):
            if not path.is_file() or path.suffix not in TEXT_SUFFIXES:
                continue
            try:
                text = path.read_text(encoding="utf-8-sig")
            except UnicodeError:
                continue
            updated = text.replace("tag:yousandi.cn,2023:", "tag:unity3d.com,2011:")
            # Replace scene filenames only: a blanket suffix replacement also
            # corrupts API names such as SceneManager.sceneLoaded.
            for name in scene_names:
                updated = updated.replace(name, str(Path(name).with_suffix(".unity")))
            if updated != text:
                path.write_text(updated, encoding="utf-8")
        for scene in list((destination / "Assets").rglob("*.scene")):
            scene.rename(scene.with_suffix(".unity"))
            meta = Path(str(scene) + ".meta")
            if meta.exists():
                meta.rename(Path(str(scene.with_suffix(".unity")) + ".meta"))
        (destination / "ProjectSettings/ProjectVersion.txt").write_text(
            f"m_EditorVersion: {UNITY_VERSION}\n", encoding="utf-8"
        )
        # Tuanjie-only build targets and launcher settings have no Unity equivalent.
        settings = destination / "ProjectSettings/ProjectSettings.asset"
        text = settings.read_text(encoding="utf-8")
        text = re.sub(r"UNITY_UOS_SECURITY;?", "", text)
        settings.write_text(text, encoding="utf-8")
        audit_project(destination)
    except BaseException:
        # This invocation created the directory and owns all of its contents.
        shutil.rmtree(destination)
        raise
    return destination


def audit_project(destination: Path) -> None:
    audit_portable_paths([p.relative_to(destination) for p in destination.rglob("*")])
    manifest = json.loads((destination / "Packages/manifest.json").read_text(encoding="utf-8"))
    lock = json.loads((destination / "Packages/packages-lock.json").read_text(encoding="utf-8"))["dependencies"]
    for name, version in manifest["dependencies"].items():
        if name not in lock or lock[name]["version"] != version:
            raise ValueError(f"package lock disagrees with manifest: {name}")
        if version.startswith("file:"):
            relative = version.removeprefix("file:")
            folder = Path(relative)
            if (
                folder.is_absolute()
                or PureWindowsPath(relative).root
                or PureWindowsPath(relative).drive
                or "\\" in relative
                or ".." in folder.parts
            ):
                raise ValueError(f"package path must be relative to Packages: {name}")
            package_json = destination / "Packages" / folder / "package.json"
            if not package_json.is_file() or json.loads(package_json.read_text(encoding="utf-8"))["name"] != name:
                raise ValueError(f"embedded package is missing or has the wrong name: {name}")
    for name, package in lock.items():
        if any(dep not in lock for dep in package["dependencies"]):
            raise ValueError(f"package lock is incomplete: {name}")
    if any(".codely" in str(value) or "tuanjie" in key for key, value in manifest["dependencies"].items()):
        raise ValueError("export still contains local tooling dependencies")
    guids: dict[str, Path] = {}
    for path in destination.rglob("*.meta"):
        match = re.search(r"^guid: (\S+)\s*$", path.read_text(encoding="utf-8-sig"), re.M)
        if match is None:
            raise ValueError(f"missing asset GUID: {path}")
        if match and not re.fullmatch("[a-fA-F0-9]{32}", match[1]):
            raise ValueError(
                f"nonstandard asset GUID: {path}; in the source editor run "
                "Quest UR5 > Normalize Metadata for Unity Export, then export again"
            )
        if match:
            guid = match[1].lower()
            if guid in guids:
                raise ValueError(f"duplicate asset GUID: {guids[guid]} and {path}")
            guids[guid] = path
    for path in destination.rglob("*"):
        if path.suffix in TEXT_SUFFIXES and path.is_file():
            try:
                text = path.read_text(encoding="utf-8-sig")
            except UnicodeError:
                continue  # Unity also supports binary assets with these extensions.
            if "tag:yousandi.cn,2023:" in text:
                raise ValueError(f"unconverted serialization tag: {path}")
    scene = destination / "Assets/Scenes/SampleScene.unity"
    if not scene.exists() or "tag:yousandi" in scene.read_text(encoding="utf-8"):
        raise ValueError("main scene conversion failed")
    build_settings = (destination / "ProjectSettings/EditorBuildSettings.asset").read_text(encoding="utf-8")
    if "Assets/Scenes/SampleScene.unity" not in build_settings:
        raise ValueError("main scene is missing from build settings")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        path = export_project(args.output)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f"Exported Unity {UNITY_VERSION} project: {path}")
    print("Open in Unity Hub with Android Build Support, SDK/NDK and OpenJDK installed.")


if __name__ == "__main__":
    main()

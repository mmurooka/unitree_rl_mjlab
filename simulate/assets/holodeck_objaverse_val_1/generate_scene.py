#!/usr/bin/env python3
"""Build G1 in MolmoSpaces holodeck-objaverse-val scene val_1."""

from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


EXCLUDED_ASSET_IDS = {
    # These source assets impose NC/SA terms that are undesirable for this repository.
    "4f3431d0740043bd94c2121e5d654d3a",
    "d0fc410c77f74c50b526c94200d498f8",
    # Ceiling fixtures are visual clutter and add unnecessary collision meshes.
    "855d96efa50a480eb7e0c70cb9f71389",  # ceiling light fixture
    "89959f29b59642759ee20867e8ca39a2",  # ceiling fan
}

REMOVED_FIXTURE_CATEGORIES = {"LightSwitch", "ToiletPaperHanger"}


def remove_matching_elements(root: ET.Element, tokens: set[str]) -> set[str]:
    removed_names: set[str] = set()
    changed = True
    while changed:
        changed = False
        for parent in root.iter():
            for child in list(parent):
                values = child.attrib.values()
                if any(token in value for token in tokens for value in values):
                    removed_names.update(
                        element.get("name")
                        for element in child.iter()
                        if element.get("name") is not None
                    )
                    parent.remove(child)
                    changed = True
        tokens |= removed_names
    return removed_names


def removable_asset_names(scene_path: Path) -> set[str]:
    metadata_path = scene_path.with_name(f"{scene_path.stem}_metadata.json")
    objects = json.loads(metadata_path.read_text())["objects"]
    return {
        name
        for name, info in objects.items()
        if info.get("parent") or info.get("category") in REMOVED_FIXTURE_CATEGORIES
    }


def prune_unused_assets(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is None:
        return

    for tag, reference_attribute in (
        ("mesh", "mesh"),
        ("material", "material"),
        ("texture", "texture"),
    ):
        used = {
            element.get(reference_attribute)
            for element in root.iter()
            if element.tag != tag and element.get(reference_attribute) is not None
        }
        for element in list(asset):
            if element.tag == tag and element.get("name") not in used:
                asset.remove(element)


def clean_generated_assets(output_dir: Path) -> None:
    for name in ("objects", "scene_assets"):
        path = output_dir / name
        if path.exists():
            shutil.rmtree(path)


def freeze_environment(root: ET.Element) -> None:
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "joint":
                parent.remove(child)


def configure_white_background(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is None:
        return
    for element in list(asset):
        if element.tag == "texture" and element.get("type") == "skybox":
            asset.remove(element)
    asset.append(
        ET.Element(
            "texture",
            {
                "type": "skybox",
                "builtin": "gradient",
                "rgb1": "1 1 1",
                "rgb2": "1 1 1",
                "width": "512",
                "height": "3072",
            },
        )
    )


def translate_environment_to_robot_origin(root: ET.Element) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("The MolmoSpaces scene is missing its worldbody")

    environment = ET.Element(
        "body", {"name": "holodeck_environment", "pos": "-3.5 -1.0 0"}
    )
    for element in list(worldbody):
        worldbody.remove(element)
        environment.append(element)
    worldbody.append(environment)


def copy_environment_assets(scene_path: Path, root: ET.Element, output_dir: Path) -> None:
    for element in root.iter():
        file_name = element.get("file")
        if file_name is None:
            continue

        source = (scene_path.parent / file_name).resolve()
        source_parts = source.parts
        if "objects" in source_parts:
            relative = Path(*source_parts[source_parts.index("objects") :])
        elif source.parent.name == "val_1_assets":
            relative = Path("scene_assets") / source.name
        else:
            relative = Path("scene_assets") / source.name

        destination = output_dir / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.exists():
            shutil.copy2(source, destination)
        element.set("file", relative.as_posix())


def merge_g1(root: ET.Element, g1_path: Path, output_path: Path) -> None:
    g1_root = ET.parse(g1_path).getroot()
    environment_asset = root.find("asset")
    environment_default = root.find("default")
    environment_worldbody = root.find("worldbody")
    if environment_asset is None or environment_default is None or environment_worldbody is None:
        raise RuntimeError("The MolmoSpaces scene is missing a required MJCF section")

    g1_asset = g1_root.find("asset")
    g1_default = g1_root.find("default")
    g1_worldbody = g1_root.find("worldbody")
    if g1_asset is None or g1_default is None or g1_worldbody is None:
        raise RuntimeError("The G1 scene is missing a required MJCF section")

    relative_g1_assets = Path(os.path.relpath(g1_path.parent / "assets", output_path.parent))
    for element in g1_asset:
        cloned = copy.deepcopy(element)
        file_name = cloned.get("file")
        if file_name is not None:
            cloned.set("file", (relative_g1_assets / file_name).as_posix())
        environment_asset.append(cloned)

    for element in g1_default:
        environment_default.append(copy.deepcopy(element))

    pelvis = g1_worldbody.find("body[@name='pelvis']")
    if pelvis is None:
        raise RuntimeError("Could not find the G1 pelvis body")
    pelvis = copy.deepcopy(pelvis)
    pelvis.set("pos", "0 0 0.793")
    pelvis.set("quat", "0.70710678 0 0 0.70710678")
    environment_worldbody.insert(0, pelvis)

    for section_name in ("actuator", "sensor"):
        section = g1_root.find(section_name)
        if section is not None:
            root.append(copy.deepcopy(section))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--molmospaces-scene", type=Path, required=True)
    parser.add_argument("--g1-scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    tree = ET.parse(args.molmospaces_scene)
    root = tree.getroot()
    root.set("model", "g1_holodeck_objaverse_val_1")

    excluded = set(EXCLUDED_ASSET_IDS) | removable_asset_names(args.molmospaces_scene)
    remove_matching_elements(root, excluded)
    prune_unused_assets(root)
    freeze_environment(root)
    configure_white_background(root)
    translate_environment_to_robot_origin(root)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    clean_generated_assets(args.output.parent)
    copy_environment_assets(args.molmospaces_scene, root, args.output.parent)
    merge_g1(root, args.g1_scene, args.output)
    ET.indent(tree, space="  ")
    tree.write(args.output, encoding="unicode", xml_declaration=False)


if __name__ == "__main__":
    main()

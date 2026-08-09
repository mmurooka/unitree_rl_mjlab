# MolmoSpaces Holodeck Objaverse val_1

This directory contains MolmoSpaces `holodeck-objaverse-val/val_1`, merged with
the repository's G1 model and MID-360 site. The environment has ten connected
office spaces, including hallways, a conference room, work areas, a kitchenette,
restrooms, and an assembly area.

The environment objects are deliberately frozen. This keeps the G1 free joint
first in MuJoCo's state and preserves the assumptions made by the existing
Unitree SDK bridge.

To keep the navigation scene lightweight, all small manipulation objects placed
on other objects are removed during generation. Small wall fixtures such as
light switches and toilet-paper holders are also removed, together with ceiling
light and fan geometry.

## Provenance and licenses

The data files in this directory are licensed separately from the repository's
software license under the terms listed below.

- Source repository: `allenai/molmospaces`
- Source commit: `1d9f2f01b38a4c911a9ec11be963be481ed4e225`
- Scene source/version: `holodeck-objaverse-val/20251217_with_occupancy`
- Scene ID: `val_1`
- Scene composition: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), Allen Institute for AI (Ai2)
- THOR assets: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), Allen Institute for AI (Ai2)
- Objaverse subset: [ODC-BY 1.0](https://opendatacommons.org/licenses/by/1-0/)
- Included Objaverse models: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/); see `ATTRIBUTIONS.json`

Two source-scene objects were removed because their individual licenses were
CC BY-SA 4.0 and CC BY-NC 4.0. The full per-object attribution and exclusion list
is recorded in `ATTRIBUTIONS.json`.

This is a modified distribution. The original scene was merged with the Unitree
G1 model, translated to place G1 at the origin, frozen, stripped of manipulation
objects and minor fixtures, given a white background, and augmented with the
simulated MID-360 site and sensors. These modifications are not endorsed by Ai2
or the individual asset creators.

MolmoSpaces also states that its provided data is intended for research and
education under the [Ai2 Responsible Use Guidelines](https://allenai.org/responsible-use).

## Regeneration

Install the MolmoSpaces `val_1` scene and its referenced objects, then run:

```bash
python3 simulate/assets/holodeck_objaverse_val_1/generate_scene.py \
  --molmospaces-scene /path/to/holodeck-objaverse-val/val_1.xml \
  --g1-scene src/assets/robots/unitree_g1/xmls/scene_g1.xml \
  --output simulate/assets/holodeck_objaverse_val_1/scene_g1.xml
```

The generated scene can be validated with:

```bash
simulate/mujoco/bin/compile \
  simulate/assets/holodeck_objaverse_val_1/scene_g1.xml \
  /tmp/g1_holodeck_objaverse_val_1.mjb
```

# VLAPLUSSIMULATION

Isaac Sim tabletop lab — a simple room with a table and your custom robot.

---

## Prerequisites

| Requirement | Notes |
|---|---|
| NVIDIA Isaac Sim | 2023.x **or** 4.x — scripts auto-detect |
| Python | The Python bundled with Isaac Sim (use `python.sh` / `python.bat`) |
| Your robot URDF | Any valid URDF with mesh paths accessible on the server |

---

## Quick start

All commands use Isaac Sim's bundled Python so that `omni.*` / `isaacsim` packages are available.

### 1 — SSH into your AWS instance and clone/pull the repo

```bash
cd ~/vlaplussimulation
```

### 2 — Build the lab scene

Robot USD (LIO2):
```
/home/ubuntu/Documents/repos/ros2_ws/src/fp_descriptions/isaac_simulation/robots/LIO2/LIO2.usd
```

```bash
~/.local/share/ov/pkg/isaac-sim-*/python.sh setup_lab.py \
    --robot-usd /home/ubuntu/Documents/repos/ros2_ws/src/fp_descriptions/isaac_simulation/robots/LIO2/LIO2.usd \
    --output lab_scene.usd
```

If you have a URDF instead, you can convert and build in one step:
```bash
python.sh setup_lab.py --urdf /path/to/robot.urdf --output lab_scene.usd
```

This produces `lab_scene.usd` — a self-contained scene you can open in Isaac Sim GUI.

### 4 — Open the scene in Isaac Sim GUI

```bash
# Add --gui to keep the window open after building:
python.sh setup_lab.py --robot-usd assets/robot.usd --output lab_scene.usd --gui

# Or open manually in the Isaac Sim Launcher → open lab_scene.usd
```

---

## File overview

```
VLAPLUSSIMULATION/
├── convert_urdf.py   # Step 1: URDF → USD conversion
├── setup_lab.py      # Step 2: builds floor + table + robot, saves lab_scene.usd
├── assets/           # Created automatically; holds robot.usd after conversion
└── lab_scene.usd     # Generated output scene
```

---

## Scene layout

```
/World
├── PhysicsScene        — gravity (9.81 m/s², Z-down)
├── Lights/
│   └── DomeLight       — ambient fill
├── GroundPlane         — infinite collision floor
├── Table/
│   ├── Top             — 1.2 m × 0.7 m × 5 cm slab at z = 0.75 m
│   └── Leg_{0..3}      — cylindrical legs with collision
└── Robot               — USD reference to assets/robot.usd, base at tabletop surface
```

---

## Customisation tips

| What to change | Where |
|---|---|
| Table size / height | `_add_table()` in `setup_lab.py` |
| Robot position on table | `_add_robot()` — edit the `AddTranslateOp` |
| Fix robot base to world | `config.fix_base = True` in `convert_urdf.py` |
| Add objects (bins, props) | Call `omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", ...)` in `build_scene()` |
| Add a second light | Call `_add_distant_light(stage)` inside `build_scene()` |

"""
setup_lab.py  —  Build a tabletop lab scene in Isaac Sim and save it as USD.

Workflow:
  1. (Optional) auto-convert URDF if the robot USD does not yet exist.
  2. Create a new USD stage with physics, lighting, floor, and a table.
  3. Reference the robot USD onto the table surface.
  4. Save the composed scene to --output (default: lab_scene.usd).

Usage:
    # Headless build (typical on AWS):
    python setup_lab.py --robot-usd assets/robot.usd --output lab_scene.usd

    # Auto-convert URDF first, then build:
    python setup_lab.py --urdf path/to/robot.urdf --output lab_scene.usd

    # Open the resulting scene interactively after building:
    python setup_lab.py --robot-usd assets/robot.usd --output lab_scene.usd --gui

Compatible with Isaac Sim 2023.x and 4.x.
"""

import argparse
import os
import sys


# ---------------------------------------------------------------------------
# Isaac Sim version detection + app launch
# ---------------------------------------------------------------------------

def _detect_isaac_version() -> int:
    try:
        import isaacsim  # noqa: F401
        return 4
    except ImportError:
        return 3


def _launch_app(headless: bool):
    if _detect_isaac_version() == 4:
        from isaacsim import SimulationApp
    else:
        from omni.isaac.kit import SimulationApp
    return SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})


# ---------------------------------------------------------------------------
# Scene helpers
# ---------------------------------------------------------------------------

def _add_physics_scene(stage):
    from pxr import UsdPhysics, PhysxSchema, Gf
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/PhysicsScene"))


def _add_dome_light(stage, intensity: float = 1000.0):
    from pxr import UsdLux
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(intensity)
    dome.CreateTextureFormatAttr("latlong")


def _add_distant_light(stage, intensity: float = 3000.0):
    from pxr import UsdLux, Gf
    import omni.kit.commands
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXform",
        prim_type="DistantLight",
        prim_path="/World/Lights/DistantLight",
        attributes={"intensity": intensity, "angle": 1.0},
    )


def _add_ground_plane(stage):
    """Infinite ground plane with collision."""
    import omni.kit.commands
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage,
        planePath="/World/GroundPlane",
        axis="Z",
        size=1500,
        position=(0, 0, 0),
        color=(0.5, 0.5, 0.5),
    )


def _add_table(stage, table_pos=(0.0, 0.0, 0.0)):
    """
    Create a simple table:
      - Top  : 1.2 m × 0.7 m × 0.05 m slab at table_height
      - Legs : 4 cylinders

    Returns the Z position of the tabletop surface.
    """
    from pxr import UsdGeom, UsdPhysics, Gf, Vt
    import omni.kit.commands

    table_height = 0.75          # metres to top surface
    top_thickness = 0.05
    top_z = table_height - top_thickness / 2.0   # centre of top slab
    top_surface_z = table_height                  # where robot base sits

    leg_height = table_height - top_thickness
    leg_radius = 0.03
    leg_offsets = [
        ( 0.55,  0.30),
        (-0.55,  0.30),
        ( 0.55, -0.30),
        (-0.55, -0.30),
    ]

    # --- tabletop ---
    top_path = "/World/Table/Top"
    omni.kit.commands.execute(
        "CreateMeshPrimWithDefaultXform",
        prim_type="Cube",
        prim_path=top_path,
    )
    top_prim = stage.GetPrimAtPath(top_path)
    xform = UsdGeom.Xformable(top_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(table_pos[0], table_pos[1], table_pos[2] + top_z))
    xform.AddScaleOp().Set(Gf.Vec3d(0.60, 0.35, top_thickness / 2.0))

    UsdPhysics.CollisionAPI.Apply(top_prim)

    # --- legs ---
    for i, (lx, ly) in enumerate(leg_offsets):
        leg_path = f"/World/Table/Leg_{i}"
        omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Cylinder",
            prim_path=leg_path,
        )
        leg_prim = stage.GetPrimAtPath(leg_path)
        leg_xform = UsdGeom.Xformable(leg_prim)
        leg_xform.ClearXformOpOrder()
        leg_xform.AddTranslateOp().Set(
            Gf.Vec3d(table_pos[0] + lx, table_pos[1] + ly, table_pos[2] + leg_height / 2.0)
        )
        leg_xform.AddScaleOp().Set(Gf.Vec3d(leg_radius, leg_radius, leg_height / 2.0))
        UsdPhysics.CollisionAPI.Apply(leg_prim)

    return top_surface_z


def _add_robot(stage, robot_usd: str, surface_z: float, table_pos=(0.0, 0.0, 0.0)):
    """
    Reference the robot USD onto the tabletop.
    The robot base is placed at the table surface centre.
    """
    from pxr import Gf, UsdGeom, Sdf

    robot_usd = os.path.abspath(robot_usd)
    robot_prim_path = "/World/Robot"

    # Add as a reference so the scene stays lightweight
    robot_prim = stage.DefinePrim(robot_prim_path)
    robot_prim.GetReferences().AddReference(robot_usd)

    xform = UsdGeom.Xformable(robot_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(table_pos[0], table_pos[1], surface_z)
    )

    print(f"[OK] Robot referenced at {robot_prim_path} (z={surface_z:.3f} m)")
    return robot_prim_path


# ---------------------------------------------------------------------------
# Main scene build
# ---------------------------------------------------------------------------

def build_scene(robot_usd: str, output_usd: str, gui: bool = False):
    app = _launch_app(headless=not gui)

    import omni.usd
    import omni.kit.commands
    from pxr import Usd, UsdGeom, Sdf

    # New empty stage
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    # Metres, Z-up
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # World xform root
    stage.DefinePrim("/World", "Xform")

    _add_physics_scene(stage)
    _add_dome_light(stage)
    _add_ground_plane(stage)

    table_pos = (0.0, 0.0, 0.0)
    surface_z = _add_table(stage, table_pos=table_pos)
    _add_robot(stage, robot_usd, surface_z=surface_z, table_pos=table_pos)

    # Save
    output_usd = os.path.abspath(output_usd)
    stage.GetRootLayer().Export(output_usd)
    print(f"[OK] Scene saved to: {output_usd}")

    if gui:
        # Keep the window open for inspection
        while app.is_running():
            app.update()

    app.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Build Isaac Sim tabletop lab scene")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--robot-usd", help="Pre-converted robot .usd path")
    group.add_argument("--urdf", help="Robot .urdf path (auto-converts to assets/robot.usd)")

    parser.add_argument(
        "--output", default="lab_scene.usd", help="Output scene USD (default: lab_scene.usd)"
    )
    parser.add_argument(
        "--gui", action="store_true", help="Open Isaac Sim GUI after building the scene"
    )
    args = parser.parse_args()

    robot_usd = args.robot_usd

    if args.urdf:
        # Auto-convert URDF → USD before building
        from convert_urdf import convert
        robot_usd = convert(args.urdf, "assets/robot.usd")

    if not os.path.isfile(robot_usd):
        print(f"[ERROR] Robot USD not found: {robot_usd}", file=sys.stderr)
        print("        Run convert_urdf.py first, or pass --urdf directly.", file=sys.stderr)
        sys.exit(1)

    build_scene(robot_usd=robot_usd, output_usd=args.output, gui=args.gui)


if __name__ == "__main__":
    main()

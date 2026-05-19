"""
setup_lab.py  —  Build an open-plan lab scene in Isaac Sim and save it as USD.

Scene layout (top-down):
  - 12 m × 16 m × 3.5 m white room
  - Island lab benches in two central rows
  - Wall-mounted benches along back and side walls
  - Simple equipment props on benches
  - Grid of ceiling strip lights
  - LIO2 robot placed on the floor in the navigation aisle

Usage:
    python setup_lab.py --robot-usd /path/to/LIO2.usd --output lab_scene.usd
    python setup_lab.py --urdf /path/to/robot.urdf  --output lab_scene.usd --gui

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
# Primitive helper
# ---------------------------------------------------------------------------

def _cube(stage, path, translate, scale, color):
    from pxr import UsdGeom, UsdPhysics, Gf
    D = UsdGeom.XformOp.PrecisionDouble
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp(D).Set(Gf.Vec3d(*translate))
    xf.AddScaleOp(D).Set(Gf.Vec3d(*scale))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


# ---------------------------------------------------------------------------
# Scene helpers
# ---------------------------------------------------------------------------

def _add_physics_scene(stage):
    from pxr import UsdPhysics, PhysxSchema, Gf
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    ps.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/PhysicsScene"))


def _add_room(stage, width=12.0, length=16.0, height=3.5):
    """White open-plan lab room centred at origin, floor at z=0."""
    wall_t = 0.15
    hw, hl = width / 2.0, length / 2.0
    WHITE      = (0.96, 0.96, 0.96)
    FLOOR_COL  = (0.93, 0.93, 0.94)   # very light grey-white

    # Floor
    _cube(stage, "/World/Room/Floor",
          translate=(0, 0, -wall_t / 2),
          scale=(width, length, wall_t),
          color=FLOOR_COL)
    # Ceiling
    _cube(stage, "/World/Room/Ceiling",
          translate=(0, 0, height + wall_t / 2),
          scale=(width, length, wall_t),
          color=WHITE)
    # Front (−Y)
    _cube(stage, "/World/Room/WallFront",
          translate=(0, -hl - wall_t / 2, height / 2),
          scale=(width + 2 * wall_t, wall_t, height),
          color=WHITE)
    # Back (+Y)
    _cube(stage, "/World/Room/WallBack",
          translate=(0, hl + wall_t / 2, height / 2),
          scale=(width + 2 * wall_t, wall_t, height),
          color=WHITE)
    # Left (−X)
    _cube(stage, "/World/Room/WallLeft",
          translate=(-hw - wall_t / 2, 0, height / 2),
          scale=(wall_t, length, height),
          color=WHITE)
    # Right (+X)
    _cube(stage, "/World/Room/WallRight",
          translate=(hw + wall_t / 2, 0, height / 2),
          scale=(wall_t, length, height),
          color=WHITE)


def _add_ceiling_lights(stage, room_width=12.0, room_length=16.0, room_height=3.5):
    """Grid of rectangular strip lights covering the ceiling evenly."""
    from pxr import UsdLux, UsdGeom, Gf
    D = UsdGeom.XformOp.PrecisionDouble

    # 3 columns × 4 rows of lights
    xs = [-3.5, 0.0, 3.5]
    ys = [-5.5, -1.8, 1.8, 5.5]
    idx = 0
    for lx in xs:
        for ly in ys:
            path = f"/World/Lights/Strip_{idx}"
            light = UsdLux.RectLight.Define(stage, path)
            light.CreateIntensityAttr(12000.0)
            light.CreateWidthAttr(1.2)
            light.CreateHeightAttr(0.15)
            light.CreateColorAttr(Gf.Vec3f(1.0, 0.99, 0.95))  # cool white
            xf = UsdGeom.Xformable(light)
            xf.AddTranslateOp(D).Set(Gf.Vec3d(lx, ly, room_height - 0.05))
            xf.AddRotateXOp(D).Set(180.0)   # face downward
            idx += 1

    # Soft fill dome to avoid pure-black shadows
    from pxr import UsdLux
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/AmbientFill")
    dome.CreateIntensityAttr(200.0)


def _add_lab_bench(stage, path_prefix, cx, cy, bench_w=1.8, bench_d=0.7,
                   bench_h=0.9, color=(0.97, 0.97, 0.97)):
    """
    Single lab bench (top slab + 4 legs) centred at (cx, cy).
    Returns the top-surface Z.
    """
    from pxr import UsdGeom, UsdPhysics, Gf
    D = UsdGeom.XformOp.PrecisionDouble
    top_t  = 0.04
    top_z  = bench_h - top_t / 2
    leg_h  = bench_h - top_t
    leg_r  = 0.03

    _cube(stage, f"{path_prefix}/Top",
          translate=(cx, cy, top_z),
          scale=(bench_w, bench_d, top_t),
          color=color)

    lx_off = bench_w / 2 - 0.08
    ly_off = bench_d / 2 - 0.06
    for i, (dx, dy) in enumerate([(1,1),(-1,1),(1,-1),(-1,-1)]):
        leg = UsdGeom.Cylinder.Define(stage, f"{path_prefix}/Leg_{i}")
        leg.CreateRadiusAttr(leg_r)
        leg.CreateHeightAttr(leg_h)
        xf = UsdGeom.Xformable(leg)
        xf.AddTranslateOp(D).Set(Gf.Vec3d(cx + dx * lx_off, cy + dy * ly_off, leg_h / 2))
        UsdPhysics.CollisionAPI.Apply(leg.GetPrim())

    return bench_h


def _add_equipment_prop(stage, path, cx, cy, base_z,
                        w=0.3, d=0.25, h=0.35, color=(0.85, 0.87, 0.90)):
    """Small box representing a lab instrument sitting on a bench."""
    _cube(stage, path,
          translate=(cx, cy, base_z + h / 2),
          scale=(w, d, h),
          color=color)


def _add_monitor_prop(stage, path, cx, cy, base_z, color=(0.15, 0.15, 0.15)):
    """Thin upright slab representing a monitor."""
    _cube(stage, path,
          translate=(cx, cy, base_z + 0.25),
          scale=(0.5, 0.04, 0.35),
          color=color)


def _add_lab_furniture(stage):
    """
    Two rows of central island benches + wall benches along back and side walls.
    Equipment props scattered on bench tops.
    """
    BENCH = (0.97, 0.97, 0.97)   # white bench top
    EQUIP = (0.80, 0.85, 0.90)   # light-blue-grey instrument

    # --- Central island rows (two rows, 3 benches each) ---
    island_ys = [-5.0, -1.5, 2.5]   # bench centres along Y
    for col, (ix, sign) in enumerate([(-3.0, -1), (3.0, 1)]):
        for row, iy in enumerate(island_ys):
            prefix = f"/World/Furniture/Island_{col}_{row}"
            bz = _add_lab_bench(stage, prefix, ix, iy,
                                bench_w=1.8, bench_d=0.75, color=BENCH)
            # 2 instrument props per bench
            _add_equipment_prop(stage, f"{prefix}/Equip_A",
                                ix - 0.45, iy, bz, color=EQUIP)
            _add_equipment_prop(stage, f"{prefix}/Equip_B",
                                ix + 0.45, iy, bz, w=0.25, d=0.2, h=0.25, color=EQUIP)

    # --- Back wall benches (y ≈ +7.0, along full width) ---
    back_y = 7.0
    for col, bx in enumerate([-4.0, -1.5, 1.5, 4.0]):
        prefix = f"/World/Furniture/BackBench_{col}"
        bz = _add_lab_bench(stage, prefix, bx, back_y,
                            bench_w=1.6, bench_d=0.65, color=BENCH)
        _add_monitor_prop(stage, f"{prefix}/Monitor", bx, back_y - 0.1, bz)
        _add_equipment_prop(stage, f"{prefix}/Equip",
                            bx + 0.5, back_y, bz, w=0.2, d=0.18, h=0.3,
                            color=EQUIP)

    # --- Right side wall bench (x ≈ +5.2, 2 benches) ---
    side_x = 5.2
    for row, sy in enumerate([-3.5, 2.0]):
        prefix = f"/World/Furniture/SideBench_{row}"
        bz = _add_lab_bench(stage, prefix, side_x, sy,
                            bench_w=0.7, bench_d=1.6, color=BENCH)
        _add_equipment_prop(stage, f"{prefix}/Equip",
                            side_x, sy, bz, w=0.25, d=0.4, h=0.4, color=EQUIP)


def _add_bruker_nmr(stage, cx=4.5, cy=6.5):
    """
    Bruker NMR spectrometer model placed at (cx, cy) — back-right corner.

    Parts:
      - Floor platform (short cylinder)
      - Main magnet body (tall cylinder, cream-white)
      - Top / bottom flanges (flat cylinders)
      - Central bore cap (small dark cylinder on top)
      - Console unit (tall dark-grey box, offset to the side)
      - Connecting cable tray (thin flat box)
    """
    from pxr import UsdGeom, UsdPhysics, Gf
    D = UsdGeom.XformOp.PrecisionDouble

    CREAM  = (0.95, 0.93, 0.88)   # Bruker off-white
    DARK   = (0.18, 0.18, 0.20)   # console / bore
    GREY   = (0.60, 0.60, 0.62)   # flanges
    base   = "/World/NMR"

    def _cyl(path, radius, height, tz, color):
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateRadiusAttr(radius)
        c.CreateHeightAttr(height)
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xf = UsdGeom.Xformable(c)
        xf.AddTranslateOp(D).Set(Gf.Vec3d(cx, cy, tz))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())

    # Floor platform
    _cyl(f"{base}/Platform",   radius=0.52, height=0.08,  tz=0.04,  color=GREY)
    # Bottom flange
    _cyl(f"{base}/FlangeBot",  radius=0.50, height=0.12,  tz=0.14,  color=GREY)
    # Main magnet cylinder
    _cyl(f"{base}/Magnet",     radius=0.42, height=1.50,  tz=0.95,  color=CREAM)
    # Top flange
    _cyl(f"{base}/FlangeTop",  radius=0.50, height=0.12,  tz=1.76,  color=GREY)
    # Top cap / bore opening (dark)
    _cyl(f"{base}/BoreCap",    radius=0.14, height=0.10,  tz=1.87,  color=DARK)

    # Console unit — tall box offset ~1.1 m to the left (-X)
    con_x = cx - 1.1
    _cube(stage, f"{base}/Console",
          translate=(con_x, cy, 0.70),
          scale=(0.55, 0.45, 0.70),
          color=DARK)
    # Console display panel (thin light-blue strip on front face)
    _cube(stage, f"{base}/Display",
          translate=(con_x, cy - 0.23, 0.85),
          scale=(0.35, 0.02, 0.20),
          color=(0.55, 0.75, 0.95))

    # Cable tray connecting console to magnet base
    _cube(stage, f"{base}/CableTray",
          translate=(cx - 0.55, cy, 0.06),
          scale=(0.55, 0.10, 0.04),
          color=GREY)

    print(f"[OK] Bruker NMR placed at ({cx}, {cy})")


def _add_robot(stage, robot_usd: str, pos=(0.0, -3.0, 0.0)):
    """Place the robot on the floor in the navigation aisle."""
    from pxr import Gf, UsdGeom
    D = UsdGeom.XformOp.PrecisionDouble

    robot_usd = os.path.abspath(robot_usd)
    prim = stage.DefinePrim("/World/Robot")
    prim.GetReferences().AddReference(robot_usd)

    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(D).Set(Gf.Vec3d(*pos))

    print(f"[OK] Robot placed at /World/Robot  pos={pos}")


# ---------------------------------------------------------------------------
# Main scene build
# ---------------------------------------------------------------------------

def build_scene(robot_usd: str, output_usd: str, gui: bool = False):
    app = _launch_app(headless=not gui)

    import omni.usd
    from pxr import UsdGeom

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")

    _add_physics_scene(stage)
    _add_room(stage, width=12.0, length=16.0, height=3.5)
    _add_ceiling_lights(stage, room_width=12.0, room_length=16.0, room_height=3.5)
    _add_lab_furniture(stage)
    _add_bruker_nmr(stage, cx=4.5, cy=6.5)   # back-right corner
    _add_robot(stage, robot_usd, pos=(0.0, -3.0, 0.0))

    output_usd = os.path.abspath(output_usd)
    stage.GetRootLayer().Export(output_usd)
    print(f"[OK] Scene saved to: {output_usd}")

    if gui:
        while app.is_running():
            app.update()

    app.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Build Isaac Sim open-plan lab scene")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--robot-usd", help="Pre-converted robot .usd path")
    group.add_argument("--urdf", help="Robot .urdf path (auto-converts to assets/robot.usd)")
    parser.add_argument("--output", default="lab_scene.usd")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    robot_usd = args.robot_usd
    if args.urdf:
        from convert_urdf import convert
        robot_usd = convert(args.urdf, "assets/robot.usd")

    if not os.path.isfile(robot_usd):
        print(f"[ERROR] Robot USD not found: {robot_usd}", file=sys.stderr)
        sys.exit(1)

    build_scene(robot_usd=robot_usd, output_usd=args.output, gui=args.gui)


if __name__ == "__main__":
    main()

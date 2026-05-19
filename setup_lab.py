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


def _add_hamilton_liquid_handler(stage, path, cx, cy, bz):
    """
    Hamilton STAR liquid handler — wide platform with a raised gantry rail and
    a pipetting arm that rides along it.
    """
    from pxr import UsdGeom, Gf
    D = UsdGeom.XformOp.PrecisionDouble
    WHITE = (0.95, 0.95, 0.95)
    GREY  = (0.55, 0.58, 0.62)
    DARK  = (0.20, 0.22, 0.25)

    # Deck / base platform
    _cube(stage, f"{path}/Deck",
          translate=(cx, cy, bz + 0.06), scale=(0.80, 0.55, 0.06), color=WHITE)
    # Left gantry post
    _cube(stage, f"{path}/PostL",
          translate=(cx - 0.36, cy, bz + 0.25), scale=(0.04, 0.06, 0.25), color=GREY)
    # Right gantry post
    _cube(stage, f"{path}/PostR",
          translate=(cx + 0.36, cy, bz + 0.25), scale=(0.04, 0.06, 0.25), color=GREY)
    # Horizontal rail
    _cube(stage, f"{path}/Rail",
          translate=(cx, cy, bz + 0.48), scale=(0.76, 0.04, 0.03), color=GREY)
    # Pipetting arm (rides on rail)
    _cube(stage, f"{path}/Arm",
          translate=(cx - 0.10, cy - 0.01, bz + 0.44), scale=(0.06, 0.50, 0.05), color=DARK)
    # Tip waste box (front-right corner of deck)
    _cube(stage, f"{path}/TipBox",
          translate=(cx + 0.28, cy - 0.18, bz + 0.14), scale=(0.14, 0.14, 0.08), color=GREY)


def _add_bruker_maldi(stage, path, cx, cy, bz):
    """
    Bruker rapifleX MALDI mass spectrometer — compact box with sample inlet,
    front panel display, and venting grilles.
    """
    CREAM = (0.94, 0.92, 0.88)
    DARK  = (0.18, 0.18, 0.20)
    BLUE  = (0.40, 0.60, 0.90)

    # Main body
    _cube(stage, f"{path}/Body",
          translate=(cx, cy, bz + 0.20), scale=(0.48, 0.38, 0.20), color=CREAM)
    # Front display strip
    _cube(stage, f"{path}/Display",
          translate=(cx, cy - 0.195, bz + 0.26), scale=(0.30, 0.02, 0.08), color=BLUE)
    # Sample inlet port (small dark cylinder on right side)
    from pxr import UsdGeom, UsdPhysics, Gf
    D = UsdGeom.XformOp.PrecisionDouble
    port = UsdGeom.Cylinder.Define(stage, f"{path}/InletPort")
    port.CreateRadiusAttr(0.025)
    port.CreateHeightAttr(0.06)
    port.CreateDisplayColorAttr([Gf.Vec3f(*DARK)])
    xf = UsdGeom.Xformable(port)
    xf.AddTranslateOp(D).Set(Gf.Vec3d(cx + 0.27, cy, bz + 0.22))
    xf.AddRotateYOp(D).Set(90.0)
    UsdPhysics.CollisionAPI.Apply(port.GetPrim())
    # Vent grille strip (top)
    _cube(stage, f"{path}/Vent",
          translate=(cx, cy, bz + 0.41), scale=(0.44, 0.34, 0.01), color=(0.70, 0.70, 0.70))


def _add_bruker_plate_reader(stage, path, cx, cy, bz):
    """
    Bruker microplate reader — flat body with a plate drawer slot on the front.
    """
    CREAM = (0.94, 0.92, 0.88)
    SLOT  = (0.15, 0.15, 0.15)
    BLUE  = (0.40, 0.60, 0.90)

    # Main body
    _cube(stage, f"{path}/Body",
          translate=(cx, cy, bz + 0.12), scale=(0.42, 0.36, 0.12), color=CREAM)
    # Plate drawer slot
    _cube(stage, f"{path}/Slot",
          translate=(cx, cy - 0.185, bz + 0.10), scale=(0.22, 0.02, 0.04), color=SLOT)
    # Status LED
    _cube(stage, f"{path}/LED",
          translate=(cx + 0.17, cy - 0.185, bz + 0.15), scale=(0.02, 0.01, 0.02),
          color=(0.20, 0.90, 0.30))
    # Top lid (slightly raised)
    _cube(stage, f"{path}/Lid",
          translate=(cx, cy, bz + 0.245), scale=(0.40, 0.34, 0.01), color=(0.88, 0.88, 0.85))


def _add_centrifuge(stage, path, cx, cy, bz):
    """
    Benchtop centrifuge — round body, hinged lid, status panel on front.
    """
    from pxr import UsdGeom, UsdPhysics, Gf
    D = UsdGeom.XformOp.PrecisionDouble
    CREAM = (0.93, 0.91, 0.88)
    DARK  = (0.22, 0.22, 0.24)

    # Body cylinder
    body = UsdGeom.Cylinder.Define(stage, f"{path}/Body")
    body.CreateRadiusAttr(0.18)
    body.CreateHeightAttr(0.22)
    body.CreateDisplayColorAttr([Gf.Vec3f(*CREAM)])
    xf = UsdGeom.Xformable(body)
    xf.AddTranslateOp(D).Set(Gf.Vec3d(cx, cy, bz + 0.14))
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())

    # Lid (flat cylinder on top)
    lid = UsdGeom.Cylinder.Define(stage, f"{path}/Lid")
    lid.CreateRadiusAttr(0.175)
    lid.CreateHeightAttr(0.03)
    lid.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.85, 0.83)])
    xf2 = UsdGeom.Xformable(lid)
    xf2.AddTranslateOp(D).Set(Gf.Vec3d(cx, cy, bz + 0.265))
    UsdPhysics.CollisionAPI.Apply(lid.GetPrim())

    # Front panel strip
    _cube(stage, f"{path}/Panel",
          translate=(cx, cy - 0.185, bz + 0.10),
          scale=(0.20, 0.01, 0.06), color=DARK)


def _add_lab_furniture(stage):
    """
    Two rows of central island benches + wall benches.
    Each bench gets a specific lab instrument instead of generic cubes.
    """
    BENCH = (0.97, 0.97, 0.97)

    # --- Central island rows (two rows, 3 benches each) ---
    # Instrument assignment per (col, row):
    #   col 0 (left,  x=-3): Hamilton | Bruker MALDI    | centrifuge
    #   col 1 (right, x=+3): plate reader | Hamilton    | Bruker MALDI
    island_ys = [-5.0, -1.5, 2.5]
    instruments_left  = [_add_hamilton_liquid_handler, _add_bruker_maldi,    _add_centrifuge]
    instruments_right = [_add_bruker_plate_reader,     _add_hamilton_liquid_handler, _add_bruker_maldi]

    for col, (ix, inst_list) in enumerate([(-3.0, instruments_left),
                                            ( 3.0, instruments_right)]):
        for row, (iy, inst_fn) in enumerate(zip(island_ys, inst_list)):
            prefix = f"/World/Furniture/Island_{col}_{row}"
            bz = _add_lab_bench(stage, prefix, ix, iy,
                                bench_w=1.8, bench_d=0.75, color=BENCH)
            inst_fn(stage, f"{prefix}/Instrument", ix, iy, bz)

    # --- Back wall benches (y ≈ +7.0) — x=-4.0 removed (NMR there) and x=4.0 removed by user ---
    back_y = 7.0
    for col, bx in enumerate([-1.5, 1.5]):
        prefix = f"/World/Furniture/BackBench_{col}"
        bz = _add_lab_bench(stage, prefix, bx, back_y,
                            bench_w=1.6, bench_d=0.65, color=BENCH)
        _add_monitor_prop(stage, f"{prefix}/Monitor", bx, back_y - 0.1, bz)
        _add_bruker_plate_reader(stage, f"{prefix}/PlateReader", bx + 0.45, back_y, bz)

    # --- Right side wall benches (x ≈ +5.2) ---
    side_x = 5.2
    for row, (sy, inst_fn) in enumerate(zip([-3.5, 2.0],
                                             [_add_centrifuge, _add_bruker_maldi])):
        prefix = f"/World/Furniture/SideBench_{row}"
        bz = _add_lab_bench(stage, prefix, side_x, sy,
                            bench_w=0.7, bench_d=1.6, color=BENCH)
        inst_fn(stage, f"{prefix}/Instrument", side_x, sy, bz)


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


def _add_default_camera(stage):
    """
    Define a perspective camera inside the room and set it as the active viewport
    camera so the scene opens from a useful viewpoint.
    Position: front-centre, 1.7 m high, looking toward the back of the lab.
    """
    from pxr import UsdGeom, Gf
    D = UsdGeom.XformOp.PrecisionDouble

    cam = UsdGeom.Camera.Define(stage, "/World/LabCamera")
    cam.CreateFocalLengthAttr(18.0)          # wide-angle
    cam.CreateHorizontalApertureAttr(36.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))

    xf = UsdGeom.Xformable(cam)
    # Place at front-centre, eye height, facing into the lab (+Y direction)
    xf.AddTranslateOp(D).Set(Gf.Vec3d(0.0, -7.0, 1.7))
    # Rotate: -90° around X puts the camera looking along +Y (Isaac Sim convention)
    xf.AddRotateXYZOp(D).Set(Gf.Vec3d(90.0, 0.0, 0.0))

    # Try to activate this camera as the viewport default
    try:
        import omni.kit.viewport.utility as vp_util
        vp = vp_util.get_active_viewport()
        if vp:
            vp.camera_path = "/World/LabCamera"
    except Exception:
        pass  # viewport API unavailable in pure headless mode

    print("[OK] Default lab camera set at (0, -7, 1.7) facing +Y")


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
    _add_bruker_nmr(stage, cx=-4.5, cy=6.8)  # back-left corner (bench slot removed)
    _add_default_camera(stage)
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

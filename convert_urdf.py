"""
convert_urdf.py  —  Convert a URDF file to a USD asset for Isaac Sim.

Usage:
    python convert_urdf.py --urdf path/to/robot.urdf --output assets/robot.usd

Compatible with Isaac Sim 2023.x and 4.x.
"""

import argparse
import os
import sys


def _detect_isaac_version() -> int:
    """Return 4 for Isaac Sim 4.x, 3 for 2023.x."""
    try:
        import isaacsim  # noqa: F401
        return 4
    except ImportError:
        return 3


def _launch_app(headless: bool = True):
    if _detect_isaac_version() == 4:
        from isaacsim import SimulationApp
    else:
        from omni.isaac.kit import SimulationApp
    return SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})


def convert(urdf_path: str, output_usd: str, merge_fixed_joints: bool = False) -> str:
    """
    Import *urdf_path* and write the result to *output_usd*.
    Returns the absolute path of the written USD file.
    """
    urdf_path = os.path.abspath(urdf_path)
    output_usd = os.path.abspath(output_usd)
    os.makedirs(os.path.dirname(output_usd) or ".", exist_ok=True)

    app = _launch_app(headless=True)

    import omni.kit.commands

    isaac_ver = _detect_isaac_version()

    if isaac_ver == 4:
        # Isaac Sim 4.x extension name
        import_ext = "isaacsim.asset.importer.urdf"
    else:
        # Isaac Sim 2023.x extension name
        import_ext = "omni.isaac.urdf"

    # Ensure the URDF importer extension is loaded
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate(import_ext, True)

    # Build import config
    result, config = omni.kit.commands.execute("URDFCreateImportConfig")
    config.merge_fixed_joints = merge_fixed_joints
    config.convex_decomp = False
    config.import_inertia_tensor = True
    config.fix_base = False          # set True if the robot is base-fixed (e.g. arm on table)
    config.make_default_prim = True
    config.create_physics_scene = False  # physics scene is created by setup_lab.py

    # Run the import
    result, robot_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=config,
        dest_path=output_usd,
    )

    if not result:
        print(f"[ERROR] URDF import failed for: {urdf_path}", file=sys.stderr)
        app.close()
        sys.exit(1)

    print(f"[OK] Robot USD written to: {output_usd}")
    print(f"     Default prim path   : {robot_prim_path}")

    app.close()
    return output_usd


def main():
    parser = argparse.ArgumentParser(description="Convert URDF → USD for Isaac Sim")
    parser.add_argument("--urdf", required=True, help="Path to input .urdf file")
    parser.add_argument(
        "--output", default="assets/robot.usd", help="Destination .usd path (default: assets/robot.usd)"
    )
    parser.add_argument(
        "--merge-fixed-joints",
        action="store_true",
        help="Merge fixed joints (reduces articulation complexity)",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.urdf):
        print(f"[ERROR] URDF not found: {args.urdf}", file=sys.stderr)
        sys.exit(1)

    convert(args.urdf, args.output, merge_fixed_joints=args.merge_fixed_joints)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Remove Link gripper joints from USD file using USD Python API.
This script properly initializes Isaac Sim environment first.

Usage:
  ~/Projects/IsaacLab/isaaclab.sh -p remove_link_joints_from_usd.py --headless
"""

import argparse
from pathlib import Path
import sys

# Import AppLauncher first (MUST be before pxr!)
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Remove Link joints from USD")
parser.add_argument("--input", type=str,
                    default="~/Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms.usd",
                    help="Input USD file")
parser.add_argument("--output", type=str,
                    default="~/Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms_nubs.usd",
                    help="Output USD file")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

# Initialize Isaac Sim FIRST (required before importing pxr!)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


def expand_path(path_str):
    """Expand ~ and make absolute."""
    return str(Path(path_str).expanduser().resolve())


def main():
    """Main function - pxr imported here AFTER Isaac Sim is ready."""
    # Import pxr INSIDE this function, after Isaac Sim is initialized
    from pxr import Usd, UsdPhysics

    input_usd = expand_path(args_cli.input)
    output_usd = expand_path(args_cli.output)

    print("=" * 80)
    print("Removing Link Gripper Joints from USD")
    print("=" * 80)
    print(f"\nInput:  {input_usd}")
    print(f"Output: {output_usd}")

    if not Path(input_usd).exists():
        print(f"\n❌ ERROR: Input file not found: {input_usd}")
        simulation_app.close()
        sys.exit(1)

    # Open USD stage
    print("\n[1/4] Opening USD stage...")
    stage = Usd.Stage.Open(input_usd)
    if not stage:
        print("❌ Failed to open USD stage")
        simulation_app.close()
        sys.exit(1)
    print("✅ USD loaded")

    # Find robot root
    print("\n[2/4] Finding robot root...")
    root_paths = ["/World/t1", "/t1", "/World"]
    robot_root = None
    for path_str in root_paths:
        prim = stage.GetPrimAtPath(path_str)
        if prim.IsValid():
            robot_root = prim
            print(f"✅ Found robot at: {path_str}")
            break

    if not robot_root:
        # Search for any prim with 't1' in name
        for prim in stage.Traverse():
            if 't1' in str(prim.GetPath()).lower():
                robot_root = prim
                print(f"✅ Found robot at: {prim.GetPath()}")
                break

    if not robot_root:
        print("❌ Could not find robot root in USD")
        simulation_app.close()
        sys.exit(1)

    # Collect Link prims to remove
    print("\n[3/4] Scanning for Link joints...")
    link_keywords = ['Link1', 'Link11', 'Link2', 'Link22']
    prims_to_remove = []

    def find_link_prims(prim):
        """Recursively find Link-related prims."""
        prim_name = prim.GetName()

        # Check if this prim name contains Link keywords
        for keyword in link_keywords:
            if keyword in prim_name:
                prims_to_remove.append(prim.GetPath())
                print(f"  Found: {prim.GetPath()}")
                return  # Don't traverse children

        # Continue searching children
        for child in prim.GetChildren():
            find_link_prims(child)

    find_link_prims(robot_root)

    if not prims_to_remove:
        print("⚠️  WARNING: No Link prims found!")
        print("   The USD may already be clean, or Link joints are named differently")
    else:
        print(f"\n[4/4] Removing {len(prims_to_remove)} Link prims...")
        for prim_path in prims_to_remove:
            stage.RemovePrim(prim_path)
            print(f"  ✅ Removed: {prim_path}")

    # Save modified USD
    print(f"\n💾 Saving to: {output_usd}")
    Path(output_usd).parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(output_usd))

    if Path(output_usd).exists():
        size_mb = Path(output_usd).stat().st_size / 1024 / 1024
        print(f"✅ Created: {output_usd} ({size_mb:.1f} MB)")

        print("\n" + "=" * 80)
        print("✅ COMPLETE!")
        print("=" * 80)
        print(f"\nRemoved {len(prims_to_remove)} Link gripper joints")
        print(f"New file: {output_usd}")
        print("\nVerify with:")
        print(f"  cd ~/Minerva/wb_manip")
        print(f"  ~/Projects/IsaacLab/isaaclab.sh -p scripts/debug_joints_loaded.py --robot booster_t1_7dof_nubs")
    else:
        print("❌ Failed to create output file")

    simulation_app.close()


if __name__ == "__main__":
    main()

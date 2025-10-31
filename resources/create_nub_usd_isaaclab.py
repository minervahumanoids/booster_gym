#!/usr/bin/env python3
"""
Create a finger-free USD from the articulated hands USD using Isaac Lab.
This removes all finger joints and links to speed up physics simulation.

Usage:
    ~/Projects/IsaacLab/isaaclab.sh -p create_nub_usd_isaaclab.py -- --headless
"""

from isaaclab.app import AppLauncher
import argparse
from pathlib import Path
import sys

# Add Isaac Lab to path
isaaclab_path = Path.home() / "Projects/IsaacLab/source/isaaclab"
if isaaclab_path.exists():
    sys.path.insert(0, str(isaaclab_path))


# Parse arguments
parser = argparse.ArgumentParser(description="Create finger-free USD")
# Note: Don't add --headless here - AppLauncher.add_app_launcher_args() adds it automatically
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# Force headless mode for USD creation (don't need GUI)
args_cli.headless = True

# Launch Isaac Sim (MUST be done before importing pxr!)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# DO NOT import pxr here at module level - import it inside functions instead


def remove_finger_prims(stage, root_path="/World/t1"):
    """Remove all finger-related prims from the USD stage."""

    # Keywords to identify finger/gripper prims (joints, links, visuals)
    finger_keywords = [
        'thumb', 'index', 'middle', 'ring', 'pinky',
        'proximal', 'distal', 'intermediate',
        'Link1', 'Link11', 'Link2', 'Link22',  # Gripper joints that should be removed
        'link1', 'link11', 'link2', 'link22',  # Case variations
        'left_Link', 'right_Link'  # Also match left_Link1, right_Link11, etc.
    ]

    removed_count = 0
    root_prim = stage.GetPrimAtPath(root_path)

    if not root_prim.IsValid():
        # Try to find the robot root
        for prim in stage.Traverse():
            if 't1' in str(prim.GetPath()).lower() or 'robot' in str(prim.GetPath()).lower():
                root_prim = prim
                root_path = str(prim.GetPath())
                print(f"[INFO] Found robot at: {root_path}")
                break

        if not root_prim.IsValid():
            print(f"[ERROR] Could not find robot in USD")
            return removed_count

    # Collect all prims to remove (can't modify while iterating)
    prims_to_remove = []

    def traverse(prim):
        """Recursively traverse and collect finger prims."""
        prim_name = prim.GetName()
        prim_path = str(prim.GetPath())

        # Check if this prim is finger-related
        is_finger = any(keyword.lower() in prim_name.lower()
                        for keyword in finger_keywords)

        if is_finger:
            prims_to_remove.append(prim.GetPath())
            print(f"  Marking for removal: {prim_path}")
            # Don't traverse children of finger prims
        else:
            # Only traverse children if this isn't a finger prim
            for child in prim.GetChildren():
                traverse(child)

    print("\n[INFO] Scanning for finger prims...")
    traverse(root_prim)

    # Remove marked prims
    print(f"\n[INFO] Removing {len(prims_to_remove)} finger prims...")
    for prim_path in prims_to_remove:
        stage.RemovePrim(prim_path)
        removed_count += 1

    return removed_count


def main():
    # Import pxr HERE, after Isaac Sim is fully initialized
    from pxr import Usd, UsdPhysics, UsdGeom

    # Paths
    # Start with the USD that has Link grippers and remove them
    input_usd = Path.home() / "Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms.usd"
    output_usd = Path.home() / "Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms_nubs.usd"

    print("=" * 80)
    print("  Creating Finger-Free USD for Fast Simulation")
    print("=" * 80)
    print(f"\nInput:  {input_usd}")
    print(f"Output: {output_usd}")

    # Check input exists
    if not input_usd.exists():
        print(f"\n[ERROR] Input USD not found: {input_usd}")
        simulation_app.close()
        sys.exit(1)

    # Load USD
    print("\n[INFO] Loading USD...")
    stage = Usd.Stage.Open(str(input_usd))

    if not stage:
        print("[ERROR] Failed to open USD stage")
        simulation_app.close()
        sys.exit(1)

    print(f"[INFO] USD loaded successfully")

    # List all prims to understand structure
    print("\n[INFO] USD structure (first 50 prims):")
    for i, prim in enumerate(stage.Traverse()):
        if i < 50:
            print(f"  {prim.GetPath()}")
        else:
            print(f"  ... and {sum(1 for _ in stage.Traverse()) - 50} more")
            break

    # Remove finger prims
    removed_count = remove_finger_prims(stage, "/World/t1")

    if removed_count == 0:
        print(
            "\n[WARNING] No finger prims were removed. Check finger_keywords or USD structure.")
    else:
        print(f"\n[SUCCESS] Removed {removed_count} finger prims")

    # Save modified USD
    print(f"\n[INFO] Saving modified USD to: {output_usd}")
    output_usd.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(output_usd))

    # Check file size
    if output_usd.exists():
        size_mb = output_usd.stat().st_size / 1024 / 1024
        print(f"[INFO] New USD size: {size_mb:.1f} MB")

    print("\n" + "=" * 80)
    print("  USD Creation Complete!")
    print("=" * 80)
    print(f"\nNew USD saved: {output_usd}")
    print("\nNext steps:")
    print("  1. Update robot_configs.py to use the new USD")
    print("  2. Test with: ./run.sh scripts/test_visualization.py")
    print()

    simulation_app.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Create a finger-free USD from the articulated hands USD.
This removes all finger joints and links to speed up physics simulation.

Usage:
    ~/Projects/IsaacLab/_isaac_sim/python.sh create_nub_usd.py
"""

from pathlib import Path
from pxr import Usd, UsdPhysics, UsdGeom
import sys


def remove_finger_prims(stage, root_path="/World/t1"):
    """Remove all finger-related prims from the USD stage."""

    # Keywords to identify finger prims (joints, links, visuals)
    finger_keywords = [
        'thumb', 'index', 'middle', 'ring', 'pinky',
        'L_thumb', 'L_index', 'L_middle', 'L_ring', 'L_pinky',
        'R_thumb', 'R_index', 'R_middle', 'R_ring', 'R_pinky'
    ]

    removed_count = 0
    root_prim = stage.GetPrimAtPath(root_path)

    if not root_prim.IsValid():
        print(f"[ERROR] Root prim not found at {root_path}")
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
            prims_to_remove.append(prim)
            print(f"  Marking for removal: {prim_path}")
        else:
            # Only traverse children if this isn't a finger prim
            for child in prim.GetChildren():
                traverse(child)

    print("\n[INFO] Scanning for finger prims...")
    traverse(root_prim)

    # Remove marked prims
    print(f"\n[INFO] Removing {len(prims_to_remove)} finger prims...")
    for prim in prims_to_remove:
        stage.RemovePrim(prim.GetPath())
        removed_count += 1

    return removed_count


def main():
    # Paths
    input_usd = Path.home() / \
        "Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms_hand_1.usd"
    output_usd = Path.home() / "Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms_nubs.usd"

    print("=" * 80)
    print("  Creating Finger-Free USD for Fast Simulation")
    print("=" * 80)
    print(f"\nInput:  {input_usd}")
    print(f"Output: {output_usd}")

    # Check input exists
    if not input_usd.exists():
        print(f"\n[ERROR] Input USD not found: {input_usd}")
        sys.exit(1)

    # Load USD
    print("\n[INFO] Loading USD...")
    stage = Usd.Stage.Open(str(input_usd))

    if not stage:
        print("[ERROR] Failed to open USD stage")
        sys.exit(1)

    print(f"[INFO] USD loaded successfully")

    # Find the robot root (might be at different paths)
    possible_roots = ["/World/t1", "/t1", "/T1", "/robot"]
    robot_root = None

    for root_path in possible_roots:
        if stage.GetPrimAtPath(root_path).IsValid():
            robot_root = root_path
            print(f"[INFO] Found robot at: {robot_root}")
            break

    if not robot_root:
        print("[WARNING] Could not find robot root, trying default /World/t1")
        robot_root = "/World/t1"

    # Remove finger prims
    removed_count = remove_finger_prims(stage, robot_root)

    if removed_count == 0:
        print(
            "\n[WARNING] No finger prims were removed. Check finger_keywords or USD structure.")
    else:
        print(f"\n[SUCCESS] Removed {removed_count} finger prims")

    # Save modified USD
    print(f"\n[INFO] Saving modified USD to: {output_usd}")
    output_usd.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(output_usd))

    print("\n" + "=" * 80)
    print("  USD Creation Complete!")
    print("=" * 80)
    print(f"\nNew USD saved: {output_usd}")
    print(f"Size: {output_usd.stat().st_size / 1024 / 1024:.1f} MB")
    print("\nNext steps:")
    print("  1. Update robot_configs.py to use the new USD")
    print("  2. Test with: ./run.sh scripts/test_visualization.py")
    print()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Simplified USD creation script that uses Isaac Sim's Python directly.
Run this with: ~/Projects/IsaacLab/_isaac_sim/python.sh create_nub_usd_simple.py
"""

import sys
from pathlib import Path

# Add Isaac Sim's Python packages
isaac_sim_python = Path.home() / "Projects/IsaacLab/_isaac_sim/python"
if isaac_sim_python.exists():
    python_lib = isaac_sim_python / "lib" / "python3.11" / "site-packages"
    if python_lib.exists():
        sys.path.insert(0, str(python_lib))

try:
    from pxr import Usd, UsdPhysics, UsdGeom
except ImportError as e:
    print(f"❌ ERROR: Cannot import pxr: {e}")
    print("\nTry running with:")
    print("  ~/Projects/IsaacLab/_isaac_sim/python.sh create_nub_usd_simple.py")
    sys.exit(1)


def remove_link_joints(stage, root_path="/World/t1"):
    """Remove Link gripper joints from USD."""

    removed_count = 0
    root_prim = stage.GetPrimAtPath(root_path)

    if not root_prim.IsValid():
        # Find robot root
        for prim in stage.Traverse():
            path_str = str(prim.GetPath())
            if 't1' in path_str.lower() and 'world' not in path_str.lower():
                root_prim = prim
                root_path = str(prim.GetPath())
                print(f"[INFO] Found robot at: {root_path}")
                break

    if not root_prim.IsValid():
        print(f"[ERROR] Could not find robot root in USD")
        return removed_count

    # Keywords for Link gripper joints
    link_keywords = ['Link1', 'Link11', 'Link2',
                     'Link22', 'link1', 'link11', 'link2', 'link22']

    # Collect prims to remove
    prims_to_remove = []

    def traverse(prim):
        prim_name = prim.GetName()
        prim_path = str(prim.GetPath())

        # Check if this is a Link joint/link
        is_link = any(kw in prim_name for kw in link_keywords)

        if is_link:
            prims_to_remove.append(prim.GetPath())
            print(f"  Marking for removal: {prim_path}")
            # Don't traverse children
        else:
            for child in prim.GetChildren():
                traverse(child)

    print("\n[INFO] Scanning for Link gripper joints...")
    traverse(root_prim)

    print(f"\n[INFO] Removing {len(prims_to_remove)} Link prims...")
    for prim_path in prims_to_remove:
        stage.RemovePrim(prim_path)
        removed_count += 1

    return removed_count


def main():
    input_usd = Path.home() / "Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms.usd"
    output_usd = Path.home() / "Minerva/booster_gym/resources/T1_usd/t1/t1_with_7dof_arms_nubs.usd"

    print("=" * 80)
    print("  Creating Nub USD (Removing Link Gripper Joints)")
    print("=" * 80)
    print(f"\nInput:  {input_usd}")
    print(f"Output: {output_usd}")

    if not input_usd.exists():
        print(f"\n[ERROR] Input USD not found!")
        sys.exit(1)

    print("\n[INFO] Loading USD...")
    stage = Usd.Stage.Open(str(input_usd))

    if not stage:
        print("[ERROR] Failed to open USD")
        sys.exit(1)

    print("[INFO] USD loaded successfully")

    # Remove Link joints
    removed = remove_link_joints(stage, "/World/t1")

    if removed == 0:
        print("\n⚠️  WARNING: No Link joints were removed!")
        print("   The USD may not have Link joints, or they're named differently")
    else:
        print(f"\n✅ SUCCESS: Removed {removed} Link gripper prims")

    # Save
    print(f"\n[INFO] Saving to: {output_usd}")
    output_usd.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(output_usd))

    if output_usd.exists():
        size_mb = output_usd.stat().st_size / 1024 / 1024
        print(f"[INFO] Created USD: {size_mb:.1f} MB")

    print("\n" + "=" * 80)
    print("✅ COMPLETE!")
    print("=" * 80)
    print(f"\nNew file: {output_usd}")
    print("\nTest with:")
    print("  cd ~/Minerva/wb_manip")
    print("  ~/Projects/IsaacLab/isaaclab.sh -p scripts/debug_joints_loaded.py --robot booster_t1_7dof_nubs")
    print()


if __name__ == "__main__":
    main()

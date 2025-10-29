#!/usr/bin/env python3
"""Merge Booster T1 (7-DOF arms) with RH56DFTP articulated hands.

This creates a robot with:
- 7-DOF arms (Shoulder Pitch/Roll/Yaw + Elbow Pitch/Yaw + Wrist Pitch/Roll)
- RH56DFTP 5-finger dexterous hands (24 DOF total for both hands)
- 13 leg/waist joints
- 2 head joints
= 56 DOF total (13 legs/waist + 2 head + 14 arms (7×2) + 3 grippers removed + 24 hands)

We'll use the pre-converted USD for the 7-DOF arms as the base,
then attach the RH56DFTP hands by removing the gripper links.
"""

import xml.etree.ElementTree as ET
import os
import shutil
from pathlib import Path

# Paths
SCRIPT_DIR = Path(__file__).parent

# We need to convert the USD back to URDF, or find a 7-DOF URDF
# For now, let's check if there's a 7-DOF URDF without grippers
T1_7DOF_URDF = SCRIPT_DIR / "T1_7_dof_arm_serial.urdf"

LEFT_HAND_URDF = SCRIPT_DIR / "RH56DFTP" / "urdf_left_with_force_sensor" / \
    "urdf" / "urdf_left_with_force_sensor.urdf"
RIGHT_HAND_URDF = SCRIPT_DIR / "RH56DFTP" / "urdf_right_with_force_sensor" / \
    "urdf" / "urdf_right_with_force_sensor.urdf"

OUTPUT_DIR = SCRIPT_DIR / "T1_7dof_with_hands"
OUTPUT_URDF = OUTPUT_DIR / "T1_7dof_with_hands.urdf"


def parse_urdf(filepath):
    """Parse URDF file and return root element."""
    tree = ET.parse(filepath)
    return tree.getroot()


def remove_gripper_links(root, gripper_name):
    """Remove gripper link but preserve the wrist joint for connecting new hand."""
    # Remove link
    for link in root.findall('link'):
        if gripper_name in link.get('name', ''):
            root.remove(link)
            print(f"Removed link: {link.get('name')}")

    # Find the last arm joint (should be wrist) to connect to hand
    wrist_joints = []
    for joint in root.findall('joint'):
        joint_name = joint.get('name', '')
        if 'Wrist_Roll' in joint_name or 'wrist' in joint_name.lower():
            wrist_joints.append((joint, joint_name))

    if wrist_joints:
        print(f"Found wrist joints: {[j[1] for j in wrist_joints]}")
        return wrist_joints

    return None


def fix_mesh_paths(element, hand_side, base_path="meshes", is_t1=False):
    """Fix mesh file paths to point to correct location."""
    for mesh in element.findall('.//mesh'):
        filename = mesh.get('filename')
        if filename:
            if is_t1:
                # T1 meshes - handle various path formats
                if filename.startswith('../meshes/'):
                    filename = filename.replace('../meshes/', f'{base_path}/')
                elif filename.startswith('meshes/'):
                    filename = filename.replace('meshes/', f'{base_path}/')
                elif 'meshes/' in filename:
                    filename = filename.split('meshes/')[-1]
                    filename = f"{base_path}/{filename}"
                mesh.set('filename', filename)
            else:
                # Hand meshes
                if filename.startswith('package://'):
                    filename = filename.split('meshes/')[-1]
                    filename = f"{base_path}/{hand_side}_hand/{filename}"
                elif 'meshes/' in filename:
                    filename = filename.split('meshes/')[-1]
                    filename = f"{base_path}/{hand_side}_hand/{filename}"
                mesh.set('filename', filename)


def rename_hand_links(hand_root, hand_side):
    """Rename hand links to include side prefix."""
    robot = hand_root

    # Rename all links
    for link in robot.findall('link'):
        old_name = link.get('name')
        if old_name == 'base_link':
            new_name = f"{hand_side}_hand_base"
        else:
            new_name = f"{hand_side}_{old_name}" if not old_name.startswith(
                hand_side) else old_name
        link.set('name', new_name)

    # Rename all joint references
    for joint in robot.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')

        old_parent = parent.get('link')
        old_child = child.get('link')

        # Update parent
        if old_parent == 'base_link':
            new_parent = f"{hand_side}_hand_base"
        else:
            new_parent = f"{hand_side}_{old_parent}" if not old_parent.startswith(
                hand_side) else old_parent
        parent.set('link', new_parent)

        # Update child
        if old_child == 'base_link':
            new_child = f"{hand_side}_hand_base"
        else:
            new_child = f"{hand_side}_{old_child}" if not old_child.startswith(
                hand_side) else old_child
        child.set('link', new_child)

        # Rename joint
        old_joint_name = joint.get('name')
        new_joint_name = f"{hand_side}_{old_joint_name}" if not old_joint_name.startswith(
            hand_side) else old_joint_name
        joint.set('name', new_joint_name)


def merge_hand(t1_root, hand_urdf, hand_side, wrist_joint, parent_link):
    """Merge hand URDF into T1 at wrist attachment point."""
    print(f"\nMerging {hand_side} hand...")

    hand_root = parse_urdf(hand_urdf)

    # Rename hand links/joints to avoid conflicts
    rename_hand_links(hand_root, hand_side)

    # Fix mesh paths
    fix_mesh_paths(hand_root, hand_side)

    # Update the wrist joint to connect to new hand
    if wrist_joint is not None:
        child = wrist_joint.find('child')
        child.set('link', f"{hand_side}_hand_base")
        print(
            f"  Connected {wrist_joint.get('name')} to {hand_side}_hand_base")

    # Add all hand links and joints to T1
    for link in hand_root.findall('link'):
        t1_root.append(link)

    for joint in hand_root.findall('joint'):
        t1_root.append(joint)

    print(
        f"  Added {len(hand_root.findall('link'))} links and {len(hand_root.findall('joint'))} joints")


def copy_meshes():
    """Copy mesh files to output directory."""
    print("\nCopying meshes...")

    # Create mesh directories
    (OUTPUT_DIR / "meshes").mkdir(parents=True, exist_ok=True)

    # Copy T1 meshes
    t1_mesh_src = T1_7DOF_URDF.parent / "meshes"
    if not t1_mesh_src.exists():
        # Try alternate location
        t1_mesh_src = SCRIPT_DIR / "T1" / "meshes"

    if t1_mesh_src.exists():
        t1_mesh_dst = OUTPUT_DIR / "meshes"
        for mesh_file in t1_mesh_src.glob("*.STL"):
            shutil.copy(mesh_file, t1_mesh_dst / mesh_file.name)
        print(f"  Copied T1 meshes")
    else:
        print(f"  WARNING: T1 mesh directory not found at {t1_mesh_src}")

    # Copy left hand meshes
    left_hand_mesh_src = SCRIPT_DIR / "RH56DFTP" / \
        "urdf_left_with_force_sensor" / "meshes"
    left_hand_mesh_dst = OUTPUT_DIR / "meshes" / "left_hand"
    left_hand_mesh_dst.mkdir(exist_ok=True)

    for mesh_file in left_hand_mesh_src.glob("*.STL"):
        shutil.copy(mesh_file, left_hand_mesh_dst / mesh_file.name)

    print(
        f"  Copied {len(list(left_hand_mesh_dst.glob('*.STL')))} left hand meshes")

    # Copy right hand meshes
    right_hand_mesh_src = SCRIPT_DIR / "RH56DFTP" / \
        "urdf_right_with_force_sensor" / "meshes"
    right_hand_mesh_dst = OUTPUT_DIR / "meshes" / "right_hand"
    right_hand_mesh_dst.mkdir(exist_ok=True)

    for mesh_file in right_hand_mesh_src.glob("*.STL"):
        shutil.copy(mesh_file, right_hand_mesh_dst / mesh_file.name)

    print(
        f"  Copied {len(list(right_hand_mesh_dst.glob('*.STL')))} right hand meshes")


def main():
    print("=" * 80)
    print("Merging Booster T1 (7-DOF arms) with RH56DFTP Articulated Hands")
    print("=" * 80)

    if not T1_7DOF_URDF.exists():
        print(f"\nERROR: 7-DOF T1 URDF not found at {T1_7DOF_URDF}")
        print("Please ensure the URDF file exists.")
        return

    # Parse T1 URDF
    print(f"\nLoading T1 7-DOF URDF: {T1_7DOF_URDF}")
    t1_root = parse_urdf(T1_7DOF_URDF)

    # Fix T1 mesh paths
    print("\nFixing T1 mesh paths...")
    fix_mesh_paths(t1_root, None, is_t1=True)

    # Find and update left wrist (last arm joint is Hand_Roll in this URDF)
    print("\nLooking for left hand attachment joint...")
    left_wrist_joints = []
    for joint in t1_root.findall('joint'):
        joint_name = joint.get('name', '')
        if 'Left_Hand_Roll' in joint_name or 'Left_Wrist_Yaw' in joint_name:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            print(f"  Found: {joint_name} (parent: {parent}, child: {child})")
            left_wrist_joints.append((joint, parent, child))

    # Find and update right wrist
    print("\nLooking for right hand attachment joint...")
    right_wrist_joints = []
    for joint in t1_root.findall('joint'):
        joint_name = joint.get('name', '')
        if 'Right_Hand_Roll' in joint_name or 'Right_Wrist_Yaw' in joint_name:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            print(f"  Found: {joint_name} (parent: {parent}, child: {child})")
            right_wrist_joints.append((joint, parent, child))

    if not left_wrist_joints or not right_wrist_joints:
        print("\nERROR: Could not find wrist joints!")
        print("Available joints:")
        for joint in t1_root.findall('joint'):
            print(f"  - {joint.get('name')}")
        return

    # Use the last joint in each arm (should be wrist roll)
    left_wrist = left_wrist_joints[-1][0]
    right_wrist = right_wrist_joints[-1][0]

    # Remove gripper/end effector links
    for link in list(t1_root.findall('link')):
        link_name = link.get('name', '')
        if 'hand' in link_name.lower() or 'gripper' in link_name.lower():
            t1_root.remove(link)
            print(f"Removed end effector: {link_name}")

    # Merge hands
    merge_hand(t1_root, LEFT_HAND_URDF, "left",
               left_wrist, left_wrist_joints[-1][1])
    merge_hand(t1_root, RIGHT_HAND_URDF, "right",
               right_wrist, right_wrist_joints[-1][1])

    # Create output directory
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Write merged URDF
    tree = ET.ElementTree(t1_root)
    ET.indent(tree, space="  ")
    tree.write(OUTPUT_URDF, encoding='utf-8', xml_declaration=True)
    print(f"\n✅ Merged URDF written to: {OUTPUT_URDF}")

    # Copy meshes
    copy_meshes()

    print("\n" + "=" * 80)
    print("✅ MERGE COMPLETE!")
    print("=" * 80)
    print(f"\nOutput directory: {OUTPUT_DIR}")
    print(f"URDF file: {OUTPUT_URDF}")
    print(f"Meshes: {OUTPUT_DIR / 'meshes'}")
    print("\nTotal DOF: ~56 (13 legs/waist + 2 head + 14 arms + 24 hands + 3 grippers removed)")


if __name__ == "__main__":
    main()

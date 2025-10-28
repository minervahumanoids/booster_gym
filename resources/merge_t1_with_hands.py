#!/usr/bin/env python3
"""
Merge Booster T1 URDF with RH56DFTP articulated hands.

This script:
1. Loads T1_locomotion.urdf
2. Removes the nub left_hand_link and right_hand_link
3. Attaches RH56DFTP left and right hands at the wrist attachment points
4. Outputs T1_with_hands.urdf with all meshes properly referenced
"""

import xml.etree.ElementTree as ET
import os
import shutil
from pathlib import Path

# Paths
SCRIPT_DIR = Path(__file__).parent
# Use robot_lab's T1 URDF which has 5-DOF arms + actuated Waist
T1_URDF = Path.home() / "Minerva" / "robot_lab" / "source" / "robot_lab" / "data" / \
    "Robots" / "booster" / "t1_description" / "urdf" / "robot.urdf"
LEFT_HAND_URDF = SCRIPT_DIR / "RH56DFTP" / "urdf_left_with_force_sensor" / \
    "urdf" / "urdf_left_with_force_sensor.urdf"
RIGHT_HAND_URDF = SCRIPT_DIR / "RH56DFTP" / "urdf_right_with_force_sensor" / \
    "urdf" / "urdf_right_with_force_sensor.urdf"

OUTPUT_DIR = SCRIPT_DIR / "T1_with_hands_5dof"
OUTPUT_URDF = OUTPUT_DIR / "T1_with_hands.urdf"


def parse_urdf(filepath):
    """Parse URDF file and return root element."""
    tree = ET.parse(filepath)
    return tree.getroot()


def remove_hand_links(root, hand_name):
    """Remove nub hand link but preserve the joint for connecting new hand."""
    # Remove link
    for link in root.findall('link'):
        if link.get('name') == hand_name:
            root.remove(link)
            print(f"Removed link: {hand_name}")

    # Find joint connecting to hand (preserve it, don't remove!)
    for joint in root.findall('joint'):
        child = joint.find('child')
        if child is not None and child.get('link') == hand_name:
            joint_name = joint.get('name')
            parent_link = joint.find('parent').get('link')
            origin = joint.find('origin')
            # Don't remove the joint! We'll update it to connect to new hand
            print(
                f"Found joint: {joint_name} (parent: {parent_link}) - will reuse for new hand")
            return joint, parent_link, origin

    return None, None, None


def fix_mesh_paths(element, hand_side, base_path="meshes", is_t1=False):
    """Fix mesh file paths to point to correct location."""
    for mesh in element.findall('.//mesh'):
        filename = mesh.get('filename')
        if filename:
            if is_t1:
                # T1 meshes from robot_lab use ../meshes/ prefix
                if filename.startswith('../meshes/'):
                    filename = filename.replace('../meshes/', f'{base_path}/')
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


def rename_hand_links(hand_root, hand_side, parent_link):
    """Rename hand links to include side prefix and set base_link parent."""
    robot = hand_root

    # Rename all links
    for link in robot.findall('link'):
        old_name = link.get('name')
        if old_name == 'base_link':
            # base_link becomes the attachment point
            new_name = f"{hand_side}_hand_base"
        else:
            new_name = f"{hand_side}_{old_name}" if not old_name.startswith(
                hand_side) else old_name
        link.set('name', new_name)
        print(f"  Renamed link: {old_name} -> {new_name}")

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


def enable_arm_joints(t1_root):
    """Convert fixed arm joints to revolute for manipulation (if needed)."""
    print("\nChecking arm joint types...")
    arm_joints = [
        "Left_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
        "Right_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw"
    ]

    for joint in t1_root.findall('joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        if joint_name in arm_joints:
            if joint_type == 'fixed':
                joint.set('type', 'revolute')
                # Ensure joint has limits
                limit = joint.find('limit')
                if limit is None:
                    limit = ET.Element('limit')
                    limit.set('lower', '-2.0')
                    limit.set('upper', '2.0')
                    limit.set('effort', '18')
                    limit.set('velocity', '18.84')
                    joint.append(limit)
                print(f"  Enabled {joint_name}: fixed → revolute")
            else:
                print(f"  ✓ {joint_name}: already {joint_type}")


def merge_hand(t1_root, hand_urdf, hand_side, elbow_yaw_joint, parent_link, origin):
    """Merge hand URDF into T1 at attachment point."""
    print(f"\nMerging {hand_side} hand...")

    hand_root = parse_urdf(hand_urdf)

    # Rename hand links/joints to avoid conflicts
    rename_hand_links(hand_root, hand_side, parent_link)

    # Fix mesh paths
    fix_mesh_paths(hand_root, hand_side)

    # Update the existing Elbow_Yaw joint to connect to new hand
    if elbow_yaw_joint is not None:
        # Update the child link to point to new hand base
        child = elbow_yaw_joint.find('child')
        child.set('link', f"{hand_side}_hand_base")
        print(
            f"  Reused {elbow_yaw_joint.get('name')} joint (Elbow_Yaw) for new hand attachment")

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

    # Copy T1 meshes from robot_lab (OBJ format)
    t1_mesh_src = T1_URDF.parent.parent / "meshes"
    t1_mesh_dst = OUTPUT_DIR / "meshes"

    for mesh_file in t1_mesh_src.glob("*.obj"):
        if "hand" not in mesh_file.name.lower():  # Skip original hand meshes
            shutil.copy(mesh_file, t1_mesh_dst / mesh_file.name)
            # Copy corresponding MTL files if they exist
            mtl_file = mesh_file.with_suffix('.mtl')
            if mtl_file.exists():
                shutil.copy(mtl_file, t1_mesh_dst / mtl_file.name)

    print(f"  Copied T1 meshes from robot_lab (OBJ format)")

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
    print("Merging Booster T1 with RH56DFTP Articulated Hands")
    print("=" * 80)

    # Parse T1 URDF
    print(f"\nLoading T1 URDF: {T1_URDF}")
    t1_root = parse_urdf(T1_URDF)

    # Fix T1 mesh paths (robot_lab uses ../meshes/)
    print("\nFixing T1 mesh paths...")
    fix_mesh_paths(t1_root, None, is_t1=True)

    # Check/enable arm joints (robot_lab T1 should already be revolute)
    enable_arm_joints(t1_root)

    # Remove left hand nub and get attachment joint/info
    left_joint, left_parent, left_origin = remove_hand_links(
        t1_root, "left_hand_link")

    # Remove right hand nub and get attachment joint/info
    right_joint, right_parent, right_origin = remove_hand_links(
        t1_root, "right_hand_link")

    # Merge left hand
    if left_parent:
        merge_hand(t1_root, LEFT_HAND_URDF, "left",
                   left_joint, left_parent, left_origin)
    else:
        print("ERROR: Could not find left hand attachment point!")
        return

    # Merge right hand
    if right_parent:
        merge_hand(t1_root, RIGHT_HAND_URDF, "right",
                   right_joint, right_parent, right_origin)
    else:
        print("ERROR: Could not find right hand attachment point!")
        return

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
    print("\nNext step: Convert URDF to USD for Isaac Sim")


if __name__ == "__main__":
    main()

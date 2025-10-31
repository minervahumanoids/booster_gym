#!/usr/bin/env python3
"""Fix hand body names in T1_7dof_with_hands URDF to match USD naming.

This makes the nub USD and articulated hands URDF interchangeable by ensuring
they both use the same EE frame names: left_hand_link, right_hand_link
"""

import xml.etree.ElementTree as ET
from pathlib import Path
import shutil

# File paths
urdf_dir = Path(__file__).parent / "T1_7dof_with_hands"
input_file = urdf_dir / "T1_7dof_with_hands.urdf"
output_file = urdf_dir / "T1_7dof_with_hands.urdf"
backup_file = urdf_dir / "T1_7dof_with_hands.urdf.backup"

# Backup original
if not backup_file.exists():
    print(f"Creating backup: {backup_file}")
    shutil.copy2(input_file, backup_file)
else:
    print(f"Backup already exists: {backup_file}")

# Parse URDF
print(f"Parsing {input_file}")
tree = ET.parse(input_file)
root = tree.getroot()

# Rename mappings for interchangeability
# Old name (URDF) -> New name (to match USD)
rename_map = {
    "left_hand_base": "left_hand_link",
    "right_hand_base": "right_hand_link",
}

changes = 0

# Rename in <link name="">
for link in root.findall('.//link'):
    old_name = link.get('name')
    if old_name in rename_map:
        new_name = rename_map[old_name]
        link.set('name', new_name)
        print(f"  Renamed link: {old_name} -> {new_name}")
        changes += 1

# Rename in <joint><parent link=""> and <child link="">
for joint in root.findall('.//joint'):
    for elem in ['parent', 'child']:
        node = joint.find(elem)
        if node is not None:
            old_name = node.get('link')
            if old_name in rename_map:
                new_name = rename_map[old_name]
                node.set('link', new_name)
                print(f"  Renamed {elem} in joint: {old_name} -> {new_name}")
                changes += 1

if changes > 0:
    # Write modified URDF
    print(f"\nWriting modified URDF to {output_file}")
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"✅ Successfully renamed {changes} references")
    print(f"\nNow both variants use:")
    print(f"  - Left EE: left_hand_link")
    print(f"  - Right EE: right_hand_link")
else:
    print("No changes needed - names already match!")

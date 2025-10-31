#!/usr/bin/env python3
"""Fix mesh paths in T1_7_dof_arm_serial.urdf to be absolute paths."""

import xml.etree.ElementTree as ET
from pathlib import Path
import shutil

# File paths
input_file = Path(__file__).parent / "T1_7_dof_arm_serial.urdf"
output_file = input_file
backup_file = Path(__file__).parent / "T1_7_dof_arm_serial.urdf.backup"

# Backup if needed
if not backup_file.exists():
    print(f"Creating backup: {backup_file}")
    shutil.copy2(input_file, backup_file)

# Parse URDF
print(f"Parsing {input_file}")
tree = ET.parse(input_file)
root = tree.getroot()

# Get absolute path to resources directory
resources_dir = Path(__file__).parent.absolute()

changes = 0

# Fix mesh paths
for mesh in root.findall('.//mesh'):
    filename = mesh.get('filename')
    if filename:
        # Check if it's a relative path
        if not filename.startswith('/') and not filename.startswith('file://'):
            # Make it absolute
            abs_path = resources_dir / filename
            if abs_path.exists():
                mesh.set('filename', str(abs_path))
                print(f"  Fixed: {filename} -> {abs_path}")
                changes += 1
            else:
                print(f"  ⚠️ Warning: Mesh file not found: {abs_path}")

if changes > 0:
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"\n✅ Fixed {changes} mesh paths")
else:
    print("\n✅ No changes needed - paths already correct")


#!/usr/bin/env python3
"""Helper script to convert USD to ASCII format for manual editing."""

from pxr import Usd
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--input", required=True, help="Input USD file")
parser.add_argument("--output", help="Output USDA file (default: input.usda)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app


if not args.output:
    args.output = args.input.replace('.usd', '.usda')
    if args.output == args.input:
        args.output = args.input + '.usda'

print(f"Converting {args.input} → {args.output}")

stage = Usd.Stage.Open(args.input)
if stage:
    stage.GetRootLayer().Export(args.output)
    print(f"✅ Success! Created {args.output}")
    print(f"\nNow you can edit {args.output} with any text editor")
    print("Search for 'Link1', 'Link11', 'Link2', 'Link22' and remove those sections")
    print("\nAfter editing, convert back with:")
    print(
        f"  ~/Projects/IsaacLab/isaaclab.sh -p convert_usda_to_usd_helper.py --input {args.output}")
else:
    print(f"❌ Failed to open {args.input}")

simulation_app.close()

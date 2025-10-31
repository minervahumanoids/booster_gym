#!/usr/bin/env python3
"""Helper script to convert USDA (ASCII) back to USD after manual editing."""

from pxr import Usd
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--input", required=True, help="Input USDA file (ASCII)")
parser.add_argument("--output", help="Output USD file (default: input.usd)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app


if not args.output:
    args.output = args.input.replace('.usda', '.usd')
    if args.output == args.input:
        args.output = args.input.replace('.usd', '_nubs.usd')

print(f"Converting {args.input} → {args.output}")

stage = Usd.Stage.Open(args.input)
if stage:
    stage.GetRootLayer().Export(args.output)
    print(f"✅ Success! Created {args.output}")
else:
    print(f"❌ Failed to open {args.input}")

simulation_app.close()

#!/bin/bash
# Convert USD to ASCII format for manual editing

INPUT="$1"
OUTPUT="${INPUT%.usd}.usda"

if [ -z "$INPUT" ]; then
    echo "Usage: $0 <input.usd>"
    echo "Example: $0 t1_with_7dof_arms.usd"
    exit 1
fi

if [ ! -f "$INPUT" ]; then
    echo "Error: File not found: $INPUT"
    exit 1
fi

echo "Converting $INPUT to ASCII format..."
echo "Output: $OUTPUT"
echo ""
echo "After editing $OUTPUT, convert back with:"
echo "  ~/Projects/IsaacLab/isaaclab.sh -p convert_usda_to_usd.py $OUTPUT"

# Try with Isaac Sim Python first
if [ -f ~/Projects/IsaacLab/_isaac_sim/python.sh ]; then
    ~/Projects/IsaacLab/_isaac_sim/python.sh << PYTHON
import sys
sys.path.insert(0, '/home/mrahme/Projects/IsaacLab/_isaac_sim/python/lib/python3.11/site-packages')
try:
    from pxr import Usd
    stage = Usd.Stage.Open('$INPUT')
    if stage:
        stage.GetRootLayer().Export('$OUTPUT')
        print("✅ Success! Created $OUTPUT")
    else:
        print("❌ Failed to open USD")
        sys.exit(1)
except Exception as e:
    print(f"❌ Error: {e}")
    print("Trying alternative method...")
    sys.exit(1)
PYTHON

    if [ $? -eq 0 ]; then
        exit 0
    fi
fi

echo "⚠️  Direct Python method failed. Try:"
echo "  ~/Projects/IsaacLab/isaaclab.sh -p convert_usd_to_ascii_helper.py --input $INPUT --output $OUTPUT"


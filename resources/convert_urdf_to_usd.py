#!/usr/bin/env python3
"""
Convert T1_with_hands URDF to USD format for Isaac Sim.

This script uses Isaac Sim's URDF importer to create a USD file that
can be used in Isaac Lab.
"""

from isaaclab.app import AppLauncher

# Launch Isaac Sim (headless)
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics
from pathlib import Path
import carb

# Paths
SCRIPT_DIR = Path(__file__).parent
URDF_FILE = SCRIPT_DIR / "T1_with_hands" / "T1_with_hands.urdf"
USD_OUTPUT = SCRIPT_DIR / "T1_with_hands" / "T1_with_hands.usd"

def main():
    print("=" * 80)
    print("Converting T1_with_hands URDF to USD")
    print("=" * 80)
    print(f"\nInput URDF: {URDF_FILE}")
    print(f"Output USD: {USD_OUTPUT}")
    
    if not URDF_FILE.exists():
        print(f"\n❌ ERROR: URDF file not found: {URDF_FILE}")
        print("Run merge_t1_with_hands.py first!")
        return
    
    # Create a new stage
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    
    print("\nImporting URDF...")
    
    # Import URDF
    try:
        success, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(URDF_FILE),
            import_config=omni.isaac.core_nodes.scripts.utils.createDefaultImportConfig(),
            dest_path="/World/T1_with_hands"
        )
        
        if not success:
            print("❌ URDF import failed!")
            return
        
        print(f"✅ URDF imported to: {prim_path}")
        
        # Get the imported prim
        robot_prim = stage.GetPrimAtPath(prim_path)
        
        # Configure physics properties
        print("\nConfiguring physics properties...")
        
        # Enable self-collisions
        if robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            artic_api = UsdPhysics.ArticulationRootAPI(robot_prim)
            artic_api.GetEnabledSelfCollisionsAttr().Set(True)
            print("  ✅ Enabled self-collisions")
        
        # Set default position/orientation
        xform_api = UsdGeom.XformCommonAPI(robot_prim)
        xform_api.SetTranslate((0, 0, 1.05))  # Standing height
        xform_api.SetRotate((0, 0, 0))
        print("  ✅ Set default transform (0, 0, 1.05)")
        
        # Save USD
        print(f"\nSaving USD to: {USD_OUTPUT}")
        stage.GetRootLayer().Export(str(USD_OUTPUT))
        print("✅ USD file saved successfully!")
        
        # Print stats
        print("\n" + "=" * 80)
        print("CONVERSION COMPLETE!")
        print("=" * 80)
        print(f"\nUSD file: {USD_OUTPUT}")
        print(f"Robot prim path: {prim_path}")
        
        # Count links and joints
        num_links = 0
        num_joints = 0
        for prim in robot_prim.GetAllChildren():
            if "link" in prim.GetName().lower():
                num_links += 1
            elif "joint" in prim.GetName().lower():
                num_joints += 1
        
        print(f"Links: ~{num_links}")
        print(f"Joints: ~{num_joints}")
        print("\nNext step: Update wb_manip/source/wb_manip/wb_manip/assets/booster.py")
        print(f"  Set usd_path to: {USD_OUTPUT}")
        
    except Exception as e:
        print(f"\n❌ ERROR during conversion: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()


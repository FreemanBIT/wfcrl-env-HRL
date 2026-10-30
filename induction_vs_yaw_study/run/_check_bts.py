
import numpy as np
import struct

bts_file = r"D:\HR_Project\wfcrl-env-HRL\induction_vs_yaw_study\FarmInputs\coarse\inflow_U08_TI10_seed01.bts"

with open(bts_file, 'rb') as f:
    # Read header
    data = f.read()
    
# BTS format: first 2 bytes = version (int16)
# Then various metadata
# Let's try to read it properly

f = open(bts_file, 'rb')

# Version
version = struct.unpack('<h', f.read(2))[0]
print(f"BTS Version: {version}")

if version == 8:
    # FAST BTS format
    # Skip 2 bytes (reserved)
    f.read(2)
    
    # NumGrid_Z, NumGrid_Y (int32)
    nz = struct.unpack('<i', f.read(4))[0]
    ny = struct.unpack('<i', f.read(4))[0]
    print(f"Grid: {ny} x {nz}")
    
    # NumTurbines (int32) - actually this might be ntstep or something else
    # Let me read more carefully
    
    # Actually the BTS header format varies by version.
    # Let me just read the file and look for wind direction info.
    pass

f.close()

# Alternative: use the wfcrl library to read the BTS
import sys, os
sys.path.insert(0, r"D:\HR_Project\wfcrl-env-HRL")

# Try to read BTS using available tools
try:
    from wfcrl.wind.bts_reader import BTSFile
    bts = BTSFile(bts_file)
    print(f"Mean wind speed: {bts.mean_u:.2f} m/s")
    print(f"Wind direction: {bts.mean_wd:.2f} deg")
except ImportError:
    print("BTSFile not available, trying alternative...")
    
    # Manual BTS parse for version 8
    with open(bts_file, 'rb') as f:
        version = struct.unpack('<h', f.read(2))[0]
        f.read(2)  # skip
        
        nz = struct.unpack('<i', f.read(4))[0]
        ny = struct.unpack('<i', f.read(4))[0]
        nt = struct.unpack('<i', f.read(4))[0]
        dz = struct.unpack('<f', f.read(4))[0]
        dy = struct.unpack('<f', f.read(4))[0]
        dt = struct.unpack('<f', f.read(4))[0]
        v_center = struct.unpack('<f', f.read(4))[0]
        h_center = struct.unpack('<f', f.read(4))[0]
        
        print(f"Grid: {ny} x {nz}, {nt} steps")
        print(f"dt={dt:.3f}s, dy={dy:.2f}m, dz={dz:.2f}m")
        print(f"Hub height: {h_center:.2f}m")
        print(f"Grid center: {v_center:.2f}m")
        
        # Mean wind speed (uhub) is usually at a specific offset
        # For version 8, it's after the grid info
        uhub = struct.unpack('<f', f.read(4))[0]
        print(f"Hub wind speed (uhub): {uhub:.3f} m/s")
        
        # Wind direction might be next
        # But format varies - let me read a few more floats
        for i in range(5):
            val = struct.unpack('<f', f.read(4))[0]
            print(f"  float[{i}]: {val:.4f}")

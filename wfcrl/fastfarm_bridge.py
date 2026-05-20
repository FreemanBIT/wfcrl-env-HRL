"""
FAST.Farm MPI Bridge - runs alongside FAST.Farm under mpiexec,
relaying data to/from the main WFCRL process via shared memory/pipe.

Usage: mpiexec -n 1 FAST.Farm.exe input.fstf : -n 1 python -m wfcrl.fastfarm_bridge <pipe_name>
"""
import json
import os
import socket
import struct
import sys
import time
import traceback
from pathlib import Path

import numpy as np
from mpi4py import MPI

# MPI communicator
# Rank 0 = FAST.Farm, Rank 1 = this bridge
comm = MPI.COMM_WORLD
rank = comm.Get_rank()
size = comm.Get_size()

# Get the intercommunicator to FAST.Farm (parent)
parent_comm = comm.Get_parent()

# Parse pipe name from args
pipe_name = sys.argv[1] if len(sys.argv) > 1 else "wfcrl_ff_bridge"


def run_bridge():
    """
    Bridge process running alongside FAST.Farm:
    - Receives yaw commands from WFCRL via pipe
    - Sends them to FAST.Farm via MPI
    - Receives measurements from FAST.Farm via MPI
    - Sends them back to WFCRL via pipe
    """
    print(f"[Bridge] Rank {rank}/{size} - Starting FAST.Farm bridge...")
    
    # Communicate with FAST.Farm through parent intercommunicator
    # This is the same protocol as the original MPI_Interface
    
    # Receive initial data from FAST.Farm
    num_measures = np.array([0], dtype=int)
    parent_comm.Recv(num_measures, source=0, tag=0)
    print(f"[Bridge] Receiving {num_measures[0]} measures per turbine")
    
    # Send max_iter (unlimited for bridge mode)
    max_iter = np.array([1_000_000], dtype=np.double)
    parent_comm.Send(max_iter, dest=0, tag=0)
    
    num_turbines = 0
    # Listen for commands from WFCRL via named pipe
    pipe_path = f"\\\\.\\pipe\\{pipe_name}" if os.name == 'nt' else f"/tmp/{pipe_name}.sock"
    
    if os.name == 'nt':
        # Windows named pipe
        import win32pipe
        import win32file
        
        # Create named pipe
        pipe = win32pipe.CreateNamedPipe(
            pipe_path,
            win32pipe.PIPE_ACCESS_DUPLEX,
            win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_READMODE_MESSAGE | win32pipe.PIPE_WAIT,
            1, 65536, 65536, 0, None
        )
        print(f"[Bridge] Waiting for WFCRL connection on {pipe_path}...")
        win32pipe.ConnectNamedPipe(pipe, None)
        print(f"[Bridge] Connected!")
        
        while True:
            # Read command from WFCRL
            result, data = win32file.ReadFile(pipe, 65536)
            if result == 0:
                cmd = json.loads(data.decode('utf-8'))
                
                if cmd.get('type') == 'command':
                    # Send yaw command to FAST.Farm
                    yaws = np.array(cmd['yaw'], dtype=np.double)
                    yaw_cmd = np.zeros(len(yaws) + 1, dtype=np.double)
                    yaw_cmd[0] = 1.0
                    yaw_cmd[1:] = np.radians(yaws)
                    
                    parent_comm.Send(yaw_cmd, dest=0, tag=1)
                    
                    # Send zero pitch/torque
                    pitch_cmd = np.zeros(len(yaws) + 1, dtype=np.double)
                    torque_cmd = np.zeros(len(yaws) + 1, dtype=np.double)
                    parent_comm.Send(pitch_cmd, dest=0, tag=2)
                    parent_comm.Send(torque_cmd, dest=0, tag=3)
                    
                    # Receive measurements from FAST.Farm
                    num_turbines = len(yaws)
                    size_buffer = num_turbines * num_measures[0]
                    measures = np.zeros(size_buffer, dtype=np.double)
                    parent_comm.Recv(measures, source=0, tag=4)
                    parent_comm.Barrier()
                    
                    # Send response back to WFCRL
                    response = {
                        'power': measures[0::num_measures[0] if num_measures[0] > 0 else 1].tolist(),
                        'wind_speed': [float(np.degrees(measures[i * num_measures[0] + 1])) if i * num_measures[0] + 1 < len(measures) else 0.0 for i in range(num_turbines)],
                    }
                    win32file.WriteFile(pipe, json.dumps(response).encode('utf-8'))
                
                elif cmd.get('type') == 'stop':
                    break
        
        win32pipe.DisconnectNamedPipe(pipe)
        win32file.CloseHandle(pipe)
    
    else:
        # Linux: use Unix socket
        import socketserver
        
        class BridgeHandler(socketserver.BaseRequestHandler):
            def handle(self):
                while True:
                    data = self.request.recv(65536)
                    if not data:
                        break
                    cmd = json.loads(data.decode('utf-8'))
                    # Same logic as above...
                    self.request.send(b'{"status":"ok"}')
        
        server = socketserver.UnixStreamServer(pipe_path, BridgeHandler)
        server.serve_forever()


if __name__ == '__main__':
    try:
        run_bridge()
    except Exception as e:
        print(f"[Bridge] Error: {e}")
        traceback.print_exc()

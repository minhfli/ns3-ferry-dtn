import subprocess
import os

PROGRAM = "ferry"

params = {
    "seed": 1337,
    "name": "SIRA", # algorithm name
    "vi": True,
    "batch": "default",
    "run": "debug", # run name for log file
    "simTime": 500,
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 2000,
    "nGrounds": 10,
    "nFerrys": 3,
    "ferrySpeed": 15,
    "groundBufferSize": 50,
    "ferryBufferSize": 300,
    "bundleGenRate": 30.0, # packets/sec
    "bundleTTL": 600000000, # microsec
    "ferryComm": True, # enable communication between ferry
}

def build_cmd(program, params):
    args = " ".join(f"--{k}={v}" for k, v in params.items())
    return f'./waf --run "{program} {args}"'

def run_command(cmd):
    print(f"\n--- Running: {cmd} ---")
    try:
        # Sử dụng shell=True để chạy được lệnh ./waf
        process = subprocess.Popen(cmd, shell=True)
        process.wait()
    except Exception as e:
        print(f"Error executing command: {e}")
        
if __name__ == "__main__":
    if os.path.basename(os.getcwd()) == "scratch":
         os.chdir("..")
         
    cmd = build_cmd(PROGRAM, params)

    print("Running:", cmd)

    run_command(cmd)


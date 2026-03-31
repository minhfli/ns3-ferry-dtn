import subprocess
import os

PROGRAM = "ferry"

params = {
    "seed": 1337,
    "name": "SIRA", # algorithm name
    "vi": True,
    "batch": "default",
    "run": "debug", # run name for log file
    "simTime": 7200,
    "warmupTime": 500,
    #* some algorithm specific config
    "DRC_graphMode": "DRC_GABRIEL",
    "waypointSelectMode" :"PROBABILISTIC",
    "HUB_nHubs": 1,
    "CHUB_virtualHub": False,
    #* default config
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 4000,
    "nGrounds": 45,
    "nFerrys": 10,
    "ferrySpeed": 12,
    "groundBufferSize": 1000,
    "ferryBufferSize": 500,
    "minGenRate": 10.0, # sec/packet
    "maxGenRate": 15.0, # sec
    "genSrcScheduler": "RANDOM_RANGE",
    "genDstScheduler": "PARETO_6040",
    "genParetoMatch": False,
    "bundleTTL": 900000000, # microsec
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


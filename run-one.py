import subprocess
import os

PROGRAM = "ferry"
PROGRAM2 = "ferryrep"

params = {
    "seed": 762227828,
    "name": "TABAF", # algorithm name
    "vi": True,
    "batch": "default",
    "run": "debug", # run name for log file
    "simTime": 10000,
    "warmupTime": 500,
    "hoverTime": 5,
    "nGrounds": 45,
    "nFerrys": 5,
    #* some algorithm specific config
    # "waypointSelectMode" :"PROBABILISTIC",
    "SnW_replications": 2,
    "waypointSelectMode" :"DETERMINISTIC",
    "MRDLAS_weightedDeadline": True,
    # "TABAF_weightedDeadline": True,
    "HUB_nHubs": 1,
    "CHUB_virtualHub": True,
    "CHUB_routeExtend": True,
    # "CHUB_reWait": True,
    #* default config
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 4000,
    "ferrySpeed": 12,
    "groundBufferSize": 1000,
    "ferryBufferSize": 1000,
    "minGenRate": 10.0, # sec/packet
    "maxGenRate": 15.0, # sec
    "genSrcScheduler": "RANDOM_RANGE",
    "genDstScheduler": "PARETO_6040",
    "genParetoMatch": False,
    "bundleTTL": 600000000, # microsec
    "ferryComm": False, # enable communication between ferry
}

if params["name"].startswith("SNW") or params["name"].startswith("EPIDEMIC"):
    PROGRAM = PROGRAM2

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


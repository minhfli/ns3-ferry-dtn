import subprocess
import os

PROGRAM = "ferry"

params = {
    "seed": 1337,
    "name": "MRDLAS", # algorithm name
    "vi": False,
    "batch": "default",
    "run": "debug2", # run name for log file
    "simTime": 7500,
    #* some algorithm specific config
    "RPDLAS_operationMode": "RPDLAS_NO_REROUTE_COLLECT_INROUTE",
    "RPDLAS_pruneMode": "RPDLAS_PRUNE_MAXIMAL",
    "waypointSelectMode" :"PROBABILISTIC",
    #* default config
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 4000,
    "nGrounds": 30,
    "nFerrys": 5,
    "ferrySpeed": 12,
    "groundBufferSize": 50,
    "ferryBufferSize": 300,
    "minGenRate": 5.0, # sec/packet
    "maxGenRate": 25.0, # sec
    "genSrcScheduler": "RANDOM_RANGE",
    "genDstScheduler": "PARETO_7030",
    "genParetoMatch": False,
    "bundleTTL": 600000000, # microsec
    "ferryComm": True, # enable communication between ferry
}

def build_cmd(program, params):
    args = " ".join(f"--{k}={v}" for k, v in params.items())
    return f'./waf --run "{program} {args}" '

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


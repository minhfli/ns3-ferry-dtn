import subprocess
import os

PROGRAM = "ferry"

params = {
    "seed": 1337,
    "name": "DRC", # algorithm name
    "vi": True,
    "batch": "default",
    "run": "debug2", # run name for log file
    "simTime": 1,
    "warmupTime": 1,
    #* some algorithm specific config
    "RPDLAS_operationMode": "RPDLAS_NO_REROUTE_COLLECT_INROUTE",
    "RPDLAS_pruneMode": "RPDLAS_PRUNE_MAXIMAL",
    "waypointSelectMode" :"PROBABILISTIC",
    #* default config
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 4000,
    "nGrounds": 30,
    "nFerrys": 10,
    "ferrySpeed": 12,
    "groundBufferSize": 50,
    "ferryBufferSize": 500,
    "minGenRate": 10.0, # sec/packet
    "maxGenRate": 20.0, # sec
    "genSrcScheduler": "RANDOM_RANGE",
    "genDstScheduler": "PARETO_6040",
    "genParetoMatch": False,
    "bundleTTL": 900000000, # microsec
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


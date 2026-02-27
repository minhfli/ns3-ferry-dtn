import subprocess
import os
import itertools
import random
import copy
import signal
import sys
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import Manager

PROGRAM = "ferry"

# ============================================================
# CONFIGURATION
# ============================================================

MAX_WORKERS = 6
N_SEEDS = 5

# ============================================================
# Base parameters
# ============================================================

base_params = {
    "seed": 0,
    "name": "PIGEON",
    "run": "exp",
    "simTime": 5000,
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 5000,
}

# ============================================================
# Sweep grid
# ============================================================

param_grid = {
    "name": ["SIRA", "PIGEON", "TABAF"],

    "nGrounds": [25],
    "nFerrys": [15],

    "groundBufferSize": [50],
    "ferryBufferSize": [300],

    "bundleGenRate": [30.0],
    "bundleTTL": [600000000],

    "ferryComm": [False, True],
}

# ============================================================
# Utilities
# ============================================================

def build_cmd(program, params):
    args = " ".join(f"--{k}={v}" for k, v in params.items())
    return f'./waf --run "{program} {args}"'


def generate_param_combinations(grid):
    keys = grid.keys()
    values = grid.values()
    for combination in itertools.product(*values):
        yield dict(zip(keys, combination))


def run_simulation(cmd, pid_list):
    """
    Worker process.
    Ghi lại PID để có thể terminate nếu cần.
    """
    try:
        process = subprocess.Popen(
            cmd,
            shell=True,
            preexec_fn=os.setsid  # tạo process group riêng
        )

        pid_list.append(process.pid)
        process.wait()

        return process.returncode

    except Exception as e:
        return f"ERROR: {e}"


# ============================================================
# Terminate handler
# ============================================================

def terminate_all(pid_list):
    print("\n\nTerminating all running simulations...")

    for pid in pid_list:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except:
            pass

    print("All child processes terminated.")


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":

    if os.path.basename(os.getcwd()) == "scratch":
        os.chdir("..")

    batch_id = datetime.now().strftime("%Y%m%d_%H%M")

    random.seed(1337)
    seed_list = [random.randint(1, 10**9) for _ in range(N_SEEDS)]

    all_configs = list(generate_param_combinations(param_grid))

    print("==============================================")
    print(f"Configs: {len(all_configs)}")
    print(f"Seeds per config: {N_SEEDS}")
    print(f"Total runs: {len(all_configs) * N_SEEDS}")
    print(f"Max parallel workers: {MAX_WORKERS}")
    print("==============================================")

    jobs = []

    for config_id, config in enumerate(all_configs):
        for seed_id, random_seed in enumerate(seed_list):

            params = copy.deepcopy(base_params)
            params.update(config)

            params["seed"] = random_seed
            algo = params["name"]

            params["run"] = (
                f"{batch_id}"
                f"_{algo}"
                f"_G{params['nGrounds']}"
                f"_F{params['nFerrys']}"
                f"_R{params['bundleGenRate']}"
                f"_C{int(params['ferryComm'])}"
                f"_seed{seed_id}"
            )

            cmd = build_cmd(PROGRAM, params)
            jobs.append(cmd)

    manager = Manager()
    pid_list = manager.list()

    # Signal handler
    def signal_handler(sig, frame):
        print("\nCtrl+C detected.")
        terminate_all(pid_list)
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)

    print("\nStarting parallel execution...\n")

    try:
        with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:

            futures = [
                executor.submit(run_simulation, cmd, pid_list)
                for cmd in jobs
            ]

            for i, future in enumerate(as_completed(futures)):
                result = future.result()
                print(f"[{i+1}/{len(jobs)}] Finished: return code = {result}")

    except KeyboardInterrupt:
        terminate_all(pid_list)
        sys.exit(1)

    print("\nAll experiments finished.")
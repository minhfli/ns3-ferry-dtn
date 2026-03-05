import subprocess
import os
import itertools
import random
import copy
from datetime import datetime

PROGRAM = "ferry"
BATCH = "20260305_2am"
BATCH_ID = "20260305" #! TEMPORARY for rerun
# BATCH_ID = datetime.now().strftime("%Y%m%d")


# ============================================================
# Base parameters
# ============================================================
base_params = {
    "batch": BATCH,
    "vi": False,
    "skip": False, # skip if already run
    "seed": 0,  # sẽ được override
    "name": "TABADLA",
    "run": "exp",
    "simTime": 7500,
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 3000,
    "ferrySpeed": 10,
}
# ============================================================
# Parameter sweep grid
# ============================================================
default_grid = {
    "name": ["SIRA", "PIGEON", "TABAF"],

    "nGrounds": [30],
    "nFerrys": [1,3,5,10],

    "groundBufferSize": [1000],
    "ferryBufferSize": [500],

    "bundleGenRate": [10.0, 30.0],
    "bundleTTL": [300000000, 600000000, 1200000000],

    "ferryComm": [False, True],
}

param_grid = { # change or set to default grid for custom run
    "name": [ "TABADLA"],

    "nGrounds": [25],
    "nFerrys": [1,3,5,10],

    "groundBufferSize": [1000],
    "ferryBufferSize": [500],

    "bundleGenRate": [10.0, 30.0],
    "bundleTTL": [ 600000000, 1200000000],

    "ferryComm": [False, True],
}

N_SEEDS = 3

# ============================================================
# Utility: build waf command
# ============================================================
def build_cmd(program, params):
    args = " ".join(f"--{k}={v}" for k, v in params.items())
    return f'./waf --run "{program} {args}"'


# ============================================================
# Run command
# ============================================================
def run_command(cmd):
    print(f"\n--- Running: {cmd} ---")
    try:
        process = subprocess.Popen(cmd, shell=True)
        process.wait()
    except Exception as e:
        print(f"Error executing command: {e}")


# ============================================================
# Generate parameter combinations (Cartesian product)
# ============================================================
def generate_param_combinations(grid):
    keys = grid.keys()
    values = grid.values()
    for combination in itertools.product(*values):
        yield dict(zip(keys, combination))


# ============================================================
# Main
# ============================================================
if __name__ == "__main__":

    # đảm bảo đang ở root ns-3 (không ở scratch/)
    if os.path.basename(os.getcwd()) == "scratch":
        os.chdir("..")

    # Tạo batch id để phân biệt các lần chạy khác nhau
    batch_id = BATCH_ID
        
    # Sinh danh sách seed cố định (đảm bảo công bằng giữa thuật toán)
    random.seed(1337)
    seed_list = [random.randint(1, 10**9) for _ in range(N_SEEDS)]

    # Sinh toàn bộ cấu hình
    all_configs = list(generate_param_combinations(param_grid))

    print("==============================================")
    print(f"Total parameter configurations: {len(all_configs)}")
    print(f"Seeds per configuration: {N_SEEDS}")
    print(f"Total runs: {len(all_configs) * N_SEEDS}")
    print("==============================================")

    for config_id, config in enumerate(all_configs):

        for seed_id, random_seed in enumerate(seed_list):

            params = copy.deepcopy(base_params)
            params.update(config)

            # set seed
            params["seed"] = random_seed

            algo = params["name"]

            # ====================================================
            # Tên run = tên file log
            # ====================================================
            params["run"] = (
                f"{batch_id}"
                f"_G{params['nGrounds']}"
                f"_F{params['nFerrys']}"
                f"_R{params['bundleGenRate']}"
                f"_C{int(params['ferryComm'])}"
                f"_seed{seed_id}"
            )

            cmd = build_cmd(PROGRAM, params)

            print("\n===================================================")
            print(f"Config {config_id + 1}/{len(all_configs)}")
            print(f"Seed {seed_id + 1}/{N_SEEDS}")
            print("Parameters:")
            for k, v in params.items():
                print(f"  {k}: {v}")
            print("===================================================")

            run_command(cmd)

    print("\nAll experiments finished.")
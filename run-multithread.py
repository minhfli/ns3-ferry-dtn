from logging.config import valid_ident
import subprocess
import os
import itertools
import random
import copy
import signal
import threading
import concurrent.futures
import time 
from datetime import datetime, timedelta
import argparse

PROGRAM = "ferry"
BATCH = "20260308_2am"
BATCH_ID = "conf431" #! TEMPORARY for rerun
# BATCH_ID = datetime.now().strftime("%Y%m%d")


# ============================================================
# Base parameters
# ============================================================
base_params = {
    "batch": BATCH,
    "vi": False,
    "skip": True, # skip if already run
    "seed": 0,  # sẽ được override
    "name": "ALGORITHM",
    "run": "RUN",
    "simTime": 11100,
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 4000,
    "ferrySpeed": 12,
    "minGenRate": 5.0, # sec/packet
    "maxGenRate": 25.0, # sec
    "genSrcScheduler": "RANDOM_RANGE",
    "genDstScheduler": "PARETO_7030",
    "genParetoMatch": False,
}
# ============================================================
# Parameter sweep grid
# ============================================================

param_grid = { # change or set to default grid for custom run
    # "name": [ "SIRA", "PIGEON", "TABAF", "SR_PIGEON"],
    "name": [ "SR_PIGEON_V2"],

    "nGrounds": [30],
    # "nFerrys": [ 7, 10],
    # "nFerrys": [ 1, 3, 5, 7, 10, 12, 15],
    "nFerrys": [ 1, 3, 5, 7, 10, 12, 15],

    "groundBufferSize": [1000],
    # "ferryBufferSize": [50, 100, 150, 200, 500],
    "ferryBufferSize": [500],

    # "bundleTTL": [ 450000000, 600000000, 900000000], # 7.5min, 10min, 15min
    "bundleTTL": [ 450000000, 600000000,900000000], 

    "ferryComm": [False, True],
}

algo_variants= {
    "PIGEON": {
        "RC": { # return closest
            "pigeonReturn": 1,
        },
    },
    "TABAF": {
        "default": {
            "waypointSelectMode": "DETERMINISTIC",
        },
        # "PWS": {
        #     "waypointSelectMode": "PROBABILISTIC",
        # },
    },
    "SR_PIGEON": {
        "default": {
            "waypointSelectMode": "PROBABILISTIC",
        },  
        "DWS": {
            "waypointSelectMode": "DETERMINISTIC",
        },
    },
    "SR_PIGEON_V2": {
        # "default": {
        #     "waypointSelectMode": "PROBABILISTIC",
        # },  
        # "DWS": {
        #     "waypointSelectMode": "DETERMINISTIC",
        # },
        # "PWS_addlvt": {
        #     "waypointSelectMode": "PROBABILISTIC",
        #     "SR_PIGEON_V2_addlvt": True
        # },
        "DWS_addlvt": {
            "waypointSelectMode": "DETERMINISTIC",
            "SR_PIGEON_V2_addlvt": True
        }
    },
    "TABADLA": {
        # "default": { # previous run show that this is not good
        #     "waypointSelectMode": "DETERMINISTIC",
        #     "TABADLA_addBundleValue": True
        # },
        # "PWS_noB": {
        #     "waypointSelectMode": "PROBABILISTIC",
        #     "TABADLA_addBundleValue": False
        # },
        "DWS_noB": {
            "waypointSelectMode": "DETERMINISTIC",
            "TABADLA_addBundleValue": False
        },
    },
}

N_SEEDS = 3
MAX_WORKERS = 8

# ============================================================
# Global states cho Multithreading & Signal Handling
# ============================================================
stop_event = threading.Event()       # Cờ báo hiệu dừng toàn bộ
active_processes = set()             # Lưu trữ các process đang chạy
process_lock = threading.Lock()      # Lock để tránh race condition khi modify set

def build_cmd(program, params):
    args = " ".join(f"--{k}={v}" for k, v in params.items())
    return f'./waf --run-no-build "{program} {args}"'

def run_single_simulation(cmd, config_name):
    """Hàm worker thực thi một mô phỏng duy nhất"""
    # Nếu có lệnh dừng, không chạy thêm task mới
    if stop_event.is_set():
        return

    print(f"\n[START] {config_name}")
    start_time = time.perf_counter() # Bắt đầu bấm giờ cho process này
    
    try:
        # start_new_session=True là mấu chốt: tạo ra một process group mới
        # để khi kill, ta có thể kill cả waf lẫn ns-3 C++ binary.
        process = subprocess.Popen(
            cmd, 
            shell=True, 
            stdout=subprocess.DEVNULL, # Có thể thay bằng file nếu muốn lưu log std
            stderr=subprocess.DEVNULL,
            start_new_session=True     
        )
        
        with process_lock:
            active_processes.add(process)
        
        # Chờ process chạy xong
        process.wait()

    except Exception as e:
        if not stop_event.is_set():
            print(f"Error executing command: {e}")
    finally:
        with process_lock:
            if process in active_processes:
                active_processes.remove(process)
        if not stop_event.is_set():
            end_time = time.perf_counter() # Kết thúc bấm giờ
            duration = end_time - start_time
            if duration < 20.0 and duration > 2.0:
                # > runtime if skip, << expected runtime
                
                print(f"[WARNING] {config_name} - Thời gian chạy bất thường: {duration:.2f} giây")
                print(f"  -> Command: {cmd}")
                
                
            print(f"[DONE] {config_name} - Thời gian chạy: {duration:.2f} giây")

def signal_handler(sig, frame):
    """Hàm xử lý khi người dùng bấm Ctrl+C"""
    print("\n\n[!!!] BẮT ĐƯỢC LỆNH CTRL+C (SIGINT). ĐANG DỌN DẸP VÀ HỦY CÁC TIẾN TRÌNH...")
    
    # Bật cờ để các task đang chờ trong queue không được thực thi
    stop_event.set()
    
    # Kill tất cả các process đang chạy ngầm
    with process_lock:
        for p in list(active_processes):
            try:
                # Kill toàn bộ process group (waf + ns-3 binary)
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                print(f"  -> Đã kill process group {p.pid}")
            except Exception as e:
                pass
    
    print("[!!!] ĐÃ HỦY TOÀN BỘ. Thoát chương trình.")
    # Force exit ngay lập tức, bỏ qua việc chờ các thread dọn dẹp
    os._exit(1)

def generate_param_combinations(grid):
    keys = grid.keys()
    values = grid.values()
    for combination in itertools.product(*values):
        yield dict(zip(keys, combination))

if __name__ == "__main__":
    if os.path.basename(os.getcwd()) == "scratch":
        os.chdir("..")

    # load comandline
    command_index = None
    parser = argparse.ArgumentParser(description="Run multiple simulations with different parameters.")
    parser.add_argument("--command-index", type=int, default=None, help="Xuất lệnh của cấu hình thứ N (bắt đầu từ 1) mà không thực thi.")
    args = parser.parse_args()
    if args.command_index:
        command_index = args.command_index - 1 # Chuyển sang index bắt đầu từ 0

    # Đăng ký hàm xử lý sự kiện Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    batch_id = BATCH_ID
    random.seed(1337)
    seed_list = [random.randint(1, 10**9) for _ in range(N_SEEDS)]
    all_configs = list(generate_param_combinations(param_grid))
    
    # Chuẩn bị danh sách các task
    tasks = []
    total_config_count = 0
    for config in all_configs:
        algo = config["name"]
        if algo not in algo_variants:
            algo_variants[algo] = {"default": {}}
        total_config_count += len(algo_variants[algo])
    
    for config_id, config in enumerate(all_configs):
        for seed_id, random_seed in enumerate(seed_list):
            params = copy.deepcopy(base_params)
            params.update(config)
            params["seed"] = random_seed
            algo = params["name"]
            
            if algo not in algo_variants:
                algo_variants[algo] = {"default": {}}
            for variant_name, variant_params in algo_variants.get(algo, {}).items():
                params.update(variant_params)
                if variant_name == "default":
                    variant_name = ""
                else:
                    variant_name = f"variant_{variant_name}_"
                params["run"] = (
                    f"{variant_name}"
                    f"{batch_id}"
                    f"_G{params['nGrounds']}"
                    f"_F{params['nFerrys']}"
                    f"_GBS{params['groundBufferSize']}"
                    f"_FBS{params['ferryBufferSize']}"
                    f"_T{params['bundleTTL'] / 1000000}"
                    f"_minR{params['minGenRate']}"
                    f"_maxR{params['maxGenRate']}"
                    f"_Src{params['genSrcScheduler']}"
                    f"_Dst{params['genDstScheduler']}"
                    f"_P{int(params['genParetoMatch'])}"
                    f"_C{int(params['ferryComm'])}"
                    f"_seed{seed_id}"
                )
                cmd = build_cmd(PROGRAM, params)
                config_name = f"Config {config_id + 1}/{len(all_configs)} | Seed {seed_id + 1}/{N_SEEDS} | Algo: {algo} | Variant: {variant_name}"
                tasks.append((cmd, config_name))


    if command_index is not None:
        config = all_configs[command_index] 
        for seed_id, random_seed in enumerate(seed_list):
            params = copy.deepcopy(base_params)
            params.update(config)
            params["seed"] = random_seed
            params["batch"] = "default"
            params["vi"] = True
            algo = params["name"]
            params["run"] = "debug"
            cmd = build_cmd(PROGRAM, params)
            config_name = f"Config {command_index + 1}/{len(all_configs)} | Seed {seed_id + 1}/{N_SEEDS} | Algo: {algo}"
            print(f"\n[COMMAND ONLY] {config_name} | Seed {seed_id + 1}/{N_SEEDS}")
            print(cmd) 
        exit(0)
    print("==============================================")
    print(f"Total parameter configurations: {len(all_configs)}")
    print(f"Total config and variants: {total_config_count}")
    print(f"Seeds per configuration: {N_SEEDS}")
    print(f"Total runs: {len(tasks)}")
    print(f"Concurrent workers (Threads): {MAX_WORKERS}")
    print("Nhấn [Ctrl + C] bất cứ lúc nào để DỪNG HOÀN TOÀN toàn bộ mô phỏng.")
    print("==============================================\n")

    total_start_time = time.time()
    # Chạy đa luồng sử dụng ThreadPoolExecutor
    with concurrent.futures.ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = [executor.submit(run_single_simulation, cmd, name) for cmd, name in tasks]
        
        # Dùng vòng lặp chờ kết quả thay vì executor.shutdown(wait=True)
        # để Main Thread vẫn rảnh rỗi và có thể bắt tín hiệu Ctrl+C ngay lập tức
        try:
            for future in concurrent.futures.as_completed(futures):
                pass
        except KeyboardInterrupt:
            # Fallback nếu signal handler không kịp bắt
            signal_handler(signal.SIGINT, None)

    if not stop_event.is_set():
        total_end_time = time.time()
        total_duration_sec = total_end_time - total_start_time
        # Chuyển đổi số giây thành định dạng hh:mm:ss cho dễ đọc
        formatted_time = str(timedelta(seconds=int(total_duration_sec)))
        
        print("\n==============================================")
        print("🎉 ALL EXPERIMENTS FINISHED SUCCESSFULLY.")
        print(f"⏱️ Tổng thời gian chạy: {formatted_time} (hh:mm:ss)")
        print("==============================================")
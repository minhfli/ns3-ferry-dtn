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

PROGRAM = "ferry"
BATCH = "20260301_8am"
# BATCH_ID = "20260228" #! TEMPORARY for rerun
BATCH_ID = datetime.now().strftime("%Y%m%d")


# ============================================================
# Base parameters
# ============================================================
base_params = {
    "batch": BATCH,
    "vi": False,
    "skip": False, # skip if already run
    "seed": 0,  # sẽ được override
    "name": "PIGEON",
    "run": "exp",
    "simTime": 10000,
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 4000,
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
    "name": [ "TABAF"],

    "nGrounds": [30],
    "nFerrys": [1,3,5,10],

    "groundBufferSize": [1000],
    "ferryBufferSize": [500],

    "bundleGenRate": [10.0, 30.0],
    "bundleTTL": [300000000, 600000000, 1200000000],

    "ferryComm": [True],
}

N_SEEDS = 3
MAX_WORKERS = 12

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

    # Đăng ký hàm xử lý sự kiện Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    batch_id = BATCH_ID
    random.seed(1337)
    seed_list = [random.randint(1, 10**9) for _ in range(N_SEEDS)]
    all_configs = list(generate_param_combinations(param_grid))

    # Chuẩn bị danh sách các task
    tasks = []
    for config_id, config in enumerate(all_configs):
        for seed_id, random_seed in enumerate(seed_list):
            params = copy.deepcopy(base_params)
            params.update(config)
            params["seed"] = random_seed
            algo = params["name"]

            params["run"] = (
                f"{batch_id}"
                f"_G{params['nGrounds']}"
                f"_F{params['nFerrys']}"
                f"_GBS{params['groundBufferSize']}"
                f"_FBS{params['ferryBufferSize']}"
                f"_T{params['bundleTTL'] / 1000000}"
                f"_R{params['bundleGenRate']}"
                f"_C{int(params['ferryComm'])}"
                f"_seed{seed_id}"
            )
            cmd = build_cmd(PROGRAM, params)
            config_name = f"Config {config_id + 1}/{len(all_configs)} | Seed {seed_id + 1}/{N_SEEDS} | Algo: {algo}"
            tasks.append((cmd, config_name))

    print("==============================================")
    print(f"Total parameter configurations: {len(all_configs)}")
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
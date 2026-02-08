import subprocess
import os

# 1. Cấu hình các dải giá trị cần thử nghiệm (Grid Search)
scenarios = {
    "nGrounds": [ 25],
    "nFerrys": [5,10,15],           # Từ 1 đến 15
    "ferryBufferSize": [ 100,  200],
    "bundleTTL": [ 600]
}

# 2. Danh sách thuật toán
algorithms = ["sira", "pigeon"]

# 3. Tham số cố định
fixed_params = {
    "simTime": 5000,
    "commRange": 150
}

def run_command(cmd):
    print(f"\n--- Running: {cmd} ---")
    try:
        # Sử dụng shell=True để chạy được lệnh ./waf
        process = subprocess.Popen(cmd, shell=True)
        process.wait()
    except Exception as e:
        print(f"Error executing command: {e}")

def main():
    # Di chuyển ra thư mục gốc của NS-3 để chạy ./waf
    # Nếu file python này đã nằm ở gốc rồi thì có thể bỏ qua dòng os.chdir
    if os.path.basename(os.getcwd()) != "ns-allinone-3.xx": # Thay bằng tên thư mục của bạn nếu cần
         os.chdir("..")

    # Đếm tổng số lần chạy để theo dõi tiến độ
    total_runs = len(algorithms) * len(scenarios["nGrounds"]) * \
                 len(scenarios["nFerrys"]) * len(scenarios["ferryBufferSize"]) * \
                 len(scenarios["bundleTTL"])
    current_run = 0

    for algo in algorithms:
        for ng in scenarios["nGrounds"]:
            for nf in scenarios["nFerrys"]:
                for fbuf in scenarios["ferryBufferSize"]:
                    for ttl in scenarios["bundleTTL"]:
                        current_run += 1
                        
                        # Tạo chuỗi định danh 'run' để C++ log file không bị đè
                        # Ví dụ: g25_f5_b100_t300
                        run_id = f"g{ng}_f{nf}_b{fbuf}_t{ttl}"
                        
                        # Xây dựng lệnh command
                        # Lưu ý: file trong scratch có tên dạng ferry-sira.cc và ferry-pigeon.cc
                        cmd = (
                            f"./waf --run \"scratch/ferry-{algo} "
                            f"--name={algo} "
                            f"--run={run_id} "
                            f"--nGrounds={ng} "
                            f"--nFerrys={nf} "
                            f"--ferryBufferSize={fbuf} "
                            f"--bundleTTL={ttl} "
                            f"--simTime={fixed_params['simTime']} "
                            f"--commRange={fixed_params['commRange']}\""
                        )
                        
                        print(f"Progress: {current_run}/{total_runs}")
                        print(cmd)
                        run_command(cmd)

if __name__ == "__main__":
    main()
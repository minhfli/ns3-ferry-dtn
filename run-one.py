import subprocess
import os

NS3_DIR = "~/ns-allinone-3.30.1/ns-3.30.1"   # chỉnh lại cho đúng
PROGRAM = "ferry"

params = {
    "seed": 1337,
    "name": "SIRA",
    "run": "test",
    "simTime": 500,
    "commRange": 150,
    "ferryHeight": 50,
    "areaWidth": 1000,
    "nGrounds": 5,
    "nFerrys": 4,
    "ferrySpeed": 15,
    "groundBufferSize": 20,
    "ferryBufferSize": 30,
    "bundleGenRate": 60.0,
    "bundleTTL": 300000000,
    "ferryComm": False,
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
    if os.path.basename(os.getcwd()) == "scratch": # Thay bằng tên thư mục của bạn nếu cần
         os.chdir("..")
         
    cmd = build_cmd(PROGRAM, params)

    print("Running:", cmd)

    run_command(cmd)


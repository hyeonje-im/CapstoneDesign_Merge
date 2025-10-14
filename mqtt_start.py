import subprocess
import os
import socket

def get_local_ip():
    """현재 노트북의 로컬 IP 주소를 반환합니다."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 실제 연결은 발생하지 않지만, 소켓의 주소를 통해 로컬 IP를 결정할 수 있습니다.
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def run_mqtt():
    local_ip = get_local_ip()
    print("현재 노트북의 IP 주소:", local_ip)
    
    exe_path = r"C:\mosquitto\mosquitto.exe"
    conf_path = r"C:\mosquitto\mosquitto.conf"
    
    # 파일 존재 여부 확인
    if not os.path.exists(exe_path):
        print(f"파일을 찾을 수 없습니다: {exe_path}")
        return
    
    subprocess.call("taskkill /IM mosquitto.exe /F", shell=True)

    subprocess.Popen([exe_path, "-c", conf_path, "-v"], cwd=r"C:\mosquitto")

if __name__ == "__main__":
    run_mqtt()





import socket

UDP_IP = "0.0.0.0"   # 모든 네트워크 인터페이스에서 수신
UDP_PORT = 4210       # ESP8266과 맞춰야 할 포트 번호

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"[PC] UDP 수신 대기 중... (포트 {UDP_PORT})")
print("[PC] ESP8266에서 패킷이 오면 여기에 출력됩니다. 종료: Ctrl+C\n")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"[수신] {addr[0]}:{addr[1]} → {data.decode('utf-8', errors='replace')}")
except KeyboardInterrupt:
    print("\n[PC] 종료")
finally:
    sock.close()

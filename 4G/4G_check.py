import socket
import threading
import time
from datetime import datetime
import signal
import sys

# 存储活动的客户端套接字
active_connections = []
received_data_per_second = []
testing = False


def handle_client(client_socket, client_address):
    global received_data_per_second
    global testing

    # 将客户端套接字添加到活动连接列表中
    active_connections.append(client_socket)

    print(f"Handling client: {client_address}")
    while True:
        try:
            # 接收客户端发送的数据
            data = client_socket.recv(1024)
            if not data:
                print(f"Client {client_address} disconnected")
                break
            if testing:
                received_data_per_second[-1] += len(data)
        except Exception as e:
            print(f"Error receiving data from {client_address}: {e}")
            break

    # 从活动连接列表中移除客户端套接字
    active_connections.remove(client_socket)
    client_socket.close()


def start_server():
    global received_data_per_second
    global testing

    # 创建TCP套接字
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 8090))
    server_socket.listen(1)
    print('等待客户端连接...')

    while True:
        try:
            client_socket, client_address = server_socket.accept()
            print(f"客户端已连接: {client_address}")

            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
            client_thread.start()
        except Exception as e:
            print(f"Error accepting connections: {e}")
            break


def test_packet_loss():
    global received_data_per_second
    global testing
    duration = 10

    print("Starting packet loss test...")
    testing = True
    received_data_per_second = [0] * duration
    print("Test running for 60 seconds...")
    for _ in range(duration):
        time.sleep(1)
        received_data_per_second.append(0)
    testing = False

    # 计算丢包率
    theoretical_bytes_per_second = 1.5 * 1024 * 1024
    with open(f"test_results_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.txt", 'w') as f:
        for actual_received in received_data_per_second[:-1]:
            loss_rate = (theoretical_bytes_per_second - actual_received) / theoretical_bytes_per_second * 100
            log_entry = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - 理论: {theoretical_bytes_per_second}, 实际: {actual_received}, 丢包率: {loss_rate:.2f}%\n"
            f.write(log_entry)
            print(log_entry)


# 启动服务器和测试
server_thread = threading.Thread(target=start_server)
server_thread.start()

# 等待服务器启动
print("Waiting for server to start...")
time.sleep(5)  # 适当增加等待时间以确保服务器准备好

# 立即进行一次丢包率测试
print("Running packet loss test...")
test_packet_loss()

# 防止主线程退出，保持程序运行
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("服务器关闭")
import socket
import threading
import os
import time
import sys

HEADER_SIZE = 4  # 序列号的大小（字节）
DATA_SIZE = int(1.5 * 1024 * 1024)  # 数据大小
PACKET_SIZE = HEADER_SIZE + DATA_SIZE  # 数据包的总大小
FILE_START = b'START'  # 文件开始标识
FILE_END = b'END'      # 文件结束标识

SAVE_DIR = './Data/BIN4G'
os.makedirs(SAVE_DIR, exist_ok=True)

# 初始化序列号计数器
current_seq_num = 0
seq_num_lock = threading.Lock()

def handle_client(client_socket, client_address):
    global current_seq_num
    print(f"客户端已连接: {client_address}")

    while True:
        try:
            # Initialize receiving data
            file_data = b''
            while True:
                packet = client_socket.recv(PACKET_SIZE)
                if not packet:
                    break

                if packet.startswith(FILE_START):
                    packet = packet[len(FILE_START):]  # Remove start marker

                if packet.endswith(FILE_END):
                    packet = packet[:-len(FILE_END)]  # Remove end marker
                    file_data += packet
                    break
                else:
                    file_data += packet

            if file_data:
                # Process received file data
                processed_length = process_received_data(file_data, client_socket)

                # Ensure acknowledgment is sent
                if processed_length > 0:
                    client_socket.sendall(b'ACK')
        except Exception as e:
            print(f"接收数据时发生错误: {e}")
            break

    client_socket.close()

def process_received_data(data, client_socket):
    global current_seq_num
    print(f"接收到数据包: {len(data)} 字节")

    # Use lock to protect sequence number counter
    with seq_num_lock:
        seq_num = current_seq_num
        current_seq_num += 1

    # Save data
    save_data(seq_num, data)

    return len(data)

def save_data(seq_num, data_chunk):
    print(f"保存数据: 序列号 {seq_num}, 大小 {len(data_chunk)}")
    file_path = os.path.join(SAVE_DIR, f"frame_{seq_num}.bin")
    with open(file_path, 'wb') as f:
        f.write(data_chunk)

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 8090))
    server_socket.listen(1)
    print('等待客户端连接...')

    while True:
        try:
            client_socket, client_address = server_socket.accept()
            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
            client_thread.start()
        except Exception as e:
            print(f"接收连接时发生错误: {e}")
            break

# 启动服务器
server_thread = threading.Thread(target=start_server)
server_thread.start()

print("等待服务器启动...")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("服务器关闭")

import os
import socket
import threading
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

HOST = 'localhost'  # 服务器地址
PORT = 8090          # 服务器端口
ADDR = (HOST, PORT)

FILE_START = b'START'  # 文件开始标识
FILE_END = b'END'      # 文件结束标识

class FileMonitor(FileSystemEventHandler):
    def __init__(self, watch_dir, socket):
        self.watch_dir = watch_dir
        self.socket = socket
        self.stop_event = threading.Event()
        self.lock = threading.Lock()  # 添加锁
        super().__init__()

    def on_created(self, event):
        if event.is_directory:
            return

        file_path = event.src_path
        print(f"发现新文件: {file_path}")

        self.lock.acquire()
        try:
            with open(file_path, 'rb') as file:
                file_data = file.read()
                sequence_number = int(time.time() * 1000) % (2**32)  # 确保序列号在范围内
                packet = FILE_START + file_data + FILE_END
                self.socket.sendall(packet)
                print(f"文件发送成功， {file_path}，大小: {len(file_data)}")

                # 等待服务器确认收到文件
                self.wait_for_acknowledgment()
        except Exception as e:
            print(f"发送文件时发生错误: {e}")
        finally:
            self.lock.release()

    def wait_for_acknowledgment(self):
        try:
            self.socket.settimeout(10)  # 设置超时时间
            ack = self.socket.recv(4)
            if ack == b'ACK':
                print("收到服务器确认")
            else:
                print("收到无效确认")
        except socket.timeout:
            print("等待确认超时")
        except Exception as e:
            print(f"接收确认时发生错误: {e}")

def setup_connection():
    tcpCliSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcpCliSock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)  # 开启心跳维护
    tcpCliSock.connect(ADDR)
    return tcpCliSock

def monitor_directory(watch_dir, socket):
    event_handler = FileMonitor(watch_dir, socket)
    observer = Observer()
    observer.schedule(event_handler, path=watch_dir, recursive=False)
    observer.start()
    try:
        while not event_handler.stop_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        event_handler.stop_event.set()
    finally:
        observer.stop()
        observer.join()

def main():
    watch_dir = './Data/BIN'  # 需要监视的目录
    if not os.path.exists(watch_dir):
        os.makedirs(watch_dir)
    tcpCliSock = setup_connection()

    monitor_thread = threading.Thread(target=monitor_directory, args=(watch_dir, tcpCliSock))
    monitor_thread.start()

    try:
        monitor_thread.join()
    except KeyboardInterrupt:
        print("停止监视文件夹")
        tcpCliSock.close()

if __name__ == "__main__":
    print("开始监视文件夹")
    main()

import socket
import threading
import os
import time
import json
import struct
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import os
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

HEADER_SIZE = 4  # 序列号的大小（字节）
DATA_SIZE = int(1.5 * 1024 * 1024)  # 数据大小
PACKET_SIZE = HEADER_SIZE + DATA_SIZE  # 数据包的总大小
FILE_START = b'START'  # 文件开始标识
FILE_END = b'END'      # 文件结束标识

DECODE_DIR = './Data/decode'
os.makedirs(DECODE_DIR, exist_ok=True)

# 初始化序列号计数器
current_seq_num = 0
seq_num_lock = threading.Lock()

# 初始化 ROS 节点
rospy.init_node('data_publisher', anonymous=True)

# 创建发布器
pcd_topic = '/cloud_registered_body4G'
odom_topic = '/Odometry4G'
img_topic = '/image4G'
thermal_topic = '/thermal4G'
pcd_publisher = rospy.Publisher(pcd_topic, PointCloud2, queue_size=10)
odom_publisher = rospy.Publisher(odom_topic, Odometry, queue_size=10)
img_publisher = rospy.Publisher(img_topic, Image, queue_size=10)
thermal_publisher = rospy.Publisher(thermal_topic, Image, queue_size=10)
bridge = CvBridge()

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
    print(f"Receive data, frame:{current_seq_num} , size: {len(data)} bytes")

    # Use lock to protect sequence number counter
    with seq_num_lock:
        seq_num = current_seq_num
        current_seq_num += 1

    # Decode data
    if len(data) >= 4:  # Ensure there's enough data to unpack
        decode_bin_data(data, seq_num)
    else:
        print("收到的数据长度不足，无法解码")

    return len(data)

def save_bin_data(data, seq_num):
    frame_index = seq_num
    json_path = os.path.join(DECODE_DIR, 'JSON', f'frame_{frame_index}.json')
    jpg_path = os.path.join(DECODE_DIR, 'JPG', f'frame_{frame_index}.jpg')
    thermal_jpg_path = os.path.join(DECODE_DIR, 'THERMAL_JPG', f'frame_{frame_index}_thermal.jpg')
    pcd_3d_path = os.path.join(DECODE_DIR, '3D', f'frame_{frame_index}.pcd')
    pcd_2d_path = os.path.join(DECODE_DIR, '2D', f'frame_{frame_index}.pcd')

    os.makedirs(os.path.join(DECODE_DIR, 'JSON'), exist_ok=True)
    os.makedirs(os.path.join(DECODE_DIR, 'JPG'), exist_ok=True)
    os.makedirs(os.path.join(DECODE_DIR, 'THERMAL_JPG'), exist_ok=True)
    os.makedirs(os.path.join(DECODE_DIR, '3D'), exist_ok=True)
    os.makedirs(os.path.join(DECODE_DIR, '2D'), exist_ok=True)

    offset = 0

    # Decode JSON data
    if len(data) < offset + 4:
        print("数据不足，无法解码 JSON 长度")
        return
    json_length = struct.unpack('I', data[offset:offset+4])[0]
    offset += 4

    if len(data) < offset + json_length:
        print("数据不足，无法解码 JSON 数据")
        return
    json_data = json.loads(data[offset:offset+json_length])
    offset += json_length

    with open(json_path, 'w') as json_file:
        json.dump(json_data, json_file, indent=4)

    # Decode 3D point cloud data
    num_points_3d = json_data.get('pcd_num_3d', 0)
    if len(data) < offset + num_points_3d * struct.calcsize('ffff'):
        print("数据不足，无法解码 3D 点云数据")
        return
    binary_bin_to_ascii_pcd(data[offset:offset + num_points_3d * struct.calcsize('ffff')], pcd_3d_path, num_points_3d)
    offset += num_points_3d * struct.calcsize('ffff')

    # Decode 2D point cloud data
    num_points_2d = json_data.get('pcd_num_2d', 0)
    if len(data) < offset + num_points_2d * struct.calcsize('ffff'):
        print("数据不足，无法解码 2D 点云数据")
        return
    binary_bin_to_ascii_pcd(data[offset:offset + num_points_2d * struct.calcsize('ffff')], pcd_2d_path, num_points_2d)
    offset += num_points_2d * struct.calcsize('ffff')

    # Decode JPEG image data
    if len(data) < offset + 4:
        print("数据不足，无法解码 JPEG 长度")
        return
    jpg_length = struct.unpack('I', data[offset:offset+4])[0]
    offset += 4

    if len(data) < offset + jpg_length:
        print("数据不足，无法解码 JPEG 数据")
        return
    jpg_data = data[offset:offset+jpg_length]
    with open(jpg_path, 'wb') as jpg_file:
        jpg_file.write(jpg_data)
    offset += jpg_length

    # Decode thermal JPEG image data
    if len(data) < offset + 4:
        print("数据不足，无法解码热成像 JPEG 长度")
        return
    thermal_jpg_length = struct.unpack('I', data[offset:offset+4])[0]
    offset += 4

    if len(data) < offset + thermal_jpg_length:
        print("数据不足，无法解码热成像 JPEG 数据")
        return
    thermal_jpg_data = data[offset:offset+thermal_jpg_length]
    with open(thermal_jpg_path, 'wb') as thermal_jpg_file:
        thermal_jpg_file.write(thermal_jpg_data)

def decode_bin_data(data, seq_num):
    frame_index = seq_num

    offset = 0

    # 解码 JSON 数据
    if len(data) < offset + 4:
        print("数据不足，无法解码 JSON 长度")
        return
    json_length = struct.unpack('I', data[offset:offset + 4])[0]
    offset += 4

    if len(data) < offset + json_length:
        print("数据不足，无法解码 JSON 数据")
        return
    json_data = json.loads(data[offset:offset + json_length])
    offset += json_length

    # 发布里程计数据
    try:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        timestamp = rospy.Time.from_sec(json_data.get("device_time", 0))
        pos = json_data.get("pos_yaw", [0, 0, 0, 0])[:3]
        yaw = json_data.get("pos_yaw", [0, 0, 0, 0])[3]
        quat = quaternion_from_euler(0, 0, yaw)

        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position.x = pos[0]
        odom_msg.pose.pose.position.y = pos[1]
        odom_msg.pose.pose.position.z = pos[2]
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_publisher.publish(odom_msg)
        rospy.loginfo(f"Published odometry data for frame {frame_index}")
    except Exception as e:
        rospy.logerr(f"Error processing JSON data for frame {frame_index}: {e}")

    # 解码3D点云数据
    num_points_3d = json_data.get('pcd_num_3d', 0)
    try:
        points_3d = []
        for i in range(num_points_3d):
            point_data = data[offset + i * struct.calcsize('ffff'):offset + (i + 1) * struct.calcsize('ffff')]
            x, y, z, intensity = struct.unpack('ffff', point_data)
            points_3d.append((x, y, z))
        offset += num_points_3d * struct.calcsize('ffff')

        # 发布3D点云数据
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        pcd_msg = pc2.create_cloud_xyz32(header, points_3d)
        pcd_publisher.publish(pcd_msg)
        rospy.loginfo(f"Published 3D point cloud data for frame {frame_index}")
    except Exception as e:
        rospy.logerr(f"Error publishing 3D point cloud data for frame {frame_index}: {e}")

    # 解码2D点云数据
    num_points_2d = json_data.get('pcd_num_2d', 0)
    try:
        points_2d = []
        for i in range(num_points_2d):
            point_data = data[offset + i * struct.calcsize('ffff'):offset + (i + 1) * struct.calcsize('ffff')]
            x, y, z, intensity = struct.unpack('ffff', point_data)
            points_2d.append((x, y, z))
        offset += num_points_2d * struct.calcsize('ffff')

        # 发布2D点云数据（如果需要，可以单独发布或与3D点云合并）
        # 这里只是发布2D点云数据的占位代码
    except Exception as e:
        rospy.logerr(f"Error processing 2D point cloud data for frame {frame_index}: {e}")

    # 解码并发布 JPEG 图像数据
    try:
        if len(data) < offset + 4:
            print("数据不足，无法解码 JPEG 长度")
            return
        jpg_length = struct.unpack('I', data[offset:offset + 4])[0]
        offset += 4

        if len(data) < offset + jpg_length:
            print("数据不足，无法解码 JPEG 数据")
            return
        jpg_data = data[offset:offset + jpg_length]
        offset += jpg_length

        img_data = np.frombuffer(jpg_data, dtype=np.uint8)
        img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header = header
        img_publisher.publish(img_msg)
        rospy.loginfo(f"Published image data for frame {frame_index}")
    except Exception as e:
        rospy.logerr(f"Error publishing image data for frame {frame_index}: {e}")

    # 解码并发布热成像 JPEG 图像数据
    try:
        if len(data) < offset + 4:
            print("数据不足，无法解码热成像 JPEG 长度")
            return
        thermal_jpg_length = struct.unpack('I', data[offset:offset + 4])[0]
        offset += 4

        if len(data) < offset + thermal_jpg_length:
            print("数据不足，无法解码热成像 JPEG 数据")
            return
        thermal_jpg_data = data[offset:offset + thermal_jpg_length]

        thermal_img_data = np.frombuffer(thermal_jpg_data, dtype=np.uint8)
        thermal_img = cv2.imdecode(thermal_img_data, cv2.IMREAD_COLOR)
        thermal_msg = bridge.cv2_to_imgmsg(thermal_img, encoding="bgr8")
        thermal_msg.header = header
        thermal_publisher.publish(thermal_msg)
        rospy.loginfo(f"Published thermal image data for frame {frame_index}")
    except Exception as e:
        rospy.logerr(f"Error publishing thermal image data for frame {frame_index}: {e}")



def binary_bin_to_ascii_pcd(data, pcd_path, num_points):
    point_size = struct.calcsize('ffff')
    header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {width}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0 
POINTS {points}
DATA ascii
'''.format(width=num_points, points=num_points)

    with open(pcd_path, 'w') as ascii_file:
        ascii_file.write(header)
        for i in range(num_points):
            point_data = data[i * point_size:(i + 1) * point_size]
            x, y, z, intensity = struct.unpack('ffff', point_data)
            ascii_file.write('{:.6f} {:.6f} {:.6f} {:.6f}\n'.format(x, y, z, intensity))

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

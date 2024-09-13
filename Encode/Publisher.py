#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import os
import json
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import time

class DataPublisher:
    def __init__(self, pcd_dir, json_dir, jpg_dir, thermal_dir, pointcloud_topic, odom_topic, img_topic, thermal_topic):
        print("DataPublisher Initialized")
        self.pcd_dir = pcd_dir
        self.json_dir = json_dir
        self.jpg_dir = jpg_dir
        self.thermal_dir = thermal_dir
        self.pointcloud_topic = pointcloud_topic
        self.odom_topic = odom_topic
        self.img_topic = img_topic
        self.thermal_topic = thermal_topic

        # 初始化 ROS 节点
        rospy.init_node('data_publisher', anonymous=True)

        # 创建发布器
        self.pc_publisher = rospy.Publisher(self.pointcloud_topic, PointCloud2, queue_size=10)
        self.odom_publisher = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.img_publisher = rospy.Publisher(self.img_topic, Image, queue_size=10)
        self.thermal_publisher = rospy.Publisher(self.thermal_topic, Image, queue_size=10)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        # 开始观察目录
        self.start_observer()

    def read_pcd(self, filename):
        # 读取 PCD 文件
        points = []
        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
                for line in lines[11:]:
                    parts = line.strip().split()
                    if len(parts) == 4:
                        points.append([float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])])
        except Exception as e:
            rospy.logerr(f"Failed to read PCD file {filename}: {e}")
        return points

    def read_json(self, filename):
        # 读取 JSON 文件
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
        except Exception as e:
            rospy.logerr(f"Failed to read JSON file {filename}: {e}")
            data = None
        return data
    
    def read_image(self, filename):
        # 读取图像文件
        image_data = cv2.imread(filename)
        if image_data is None:
            rospy.logerr(f"Failed to read image from {filename}")
            return np.zeros((1, 1, 3), dtype=np.uint8)  # 返回一个空图像作为占位符
        return image_data
    
    def read_thermal(self, filename):
        # 读取热成像图像文件
        image_data = cv2.imread(filename)
        if image_data is None:
            rospy.logerr(f"Failed to read thermal image from {filename}")
            return np.zeros((1, 1, 3), dtype=np.uint8)
        return image_data  # 确保返回图像数据

    def publish_data(self, frame_index):
        pcd_file = os.path.join(self.pcd_dir, f"frame_{frame_index}.pcd")
        json_file = os.path.join(self.json_dir, f"frame_{frame_index}.json")
        jpg_file = os.path.join(self.jpg_dir, f"frame_{frame_index}.jpg")
        thermal_file = os.path.join(self.thermal_dir, f"frame_{frame_index}_thermal.jpg")

        if os.path.exists(pcd_file):
            # 读取并发布点云数据
            points = self.read_pcd(pcd_file)
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            pc_msg = pc2.create_cloud_xyz32(header, [(p[0], p[1], p[2]) for p in points])
            self.pc_publisher.publish(pc_msg)
            rospy.loginfo(f"Published point cloud data for frame {frame_index}")
        else:
            rospy.logwarn(f"PCD file not found for frame {frame_index}: {pcd_file}")

        if os.path.exists(json_file):
            # 读取并发布里程计数据
            json_data = self.read_json(json_file)
            if json_data:
                try:
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
                    self.odom_publisher.publish(odom_msg)
                    rospy.loginfo(f"Published odometry data for frame {frame_index}")
                except KeyError as e:
                    rospy.logerr(f"Missing expected key in JSON data for frame {frame_index}: {e}")
                except Exception as e:
                    rospy.logerr(f"Error processing JSON data for frame {frame_index}: {e}")
            else:
                rospy.logwarn(f"JSON data is not valid for frame {frame_index}")
        else:
            rospy.logwarn(f"JSON file not found for frame {frame_index}: {json_file}")

        if os.path.exists(jpg_file):
            # 读取并发布图像数据
            image_data = self.read_image(jpg_file)
            if image_data is not None:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(image_data, encoding="bgr8")
                    img_msg.header = header
                    self.img_publisher.publish(img_msg)
                    rospy.loginfo(f"Published image data for frame {frame_index}")
                except Exception as e:
                    rospy.logerr(f"Error converting image data for frame {frame_index}: {e}")
            else:
                rospy.logwarn(f"Image data is not valid for frame {frame_index}")
        else:
            rospy.logwarn(f"Image file not found for frame {frame_index}: {jpg_file}")

        if os.path.exists(thermal_file):
            # 读取并发布热成像图像数据
            thermal_data = self.read_thermal(thermal_file)
            if thermal_data is not None:
                try:
                    thermal_msg = self.bridge.cv2_to_imgmsg(thermal_data, encoding="bgr8")
                    thermal_msg.header = header
                    self.thermal_publisher.publish(thermal_msg)
                    rospy.loginfo(f"Published thermal image data for frame {frame_index}")
                except Exception as e:
                    rospy.logerr(f"Error converting thermal image data for frame {frame_index}: {e}")
            else:
                rospy.logwarn(f"Thermal image data is not valid for frame {frame_index}")
        else:
            rospy.logwarn(f"Thermal image file not found for frame {frame_index}: {thermal_file}")

    def start_observer(self):
        try:
            event_handler = FileChangeHandler(self)
            observer = Observer()
            os.makedirs(self.pcd_dir, exist_ok=True)
            os.makedirs(self.json_dir, exist_ok=True)
            os.makedirs(self.jpg_dir, exist_ok=True)
            os.makedirs(self.thermal_dir, exist_ok=True)

            observer.schedule(event_handler, path=self.pcd_dir, recursive=False)
            observer.schedule(event_handler, path=self.json_dir, recursive=False)
            observer.schedule(event_handler, path=self.jpg_dir, recursive=False)
            observer.schedule(event_handler, path=self.thermal_dir, recursive=False)
            observer.start()
            rospy.loginfo("Started observing directories")
            rospy.spin()
        except FileNotFoundError as e:
            rospy.logerr(f"Directory not found: {e}")
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

class FileChangeHandler(FileSystemEventHandler):
    def __init__(self, data_publisher):
        self.data_publisher = data_publisher

    def on_created(self, event):
        if not event.is_directory and event.src_path.endswith('.pcd'):
            rospy.loginfo(f"New PCD file detected: {event.src_path}. Waiting before processing...")
            time.sleep(0.05)  # Wait for 0.5 seconds before processing
            frame_index = self.extract_frame_index(event.src_path)
            if frame_index != -1:
                self.data_publisher.publish_data(frame_index)

    def extract_frame_index(self, file_path):
        filename = os.path.basename(file_path)
        try:
            frame_index = int(filename.split('_')[1].split('.')[0])
        except (IndexError, ValueError) as e:
            rospy.logerr(f"Failed to extract frame index from filename {filename}: {e}")
            frame_index = -1  # Default to an invalid index
        return frame_index

if __name__ == '__main__':
    pcd_dir = './Data/decode/3D'   # 保存 PCD 文件的目录
    json_dir = './Data/decode/JSON' # 保存 JSON 文件的目录
    jpg_dir = './Data/decode/JPG'   # 保存图像文件的目录
    thermal_dir = './Data/decode/THERMAL_JPG'  # 保存热成像图像文件的目录
    pcd_topic = '/cloud_registered_body4G'
    odom_topic = '/Odometry4G'
    img_topic = '/image4G'
    thermal_topic = '/thermal4G'

    DataPublisher(pcd_dir, json_dir, jpg_dir, thermal_dir, pcd_topic, odom_topic, img_topic, thermal_topic)

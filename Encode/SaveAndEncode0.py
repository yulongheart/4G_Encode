import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
import numpy as np
import cv2
import os
import math
import random
from collections import defaultdict, deque, OrderedDict
import sensor_msgs.point_cloud2 as pc2
import json
import struct


class DataCollector:
    def __init__(self, image_topic, pointcloud_topic, odometry_topic, save_dir, time_tolerance=0.1):
        self.current_frame = 0
        self.save_dir = save_dir
        self.time_tolerance = time_tolerance
        self.odometry_position = None
        self.odometry_orientation = None
        self.pointcloud_frames = []

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.image_queue = deque()
        self.odometry_queue = deque()

        image_sub = message_filters.Subscriber(image_topic, Image)
        pointcloud_sub = message_filters.Subscriber(pointcloud_topic, PointCloud2)
        odometry_sub = rospy.Subscriber(odometry_topic, Odometry, self.odometry_callback)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, pointcloud_sub], 10, time_tolerance)
        self.ts.registerCallback(self.queue_data)

    def odometry_callback(self, odometry_data):
        self.odometry_queue.append(odometry_data)

    def queue_data(self, image_data, pointcloud_data):
        self.image_queue.append(image_data)
        self.pointcloud_frames.append(pointcloud_data)

        if len(self.pointcloud_frames) == 10:
            self.process_and_save()

    def find_closest_odometry(self, timestamp):
        if not self.odometry_queue:
            return None

        # Make a copy of the deque to prevent mutation during iteration
        odometry_copy = list(self.odometry_queue)

        closest_odometry = min(odometry_copy, key=lambda odom: abs(odom.header.stamp - timestamp))
        return closest_odometry

    def find_closest_image(self, timestamp):
        if not self.image_queue:
            return None

        closest_image = min(self.image_queue, key=lambda img: abs(img.header.stamp - timestamp))
        return closest_image

    def transform_point(self, point, transform_matrix):
        homogenous_point = np.array([point[0], point[1], point[2], 1.0])
        transformed_point = np.dot(transform_matrix, homogenous_point)
        return transformed_point[:3], point[3]

    def process_and_save(self):
        final_pointcloud = []
        last_pointcloud = self.pointcloud_frames[-1]
        timestamp = last_pointcloud.header.stamp

        for pcl in self.pointcloud_frames:
            closest_odometry = self.find_closest_odometry(pcl.header.stamp)
            if closest_odometry:
                transform_matrix = self.get_transform_matrix(closest_odometry)
                for point in pc2.read_points(pcl, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                    transformed_point, intensity = self.transform_point(point, transform_matrix)
                    final_pointcloud.append(
                        (transformed_point[0], transformed_point[1], transformed_point[2], intensity))

        closest_image = self.find_closest_image(timestamp)
        closest_odometry = self.find_closest_odometry(timestamp)

        self.process_frame(closest_image, final_pointcloud, closest_odometry)

        self.pointcloud_frames = []
        self.image_queue = deque(filter(lambda img: img.header.stamp >= timestamp, self.image_queue))
        self.odometry_queue = deque(filter(lambda odom: odom.header.stamp >= timestamp, self.odometry_queue))

    def process_frame(self, image_data, fused_pointcloud, odometry_data):
        self.process_image(image_data)
        self.process_pointcloud(fused_pointcloud)
        if odometry_data:
            self.process_odometry(odometry_data)
        else:
            self.odometry_position = None
            self.odometry_orientation = None
        self.save_bin_file()
        self.current_frame += 1

    def process_image(self, data):
        try:
            height = data.height
            width = data.width
            image_data = np.frombuffer(data.data, dtype=np.uint8).reshape((height, width, -1))
            resized_image = cv2.resize(image_data, (640, 480))
            _, jpg_data = cv2.imencode('.jpg', resized_image)
            self.image_data = jpg_data.tobytes()
        except Exception as e:
            rospy.logerr("Failed to process image: {}".format(e))

    def process_pointcloud(self, points):
        self.pointcloud_3d_data = self.convert_points_to_binary(points)
        points_2d = self.convert_to_2d(points)
        points_2d_sampled = self.sample_points(points_2d)
        self.pointcloud_2d_data = self.convert_points_to_binary(points_2d_sampled)
        self.generate_json(points, points_2d_sampled)

    def process_odometry(self, data):
        self.odometry_position = [data.pose.pose.position.x,
                                  data.pose.pose.position.y,
                                  data.pose.pose.position.z]
        self.odometry_orientation = [data.pose.pose.orientation.x,
                                     data.pose.pose.orientation.y,
                                     data.pose.pose.orientation.z,
                                     data.pose.pose.orientation.w]

    def get_transform_matrix(self, odometry_data):
        position = [odometry_data.pose.pose.position.x, odometry_data.pose.pose.position.y,
                    odometry_data.pose.pose.position.z]
        orientation = [odometry_data.pose.pose.orientation.x, odometry_data.pose.pose.orientation.y,
                       odometry_data.pose.pose.orientation.z, odometry_data.pose.pose.orientation.w]
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation)
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = position
        return transform_matrix

    def quaternion_to_rotation_matrix(self, quaternion):
        x, y, z, w = quaternion
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
        ])

    def convert_points_to_binary(self, points):
        binary_data = bytearray()
        for point in points:
            binary_data.extend(struct.pack('ffff', float(point[0]), float(point[1]), float(point[2]), float(point[3])))
        return binary_data

    def convert_to_2d(self, points):
        return [(x, y, 0.0, intensity) for x, y, _, intensity in points]

    def sample_points(self, points, distance_threshold=0.05):
        grid = defaultdict(list)
        grid_size = distance_threshold
        for point in points:
            grid_key = (int(point[0] // grid_size), int(point[1] // grid_size))
            grid[grid_key].append(point)
        return [self.average_points(cell_points) for cell_points in grid.values()]

    def average_points(self, points):
        avg_x = sum(point[0] for point in points) / len(points)
        avg_y = sum(point[1] for point in points) / len(points)
        avg_intensity = sum(point[3] for point in points) / len(points)
        return avg_x, avg_y, 0.0, avg_intensity

    def quaternion_to_yaw(self, quat):
        _, _, z, w = quat
        return math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))

    def generate_json(self, points_3d, points_2d):
        timestamp = rospy.Time.now().to_sec()
        if self.odometry_position and self.odometry_orientation:
            yaw = self.quaternion_to_yaw(self.odometry_orientation)
            pos_yaw = self.odometry_position + [round(yaw, 6)]
        else:
            pos_yaw = None

        self.json_data = OrderedDict([
            ("device_id", 100002),
            ("device_time", timestamp),
            ("pos_yaw", pos_yaw),
            ("floor", 7),
            ("is_exit", random.random() < 0.1),
            ("fire_point", self.get_fire_point()),
            ("3dpcd_num", len(points_3d)),
            ("2dpcd_num", len(points_2d))
        ])

    def get_fire_point(self):
        if self.odometry_position:
            return self.odometry_position if random.random() < 0.1 else None
        return None

    def save_bin_file(self):
        if self.image_data and self.pointcloud_3d_data and self.pointcloud_2d_data and self.json_data:
            bin_filename = os.path.join(self.save_dir, "frame_{}.bin".format(self.current_frame + 1))
            with open(bin_filename, 'wb') as bin_file:
                # Encode JSON data
                json_data = json.dumps(self.json_data).encode('utf-8')
                bin_file.write(struct.pack('I', len(json_data)))
                bin_file.write(json_data)

                # Encode 3D point cloud data
                bin_file.write(self.pointcloud_3d_data)

                # Encode 2D point cloud data
                bin_file.write(self.pointcloud_2d_data)

                # Encode JPEG image data
                bin_file.write(struct.pack('I', len(self.image_data)))
                bin_file.write(self.image_data)

            rospy.loginfo("Saved BIN file: {}".format(bin_filename))

        # Reset data for the next frame
        self.image_data = None
        self.pointcloud_3d_data = None
        self.pointcloud_2d_data = None
        self.json_data = None


if __name__ == '__main__':
    print("Data Encode ...")
    image_topic = '/iray/thermal_img'
    pointcloud_topic = '/cloud_registered_body'
    odometry_topic = '/Odometry'  # Odometry topic
    save_dir = './Data/BIN'
    time_tolerance = 0.1  # Maximum allowed time difference between synchronized messages

    rospy.init_node('data_collector', anonymous=True)

    data_collector = DataCollector(image_topic, pointcloud_topic, odometry_topic, save_dir, time_tolerance)

    rospy.spin()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import tf2_ros
import os
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import time
import yaml
from rclpy.serialization import deserialize_message
import importlib

class BagConverter(Node):
    def __init__(self, bag_path, output_dir):
        super().__init__('bag_converter')
        self.bag_path = bag_path
        self.output_dir = output_dir
        
        # 创建输出目录
        os.makedirs(os.path.join(output_dir, 'pointcloud'), exist_ok=True)
        
        # 初始化TF缓存
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 读取bag
        self.reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr',
                                             output_serialization_format='cdr')
        self.reader.open(storage_options, converter_options)
        
        # 处理消息
        self.process_bag()
        
    def process_bag(self):
        scan_msgs = []
        tf_msgs = []
        topic_types = {}
        for topic_metadata in self.reader.get_all_topics_and_types():
            topic_types[topic_metadata.name] = topic_metadata.type
        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            
            if topic == '/scan':
                msg_type = self.get_message_type(topic_types[topic])
                scan_msg = deserialize_message(data, msg_type)
                scan_msgs.append((scan_msg, t))
            elif topic == '/tf' or topic == '/tf_static':
                msg_type = self.get_message_type(topic_types[topic])
                tf_msg = deserialize_message(data, msg_type)
                tf_msgs.append((tf_msg, t))
        
        info_file = open(os.path.join(self.output_dir, 'info.txt'), 'w')
        for scan_msg, scan_time in scan_msgs:
            points = self.laserscan_to_pointcloud(scan_msg)
            time_ns = scan_msg.header.stamp.sec * 10**9 + scan_msg.header.stamp.nanosec
            cloud_file = os.path.join(self.output_dir, 'pointcloud', f"{time_ns}.txt")
            with open(cloud_file, 'w') as f:
                for point in points:
                    f.write(f"{point[0]} {point[1]}\n")
            
            x, y, z = 0.0, 0.0, 0.0
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            
            info_file.write(f"{time_ns} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
            self.get_logger().info(f"处理扫描: {time_ns}")
        
        info_file.close()
        self.create_config_file()
        
    def laserscan_to_pointcloud(self, scan_msg):
        points = []
        angle = scan_msg.angle_min
        
        for r in scan_msg.ranges:
            if r >= scan_msg.range_min and r <= scan_msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))
            angle += scan_msg.angle_increment
            
        return points
    
    def create_config_file(self):
        config = {
            'create_gridmap_node': {
                'laser_to_baselink': {
                    'position': [0.0, 0.0, 0.0],
                    'quaternion': [0.0, 0.0, 0.0, 1.0]
                },
                'create_map_path': self.output_dir,
                'filter_param': {
                    'max_angle_radians': 0.0175,
                    'max_distance_meters': 0.2,
                    'max_length': 0.05,
                    'max_time_seconds': 160,
                    'min_num_points': 200,
                    'voxel_filter_size': 0.025
                },
                'ceres_param': {
                    'use_nonmonotonic_steps': False,
                    'max_num_iterations': 20,
                    'num_threads': 1,
                    'occupied_space_weight': 10.0,
                    'translation_weight': 20.0,
                    'rotation_weight': 40.0
                },
                'probability_param': {
                    'p_occ': 0.55,
                    'p_free': 0.45,
                    'p_prior': 0.5
                },
                'submap_param': {
                    'resolution': 0.05,
                    'sizex': 100,
                    'sizey': 100,
                    'initx': 100,
                    'inity': 100,
                    'num_accumulated': 35,
                    'missing_data_ray_length': 6.0,
                    'max_range': 8.0,
                    'min_range': 0.2
                },
                'mapbeauti': {
                    'side_fill_thresh': 0.3,
                    'approx_poly_thresh': 0.01,
                    'dilate_kernel_size': 3,
                    'noise_removal_thresh': 100
                }
            }
        }
        
        with open(os.path.join(self.output_dir, 'gridmap_node.yaml'), 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

    def get_message_type(self, type_str):
        parts = type_str.split('/')
        if len(parts) != 3:
            self.get_logger().error(f"无效的消息类型: {type_str}")
            return None
            
        package_name, _, message_name = parts
        module_name = f"{package_name}.msg"
        
        try:
            module = importlib.import_module(module_name)
            return getattr(module, message_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"无法导入消息类型 {type_str}: {e}")
            return None

def main():
    if len(sys.argv) < 3:
        print("Usage: convert_bag_to_demo_format.py <bag_path> <output_dir>")
        return
        
    rclpy.init()
    converter = BagConverter(sys.argv[1], sys.argv[2])
    rclpy.shutdown()

if __name__ == '__main__':
    main()
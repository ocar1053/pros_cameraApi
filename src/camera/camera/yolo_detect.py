import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
import os
import threading
from ultralytics import YOLO  # Import YOLOv8 library
from .convert_camera_coordinates import convert_camera_coordinates
import pickle

class Pros_pokemon_yolo(Node):

    def __init__(self):
        super().__init__('pokemon_yolo_node')
        self.model_name = 'yolov8n.pt'
        self.get_logger().info('Pokemon Node is running using ' + self.model_name)
        self.bridge = CvBridge()
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.model_path = os.path.join(self.current_dir, '../', 'resource', self.model_name)
        self.model = YOLO(self.model_path)

        self.__depth_lock = threading.Lock()
        self.__depth_image = None

        # Load camera parameters for world coordinate transformation
        with open("/workspaces/src/camera/resource/camera.pkl", "rb") as infile:
            camera = pickle.load(infile)
            camera_matrix = camera["camera_matrix"]
            extrinsic_matrix = camera["extrinsic_matrix"]
            rotation_matrix = camera["rotation_matrix"]

        camera_transform = extrinsic_matrix[:, -1][:-1].reshape(1, -1)
        self.convert_camera = convert_camera_coordinates(camera_matrix, extrinsic_matrix, rotation_matrix, camera_transform)

        # Subscribe to the depth image topic
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.listener_depth_callback,
            10
        )

        # Subscribe to the compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/out/compressed',
            self.listener_callback,
            10
        )

        # Publisher for the processed image
        self.processed_image_pub = self.create_publisher(
            CompressedImage,
            '/processed_image_yolo_pokemon/compressed',
            10
        )

        # Publisher for the detection data
        self.detection_data_pub = self.create_publisher(
            String,
            '/detection_data/yolo_pokemon',
            10
        )

    def listener_depth_callback(self, data):
        # Store the latest depth image in an OpenCV format
        with self.__depth_lock:
            self.__depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def listener_callback(self, msg):
        self.get_logger().info('Received frame')

        # Convert the ROS image message to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform object detection using YOLOv8
        results = self.model(frame, conf=0.85)

        detection_results = []

        for result in results:
            boxes = result.boxes  # Get detected bounding boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                conf = float(box.conf[0])  # Confidence score
                cls = int(box.cls[0])  # Class index
                label = self.model.names[cls]  # Class label

                # Calculate center coordinates (u, v)
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # Get depth value at the center of the bounding box
                with self.__depth_lock:
                    if self.__depth_image is not None and 0 <= v < self.__depth_image.shape[0] and 0 <= u < self.__depth_image.shape[1]:
                        depth_value = float(self.__depth_image[v, u])  # Depth value in millimeters or meters
                    else:
                        depth_value = float('nan')

                # Transform pixel coordinates and depth to world coordinates
                if not np.isnan(depth_value):
                    world_position = self.convert_camera.screen_point_to_world_point(np.array([[u, v, 1]]), depth_value / 1000)
                    world_coords = {
                        'world_x': round(float(world_position[0]), 3),
                        'world_y': round(float(world_position[1]), 3),
                        'world_z': round(float(world_position[2]), 3)
                    }
                else:
                    world_coords = {'world_x': None, 'world_y': None, 'world_z': None}

                detection_results.append({
                    'class_id': cls,
                    'label': label,
                    'confidence': conf,
                    'bbox': [x1, y1, x2, y2],
                    'center_uv': [u, v],
                    'depth': depth_value,  # Add depth information to the detection results
                    'world_coords': world_coords  # Add world coordinates to the detection results
                })

                # Draw bounding box, label, and world coordinates on the image
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Display the label with confidence
                label_text = f'{label} {conf:.2f}'
                cv2.putText(frame, label_text, (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (36, 255, 12), 2)

                # Display the world coordinates on the bounding box
                world_text = f'({world_coords["world_x"]}, {world_coords["world_y"]}, {world_coords["world_z"]})'
                cv2.putText(frame, world_text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 1)

        if detection_results:
            detection_info = json.dumps(detection_results)
        else:
            self.get_logger().info('No objects detected')
            detection_info = 'No objects detected'

        # Publish the processed image
        processed_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.processed_image_pub.publish(processed_msg)

        # Publish the detection data
        string_msg = String()
        string_msg.data = detection_info
        self.detection_data_pub.publish(string_msg)

def main():
    rclpy.init()
    node = Pros_pokemon_yolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

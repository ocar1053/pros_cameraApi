import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
import json

class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO('yolov8n.pt')  # load YOLO model

        # subscribe to camera image topics
        self.subscription1 = self.create_subscription(
            CompressedImage,
            '/real_sense/compressed',
            self.listener_callback_real_sense,
            10
        )

        self.subscription2 = self.create_subscription(
            CompressedImage,
            '/astra/compressed', 
            self.listener_callback_astra,
            10
        )

        # publish UV coordinates
        self.uv_pub = self.create_publisher(String, '/yolo/uv_coordinates', 10)

        # check camera status
        self.timer = self.create_timer(5.0, self.check_cam# 重置狀態以便下次檢查era_status)  

        self.real_sense_active = False
        self.astra_active = False

    def listener_callback_real_sense(self, msg):
       
        self.real_sense_active = True
        
        self.process_image(msg, 'real_sense')

    def listener_callback_astra(self, msg):
     
        self.astra_active = True
        
        self.process_image(msg, 'astra')

    def process_image(self, msg, camera_name):
       

       
        # Convert the ROS image message to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        # Perform YOLO object detection
        results = self.model(frame)

        uv_results = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                # Calculate center coordinates (u, v)
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                uv_results.append({
                    'class_id': cls,
                    'label': label,
                    'center_uv': [u, v],
                    'camera_name': camera_name,  # Add camera ID to the results
                })

        uv_info = json.dumps(uv_results)
        string_msg = String()
        string_msg.data = uv_info
        self.uv_pub.publish(string_msg)  # Publish UV coordinates

    def check_camera_status(self):
      
        if not self.real_sense_active:
            self.get_logger().warning('real_sense is not active or not connected!')
        else:
            self.real_sense_active = False 

     
        if not self.astra_active:
            self.get_logger().warning('astra is not active or not connected!')
        else:
            self.astra_active = False  

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

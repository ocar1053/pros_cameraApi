import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import json

class pros_yolo(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.get_logger().info('Node is running')
        self.bridge = CvBridge()


        self.subscription = self.create_subscription(
                CompressedImage,
                '/out/compressed',
                self.listener_callback,
                10
        )
                
        self.__processed_image_pub = self.create_publisher(
                CompressedImage,
                '/processed_image/compressed',
                10
        )
        
        self.__processed_binaryimage_pub = self.create_publisher(
                CompressedImage,
                '/processed_binaryimage_pub',
                10
        )
        
        self.__arucoData_pub = self.create_publisher(
                String,
                'aruco_detection',
                10
        )

        self.__threshold = 100

        

    def listener_callback(self, msg):
        self.get_logger().info('Received frame')
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, binary_image = cv2.threshold(gray, self.__threshold, 255, cv2.THRESH_BINARY)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
                
        parameters = cv2.aruco.DetectorParameters_create()        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(binary_image, aruco_dict, parameters=parameters)

        detection_results = []

        if ids is not None:
            self.get_logger().info(f'Detected markers: {ids.flatten()}')            
            
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                corner = corners[i][0].tolist()

                detection_results.append({
                    'id' : marker_id,
                    'corners' : corner
                })
            
            detection_info = json.dumps(detection_results)

            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            self.get_logger().info('No markers detected')
            detection_info = 'No markers detecteted'
        
        
        processed_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.__processed_image_pub.publish(processed_msg)

        processed_msg = self.bridge.cv2_to_compressed_imgmsg(binary_image)
        self.__processed_binaryimage_pub.publish(processed_msg)

        string_msg = String()
        string_msg.data  = detection_info
        self.__arucoData_pub.publish(string_msg)

                
        
def main():
    rclpy.init()
    node = pros_yolo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
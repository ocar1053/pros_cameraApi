import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')


        # create parameter camera_name
        self.declare_parameter('camera_name', 'real_sense')
        
        # load camera_name parameter
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.get_logger().info(f'Using camera: {self.camera_name}')
       
        self.subscription = self.create_subscription(
            Image,
            f'/{self.camera_name}/color/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.counter = 0  # Initialize counter



        self.save_folder = f'/workspaces/src/camera/images_folder/{self.camera_name}'
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)
        
    def listener_callback(self, msg):
        self.get_logger().info('Receiving image')
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Save image to file with a counter
        image_filename = os.path.join(self.save_folder, f'image_{self.counter:04d}.jpg')
        cv2.imwrite(image_filename, current_frame)
        self.get_logger().info(f'Image saved to {image_filename}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from .convert_camera_coordinates import convert_camera_coordinates
import pickle
import numpy as np

class ArucoDepthSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_depth_subscriber')
         # create parameter camera_name
        self.declare_parameter('camera_name', 'real_sense')
        
        # load camera_name parameter
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.get_logger().info(f'Using camera: {self.camera_name}')

        self.subscription = self.create_subscription(
            String,
            'f'/{self.camera_name}/aruco_detection',
            self.listener_callback,
            10)
        self.subscription 

        self.position_pub = self.create_publisher(
            String,
            f'/{self.camera_name}/position',
            10
        )

        with open(f"/workspaces/src/camera/resource/{self.camera_name}.pkl", "rb") as infile:
            camera = pickle.load(infile)
            camera_matrix = camera["camera_matrix"]
            extrinsic_matrix = camera["extrinsic_matrix"]
            rotation_matrix = camera["rotation_matrix"]
        
        camera_transfrom = extrinsic_matrix[:,-1][:-1].reshape(1, -1)
        self.convert_camera = convert_camera_coordinates(camera_matrix, extrinsic_matrix, rotation_matrix, camera_transfrom)




    def listener_callback(self, msg):
        try:            
            detection_results = json.loads(msg.data)
            print(detection_results)
            all_markers = []
            for marker in detection_results:
                marker_id = marker['id']
                corners = marker['corners']
                depths = marker['depths']

                self.get_logger().info(f'Marker ID: {marker_id}')                
                marker_points = []
                for i in range(len(corners)):
                    x = corners[i][0]
                    y = corners[i][1]
                    depth = depths[i]

                    world_position = self.convert_camera.screen_point_to_world_point(np.array([[x, y, 1]]) , depth/1000)                    
                    point_data = {
                        'pixel_x' : x,
                        'pixel_y' : y,
                        'world_x' : round(float(world_position[0]), 3),
                        'world_y' : round(float(world_position[1]), 3),
                        'world_z' : round(float(world_position[2]), 3)
                    }         
                    
                    marker_points.append(point_data)

                marker_data  = {
                    'marker_id' : marker_id,
                    'points' : marker_points
                }
                all_markers.append(marker_data)
            
            json_data = json.dumps(all_markers)

            json_msg = String()
            json_msg.data = json_data
            print(json_data)
            self.position_pub.publish(json_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON data: {e}')
        except KeyError as e:
            self.get_logger().error(f'Missing key in data: {e}')
        except Exception as e:
            self.get_logger().error(f'An error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDepthSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

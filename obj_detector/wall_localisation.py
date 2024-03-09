import rclpy
from rclpy.node import Node

from cdf_msgs.msg import Obstacles
from sensor_msgs.msg import LaserScan

import obj_detector.trigo as trigo 

import numpy as np

class WallLocalisation(Node):
    def __init__(self):
        super().__init__('wall_localisation')
        self.subscription = self.create_subscription(
            Obstacles,
            'obstacle',
            self.wall_localisation_callback,
            1)
        self.subscription_laser = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            1)
        
        self.subscription
        self.subscription_laser

    def wall_localisation_callback(self, msg : Obstacles):
        liste_segment = []
        for segment in msg.segments:
            segment_temp = [segment.index_first_point,segment.index_last_point]
            if segment_temp[1]- segment_temp[0] > 20:
                liste_segment.append(segment_temp)
        self.get_logger().info(f'segments : {liste_segment}')

def main(args=None):
    rclpy.init(args=args)
    wall_localisation = WallLocalisation()
    rclpy.spin(wall_localisation)
    wall_localisation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
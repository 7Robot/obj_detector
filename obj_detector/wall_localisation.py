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
            if trigo.distance(segment_temp[1],segment_temp[0],self.message_Lidar) > 0.2:
                liste_segment.append([segment_temp[0],segment_temp[1]])
        self.get_logger().info(f'segments : {liste_segment}')

        for k in list(range(len(liste_segment)-1)) :
            nb_point_1 = liste_segment[k][1]-liste_segment[k][0]
            theta_1 = nb_point_1*self.message_Lidar.angle_increment
            theta_bis_1 = trigo.calc_theta_bis(liste_segment[k],theta_1,self.message_Lidar)
            theta_ref_1 = theta_bis_1 + self.message_Lidar.angle_increment*liste_segment[k][0]
            nb_point_2 = liste_segment[k+1][1]-liste_segment[k+1][0]
            theta_2 = nb_point_2*self.message_Lidar.angle_increment
            theta_bis_2 = trigo.calc_theta_bis(liste_segment[k+1],theta_2,self.message_Lidar)
            theta_ref_2 = theta_bis_2 + self.message_Lidar.angle_increment*liste_segment[k+1][0]
            if np.abs(theta_ref_2 - theta_ref_1) > 0.7 :
                segment_perpendiculaire = [liste_segment[k],liste_segment[k+1]]
                rref1 = trigo.calc_rref(segment_perpendiculaire[0][0],theta_bis_1,self.message_Lidar)
                rref2 = trigo.calc_rref(segment_perpendiculaire[1][0],theta_bis_2,self.message_Lidar)
                self.get_logger().info(f'segment perpendiculaire : {segment_perpendiculaire}')
                self.get_logger().info(f'rref1 : {rref1} and rref2 : {rref2}')

    def laser_callback(self, msg : LaserScan):
        self.message_Lidar = msg

def main(args=None):
    rclpy.init(args=args)
    wall_localisation = WallLocalisation()
    rclpy.spin(wall_localisation)
    wall_localisation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
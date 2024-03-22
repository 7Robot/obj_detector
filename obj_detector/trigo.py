import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
import obj_detector.node_cluster as source

def distance(i, j, msg : LaserScan):
        return np.sqrt(float(msg.ranges[i])**2 + float(msg.ranges[j])**2 - 2*float(msg.ranges[i])*float(msg.ranges[j])*np.cos(np.abs((j-i))*float(msg.angle_increment)))

def calc_m1(segment,msg:LaserScan):
    return distance(segment[0],segment[1],msg)

def calc_alpha(segment,msg:LaserScan):
    delta_theta = msg.angle_increment
    nb_point = segment[1]-segment[0]

    theta = delta_theta*nb_point
    r0 = segment[0]
    r1 = segment[1]

    m1 = calc_m1(segment,msg)
    
    alpha = np.arccos((r0-2*r1*np.cos(theta))/m1)
    return alpha

def calc_theta_bis(segment,theta,msg:LaserScan):
    return np.arccos(cos_alpha(segment,msg)) - theta - np.pi/2

def calc_rref(i,theta_bis,msg:LaserScan):
     return float(msg.ranges[i])*np.cos(theta_bis)

def cos_alpha(segment,msg:LaserScan):
    delta_theta = msg.angle_increment
    nb_point = segment[1]-segment[0]
    theta = delta_theta*nb_point
    r0 = float(msg.ranges[segment[0]])
    r1 = float(msg.ranges[segment[1]])
    m1 = calc_m1(segment,msg)
    cosalpha = ((m1**2)+(r0**2)-(r1**2))/(2*m1*r0)
    return cosalpha
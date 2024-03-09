import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import obj_detector.affichage as affichage
import obj_detector.trigo as trigo 
from geometry_msgs.msg import Point
from cdf_msgs.msg import Obstacles, CircleObstacle, SegmentObstacle

bon_point = list(range(170, 450)) + list(range(770, 1055)) + list(range(1380, 1660))  #143, 489, 735, 1091, 1333, 1693 #170, 480, 770, 1055, 1350, 1660

class node_cluster(Node):
    def __init__(self):
        super().__init__('node_cluster')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.obstacle_detect_callback,
            1)
        
        self.publisher_ = self.create_publisher(
            MarkerArray, 
            'plante', 
            10)
        
        self.publisher_obstacle = self.create_publisher(
            Obstacles,
            'obstacle',
            10)

        self.subscription  

    def coordonnee_point(self, i, msg : LaserScan):
        theta_min = msg.angle_min
        delta_theta = msg.angle_increment
        return [np.cos(i*delta_theta+theta_min)*float(msg.ranges[i]), np.sin(i*delta_theta+theta_min)*float(msg.ranges[i])]

    def coordonnee_cercle(self, segment, msg:LaserScan):
        theta_min = msg.angle_min
        delta_theta = msg.angle_increment

        point_milieu = int((segment[1]-segment[0])/2)+segment[0]
        longueur = self.distance(segment[0], segment[1], msg)
        dist = msg.ranges[point_milieu]+np.sqrt(3)*longueur/2
        return [np.cos(point_milieu*delta_theta+theta_min)*dist, np.sin(point_milieu*delta_theta+theta_min)*dist]

    
    def distance(self, i, j, msg : LaserScan):
        return np.sqrt(float(msg.ranges[i])**2 + float(msg.ranges[j])**2 - 2*float(msg.ranges[i])*float(msg.ranges[j])*np.cos(np.abs((j-i))*float(msg.angle_increment)))
    
    def sortie_obstacle(self, liste_obstacle, msg : LaserScan):
        if len(liste_obstacle) == 0:
            return None
        else :
            obstacle = [liste_obstacle[0], liste_obstacle[-1]]
            return obstacle
        
    def traitement_segment(self, segment, msg:LaserScan):
        nb_point = segment[1]-segment[0]
        delta_theta = msg.angle_increment

        seuil_angle = 0.05
    
        if nb_point < 10 :
            return [segment]
        
        else :
            liste_segment = []
            point_milieu = segment[0]+10

            for k in range(int(nb_point/10)-1):
                cos_alpha_1 = trigo.cos_alpha([segment[0],point_milieu],msg)
                cos_alpha_2 = trigo.cos_alpha([segment[0],point_milieu+10],msg)
                if np.abs(cos_alpha_1 - cos_alpha_2) < seuil_angle :
                    point_milieu = point_milieu + 10

                else :
                    liste_segment.append([segment[0],point_milieu-10])
                    segment[0] = point_milieu
                    point_milieu = point_milieu+10
            
            cos_alpha_1 = trigo.cos_alpha([segment[0],point_milieu],msg)
            cos_alpha_2 = trigo.cos_alpha([segment[0],segment[1]],msg)
            if np.abs(cos_alpha_1 - cos_alpha_2) < seuil_angle :
                liste_segment.append([segment[0],segment[1]])
            else :
                liste_segment.append([segment[0],point_milieu-10])
                liste_segment.append([point_milieu,segment[1]])
            return liste_segment

    def obstacle_detect_callback(self, msg: LaserScan):
        number_of_points = len(msg.ranges)
        theta_min = msg.angle_min
        delta_theta = msg.angle_increment

        liste_obstacles = []
        points_obstacles = []

        coordonnee_obstacle = []
        coordonnee_plante = []
        radius_plante = []
        coordonnee_segment = [] 
        taille_obstacle = []
        coordonnee_mesure = []
        coordonnee_mesure_debug = []
        liste_segment = []
        
        for k in bon_point:
            if str(msg.ranges[k]) == 'inf' or str(msg.ranges[k+1]) == 'inf' :
                liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))
                points_obstacles = []

            elif k+1 not in bon_point:
                liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))
                points_obstacles = []

            elif np.abs(float(msg.ranges[k]) - float(msg.ranges[k+1])) < 0.05:
                points_obstacles.append(k)

            else : 
                coordonnee_mesure.append(self.coordonnee_point(k, msg))
                liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))
                points_obstacles = []
        
        liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))

        new_liste_obstacles = [x for x in liste_obstacles if x != None]
        liste_obstacles = new_liste_obstacles

        for i in range(len(liste_obstacles)):
            taille_obstacle = self.distance(liste_obstacles[i][0], liste_obstacles[i][1], msg)
            if taille_obstacle < 0.05:
                coordonnee_plante.append(self.coordonnee_cercle(liste_obstacles[i], msg))
                radius_plante.append(taille_obstacle*np.sqrt(3)/3)

            else :
                segment_temp = self.traitement_segment(liste_obstacles[i],msg)
                for k in range(len(segment_temp)):
                    coordonnee_segment.append([self.coordonnee_point(segment_temp[k][0], msg), 
                                              self.coordonnee_point(segment_temp[k][1], msg)])
                    liste_segment.append(segment_temp[k])
                
        marker_array_circle = affichage.affichage_plante(coordonnee_plante, radius_plante)
        self.publisher_.publish(marker_array_circle)

        marker_array_segment = affichage.affichage_segment(coordonnee_segment)
        self.publisher_.publish(marker_array_segment)

        #marker_array_mesure = affichage.affichage_point(coordonnee_mesure)
        #self.publisher_.publish(marker_array_mesure)

        #marker_array_mesure = affichage.affichage_point_debug(coordonnee_mesure_debug)
        marker_array_mesure = affichage.affichage_point_debug([[0.,0.]])
        self.publisher_.publish(marker_array_mesure)        

        obstacle = Obstacles()
        for i in range(len(coordonnee_plante)):
            circle = CircleObstacle()
            circle.center.x = coordonnee_plante[i][0]
            circle.center.y = coordonnee_plante[i][1]
            circle.radius = radius_plante[i]
            obstacle.circles.append(circle)

        for i in range(len(coordonnee_segment)):
            segment = SegmentObstacle()
            segment.first_point.x = coordonnee_segment[i][0][0]
            segment.first_point.y = coordonnee_segment[i][0][1]
            segment.last_point.x = coordonnee_segment[i][1][0]
            segment.last_point.y = coordonnee_segment[i][1][1]
            segment.index_first_point = liste_segment[i][0]
            segment.index_last_point = liste_segment[i][1]
            obstacle.segments.append(segment)
        self.publisher_obstacle.publish(obstacle)

def main():
    print('Hi from plante_detector.')
    rclpy.init(args=None)
    node = node_cluster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
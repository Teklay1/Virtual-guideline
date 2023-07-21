#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray
import numpy as np
import math
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from std_msgs.msg import Header

ofset = 2.0
class SubscribePointCloud(object):
    def __init__(self):
        rospy.init_node('subscribe_custom_point_cloud')
        rospy.Subscriber('point_blue', PointCloud2, self.blue_callback)
        rospy.Subscriber('point_green', PointCloud2, self.green_callback)
        rospy.spin()

    def blue_callback(self, point_cloud2):
            topic = 'visualization_marker_array'
            publisher = rospy.Publisher("b_line", Marker)
              
            #rospy.init_node('register')
            rospy.init_node('subscribe_custom_point_cloud')
             
            #marker = MarkerArray()
            marker = Marker()
              
            marker.header.frame_id = "camera_link"
            marker.points = []
            list = []
            m = 0
            h = 0
            C = []
            D = [] 
            n = 0
            q = 0
            r = 0
            d = 0
            c = 0
            start = True
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            for point  in pc2.read_points(point_cloud2):
              
              if point[2] <= 0.05 :
                pass           
              
              elif start == True :
                 
                 n = -point[0]
                 b = point[2]
                 start = False
                 pass
                 
              elif point[2] - b < -0.2 :
                pass
              
                 
              else :
                a = [point[2]-ofset,-point[0],-point[1]]
                list.append(a)
                n = -point[0]
                pass
                
            sortsecond = lambda val: val[1]
            list.sort(key=sortsecond)
            m = len(list)    
            h = m-1
            
            for i in range(m) :
                
                c = list[i][0]
                d = list[i][1]
                C.append(c)
                D.append(d) 
            
            
            Cm = np.average(C)
            Dm = np.average(D)
            sxx = np.sum((D-Dm)**2)
            vx2 = sxx/h             # sampling variance
            sxy2 = np.sum((D-Dm)*(C-Cm)) / h # covariance
            data1 = np.array([C, D]) 
            #c1 = np.cov(data1, bias=True)
            q =  sxy2/ vx2  # slop of the line ex. y=c + bx => q value of b           
            r = Cm - c*Dm  
        #regression line：y =', q, ' + ', r, '*x'  
            first_line_point = Point()
            first_line_point.x = q + r*list[h][1]#list[i][0]
            first_line_point.y = list[h][1]
            first_line_point.z = 0
            marker.points.append(first_line_point)
            first_line_marker = Point()
            first_line_marker.x = q + r*list[h][1]#list[i][0]
            first_line_marker.y = list[i][1]
            first_line_marker.z = 0
            marker.points.append(first_line_marker)
              
              
            publisher.publish(marker)
            
            
            
            
    def green_callback(self, point_cloud2):
          topic = 'visualization_marker_array'
          pub = rospy.Publisher("g_line", Marker)
            
          #rospy.init_node('register')
          rospy.init_node('subscribe_custom_point_cloud')
           
          #marker = MarkerArray()
          marker = Marker()
            
          marker.header.frame_id = "camera_link"
          marker.points = []
          A = []
          B = [] 
          list = []
          n = 0
          f = 0
          b = 0
          m = 0
          c = 0
          #list = []
          Green = True
          marker.type = marker.LINE_STRIP
          marker.action = marker.ADD
          marker.scale.x = 0.1
          marker.scale.y = 0.1
          marker.scale.z = 0.1
          marker.color.a = 1.0
          marker.color.r = 1.0
          marker.color.g = 1.0
          marker.color.b = 0.0
          marker.pose.orientation.x = 0.0
          marker.pose.orientation.y = 0.0
          marker.pose.orientation.z = 0.0
          marker.pose.orientation.w = 1.0
          for point  in pc2.read_points(point_cloud2):
            
            if point[2] <= 0.05 :
              pass           
            
            elif Green == True :
               
               m = -point[0]
               c = point[2]
               Green = False
               pass
               
            elif point[2] - c < -0.2 :
              pass
            
               
            else :
              d = [point[2]-ofset,-point[0],-point[1]]
              list.append(d)
              m = -point[0]
              pass
              
          sortsecond = lambda val: val[1]
          list.sort(key=sortsecond)
          n = len(list)    
          f = n-1
               
          for i in range(len(list)) :    
            a = list[i][0]
            b = list[i][1]
             
            A.append(a)
            B.append(b)
          data = np.array([A, B])  
          Am = np.average(A)
          Bm = np.average(B)
          sxx = np.sum((B-Bm)**2)
          vx = sxx/f               # sampling variance
          sxy = np.sum((B-Bm)*(A-Am)) / f # covariance
          #sxy = np.cov(data, bias=True)
          o = sxy / vx             
          p = Am - a*Bm  
          
          second_line_point = Point()
          second_line_point.x = p + o*list[f][1] #list[i][0]
          second_line_point.y = list[0][1] #list[i][1]
          second_line_point.z = 0 #list[i][2] 
          marker.points.append(second_line_point)
          second_line_marker = Point()
          second_line_marker.x = p + o*list[f][1] #all x points sort to 
          second_line_marker.y = list[f][1] # y points
          second_line_marker.z = 0
          marker.points.append(second_line_marker)
            
          pub.publish(marker)
            


def main():
    try:
        SubscribePointCloud()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

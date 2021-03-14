#! /usr/bin/env python3

# Ioannis Broumas&Galina Sidorenko
# ioabro17@student.hh.se
# Nov 2020
# Remember OpenCV is BGR

import os
import sys
import cv2 as cv
import numpy as np
from cv2 import aruco
import glob
import pickle
import datetime

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

# Check for camera calibration data
if not os.path.exists('/home/ubuntu/dev_ws/src/deis_py_dev/cameraCalibrations/sparkieCalibration.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('/home/ubuntu/dev_ws/src/deis_py_dev/cameraCalibrations/sparkieCalibration.pckl', 'rb')
    (cameraMatrix, distCoeffs) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)

measuredLength = 2.95 # Measured length of one side of the printed marker cm
id_to_follow=21
minimum_dist=15
goal_distance=20

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        self.publisher_f = self.create_publisher(String, 'cam_follower', 10)
        self.publisher_d= self.create_publisher(String, '/distance', 10)
        self.get_logger().info("It's rabbit season!")

def main(args=None):
    rclpy.init(args=args)

    f = Follower()
    msg = String()
    msg_d = String()
    msg.data = '0 0\n'

    base_speed = 90

    # Read the video stream
    cap = cv.VideoCapture(0)

    HorizontalPixels = cap.get(3)
    # print(cap.get(3)) # Width - Horizontal
    VerticalPixels = cap.get(4)
    # print(cap.get(4)) # Height - Vertical
    # Resolution = HorizontalPixels * VerticalPixels
    # VerticalDistance = 10
    # cm_to_pixel = VerticalDistance / VerticalPixels
    flag=0
    K1=1
    K2=0
    K3=0 
    if not cap.isOpened:
        print('--(!)Error opening video capture')
        f.destroy_node()
        rclpy.shutdown()
    
    while True:

        # Capturing each frame of our video stream
        ret, frame = cap.read()

        if frame is None:
            print('--(!) No captured frame -- Break!')
            break

        # grayscale image
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
        if ids is not None:
            # Print corners and ids to the console
            #for i, corner in zip(ids, corners):
            #     print('ID: {}; Corners: {}'.format(i, corner))

            if id_to_follow in ids:
                idx = list(ids).index(id_to_follow)
                #print(idx)
                #print(ids[idx])
                #print(corners[idx])
                corner = corners[idx]
                # Get center
                cx = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                cy = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
                #print("C_X: %d" %cx)
                #print("C_Y: %d" %cy)
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corner, measuredLength, cameraMatrix, distCoeffs)
                distance = cv.norm(tvecs)
                time_str=datetime.datetime.now().strftime("%S.%f")
            	 #current_time = datetime.get_clock().now().to_msg()
            	
                frame = aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 0, 255))
                cam_center = HorizontalPixels / 2
                
                #msg_d.data=""+str(time_str)+":"+current_time+":"+str(distance)
                #msg_d.data=""+str(time_str)+":"+str(distance)
                msg_d.data=""+str(time_str)+":"+str(format(distance, '.4f'))
                f.publisher_d.publish(msg_d)  
                
                
                '''
                if distance < minimum_dist:
                    flag=0
                    msg.data = '0 0\n'
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                elif cx > cam_center:
                    flag=0
                    gain = int((cam_center - cx) / 10)
                    LM = base_speed + gain
                    RM = base_speed 
                    msg.data = '%d %d\n' %(LM, RM)
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                elif cx < cam_center:
                    flag=0
                    gain = int((cx - cam_center) / 10)
                    LM = base_speed 
                    RM = base_speed + gain
                    msg.data = '%d %d\n' %(LM, RM)
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                else:
                    flag=0
                    LM = base_speed
                    RM = base_speed 
                    msg.data = '%d %d\n' %(LM, RM)
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                '''
                speed=base_speed+K1*int(distance - goal_distance)
                if distance < minimum_dist:
                    flag=0
                    msg.data = '0 0\n'
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                elif cx > cam_center:
                    flag=0
                    gain = int((cam_center - cx) / 10)
                    LM = speed + gain
                    RM = speed 
                    msg.data = '%d %d\n' %(LM, RM)
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                elif cx < cam_center:
                    flag=0
                    gain = int((cx - cam_center) / 10)
                    LM = speed 
                    RM = speed + gain
                    msg.data = '%d %d\n' %(LM, RM)
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                else: #just move forward
                    flag=0
                    LM = speed
                    RM = speed 
                    msg.data = '%d %d\n' %(LM, RM)
                    f.publisher_f.publish(msg)
                    #f.get_logger().info(': "%s"' % msg.data)
                f.get_logger().info('%s: %s'  % (msg_d.data, msg.data))

        # No marker found
        else:
            if flag==0: #if marker has been lost, we sends command to stop only once
                 time_str=datetime.datetime.now().strftime("%S.%f")
                 msg.data = '0 0\n'
                 f.publisher_f.publish(msg)
                 flag=1
                 f.get_logger().info( '%s:1000:%s' %(time_str, msg.data))
            #f.get_logger().info('No marker found')

        #f.get_logger().info(': "%s"' % msg.data)
        
        frame_resized = cv.resize(frame, (480,360))    
        cv.imshow("Snapshot", frame_resized)
        
        #cv.imshow("Snapshot", frame)
            
        if cv.waitKey(10) == 27: # Wait for 10ms, if key == 27 (esc char) break
            break

    cv.destroyAllWindows()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    f.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

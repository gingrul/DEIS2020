import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import cv2
from subprocess import call
import sys
import os
from datetime import datetime
import time

class KeyPublisher(Node):

    def __init__(self):
        super().__init__('key_presses_sender')
        self.pubAction = self.create_publisher(String, 'key_presses_gr3', 10)
        self.windowHeight=500
        self.windowWidth=500
        self.current_screen_image = np.zeros((self.windowHeight, self.windowWidth, 3), dtype=np.uint8)
        self.basic=150

def main(args=None):
    print ("Initializing node")
    rclpy.init(args=args)

    myKeySender = KeyPublisher()

    print ("Start opencv window")
    cv2.imshow("commandSender_screen", myKeySender.current_screen_image)
    cv2.waitKey(100)

    print ("Entering main loop")

    while rclpy.ok():

        cv2.imshow("commandSender_screen",
            myKeySender.current_screen_image)
        key= cv2.waitKey(10) & 0xFF

        if(key== ord("q")):
            print ("Pressed q")
            print ("Quit")
            break
        elif(key== ord("f")):
            print ("Pressed f: Go forward")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",-2,-155;155" #set speed, from GPS, to robots with no platoon, all robots, speed should be 255 left wheel, 255 right wheel
            myKeySender.pubAction.publish(msg)
            print ("sent command: ", msg.data)

        elif(key== ord("b")):
            print ("Pressed b: Back")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string)  + ",-2,155;-155" #set speed, from GPS, to robots with no platoon, all robots, speed should be -255 left wheel, -255 right wheel
            myKeySender.pubAction.publish(msg)
            print ("sent command: ", msg.data)

        elif(key== ord("l")):
            print ("Pressed l: Turn left")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",-2,0;155" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
            myKeySender.pubAction.publish(msg)
            print ("sent command: ", msg.data)

        elif(key== ord("r")):
            print ("Pressed r: Turn right")

            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",-2,-155;0" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
            myKeySender.pubAction.publish(msg)
            print ("sent command: ", msg.data)

        elif(key== ord("s")):
            print ("Pressed s: Stop")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",-2,0;0" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            print ("sent command: ", msg.data)
            
        #else:
            #print ("Pressed .: GO STRAIGHT")
            #dt_string = datetime.now().strftime("%d/%m/%Y-%H:%M:%S.%f")
            #msg = String()
            #msg.data = ""+ str(dt_string) + ",j,-1,-1,-2;0,0,201" #set message parameters
            #myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            #time.sleep(2)

    cv2.destroyAllWindows()
    myKeySender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print ('-------------------------------------')
    print ('-            Research Step             -')
    print ('-          command sender           -')
    print ('-       Mar 2021, HH,               -')
    print ('-------------------------------------')

    main()


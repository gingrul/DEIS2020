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
        self.basic=90

def main(args=None):
    print ("Initializing node")
    rclpy.init(args=args)

    myKeySender = KeyPublisher()
    
    speed=myKeySender.basic
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
            msg.data = ""+ str(dt_string) + ",3,"+str(speed)+" "+str(speed)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 255 left wheel, 255 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))

        elif(key== ord("b")):
            print ("Pressed b: Back")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string)  + ",3,"+str(-speed)+" "+str(-speed) +"\n"#set speed, from GPS, to robots with no platoon, all robots, speed should be -255 left wheel, -255 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))

        elif(key== ord("l")):
            print ("Pressed l: Turn left")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",3,"+str(speed)+" "+str(0)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))

        elif(key== ord("r")):
            print ("Pressed r: Turn right")

            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",3,"+str(0)+" "+str(speed)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))

        elif(key== ord("s")):
            print ("Pressed s: Stop")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",3,0 0\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))
         
        elif(key== ord("a")): #accelerate
            print ("Pressed a: Accelerate 3 forward for 20%")
            if speed ==0:
               speed=10 
            else: 
                speed=int(1.1*speed)
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",3,"+str(speed)+" "+str(speed)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))
            
        elif(key== ord("d")): #decelerate
            print ("Pressed d: Decelerate 3 forward for 10%")
            speed=int(0.9*speed)
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",3,"+str(speed)+" "+str(speed)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))
            
        elif(key== ord("z")): #accelerate
            print ("Pressed z: Accelerate 1 forward for 10%")
            if speed ==0:
               speed=10 
            else: 
                speed=int(1.1*speed)
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",1,"+str(speed)+" "+str(speed)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))
            
        elif(key== ord("c")): #decelerate
            print ("Pressed c: Decelerate 1 forward for 10%")
            speed=int(0.9*speed)
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",1,"+str(speed)+" "+str(speed)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))
            
            
        elif(key== ord("m")): #decelerate
            print ("Pressed m: ALL moving forward")            
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",-2,"+str(speed)+" "+str(speed)+"\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))
           
        elif(key== ord("n")):
            print ("Pressed n: ALL Stop")
            dt_string = datetime.now().strftime("%S.%f")
            msg = String()
            msg.data = ""+ str(dt_string) + ",-2,0 0\n" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 0 right wheel
            myKeySender.pubAction.publish(msg)
            #print ("sent command: ", msg.data)
            myKeySender.get_logger().info('%s'  % (msg.data))
           
            
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
    print ('-          Galina                      -')
    print ('-       March 2021, HH                 -')
    print ('-------------------------------------')

    main()

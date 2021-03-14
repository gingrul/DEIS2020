import time
import serial
import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from std_msgs.msg import Int64 
from std_msgs.msg import String
import numpy as np # NumPy Python library
from datetime import datetime
#from .ActionPerform import ActionPerformer
from .Action_message_convert import MessageTransformer

#ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

class ActionSubscriber(Node):
  
  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('Robot_3')
     
    # Create subscriber(s)    
    # The callback function is called as soon as a message is received.
    # The maximum number of queued messages is 10.
    self.subscription_a = self.create_subscription(
      String,
      '/key_presses_gr3',
      self.keypress_received,
      10)
    self.subscription_a  # prevent unused variable warning
         
    # Create publisher(s)   
    self.publish_to_em = self.create_publisher(String, '/EM', 10)#will publish there only in the case of EM
    self.publish_to_feedback = self.create_publisher(String, '/platooning', 10)
    time_period=1.0 #seconds
    self.tmr=self.create_timer(time_period, self.publish_feedback)
    self.ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
    self.get_logger().info('Node Key_Subscriber initialized!')
    
    
   
  def keypress_received(self, msg):
    """
    Callback function.
    This function gets called as soon as the key-press is received."""
    m=MessageTransformer()
    object_string=msg.data
    INFO=m.splitter_actions(object_string)
    target_robot_id=INFO[1]
    speeds=INFO[2]
    

    if  (target_robot_id==str(1)) or (target_robot_id==str(-2)): #if it concerns our robot. Our robot_id=3, -2 means all robots

        print(speeds)
        #self.ser.write(speeds.encode('utf-8'))
        self.ser.write(speeds.encode())
    
        #time.sleep(1)
        #while True:
        #if ser.in_waiting > 0:
        #    line = ser.readline().decode('utf-8').rstrip()
        #    print("--------received data-------")
        #    print(line)
        
        
        if speeds==str("0 0\n"):   
            
            #print(INFO[0]+":"+"0")
            self.publish_feedback(INFO[0]+":"+"0")
            self.publish_EM(INFO[0]+":"+"0") 
        
 
 
   #publish feedback to EM   
  def publish_EM(self,my_info):
    dt_string = datetime.now().strftime("%S.%f")
    msg = String()
    msg.data = ""+ str(dt_string) +":"+ my_info 
    self.publish_to_em.publish(msg) # Publish stop to EM           
    
  #publish feedback to platooning   
  def publish_feedback(self,my_info=str('1')):
    
    dt_string = datetime.now().strftime("%S.%f")
    msg = String()
    msg.data = ""+ str(dt_string) +":"+ my_info 
    self.publish_to_feedback.publish(msg) # Publish 1 if no EM, otherwise publish 0 0(STOP!)

       
def main(args=None):
    
    
  # Initialize the rclpy library
    rclpy.init(args=args)
 
  # Create the node
    node  = ActionSubscriber()
    node.ser.flush() 
    rclpy.spin(node)
    node.destroy_node()
 
  # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
  main()

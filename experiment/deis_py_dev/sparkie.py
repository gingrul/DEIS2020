#! usr/bin/env/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Christo George
# christogeorge@live.in

import serial
import datetime
import time

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Sparkie(Node):

    def __init__(self):
        super().__init__('sparkie')
        #self.subscription = self.create_subscription(String, 'teleop', self.teleop_callback, 10 ) # Don't change this
        #self.subscription = self.create_subscription(String, 'GPS', self.gps_callback, 10 )
        self.subscription = self.create_subscription(String, '/EM', self.em_callback, 10 )
        #self.subscription = self.create_subscription(String, '/platooning', self.platooning_callback, 10 )
        self.subscription = self.create_subscription(String, 'cam_follower', self.cam_callback, 10 )
        #self.subscription = self.create_subscription(String, 'cmd', self.cmd_callback, 10 )
        self.publisher_odom = self.create_publisher(String, 'odom_raw', 50)
        #self.publisher_imu = self.create_publisher(String, 'imu_r', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.get_sensorData_callback)
        self.ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
        #self.sensor_readings_file = open("data/sensor-readings.csv", "w")
        self.get_logger().info('Node Sparkie initialized!')

    def teleop_callback(self, msg):
        self.ser.write(msg.data.encode())
        
    def gps_callback(self, msg):
         
        self.ser.write(msg.data.encode())
        
    def cam_callback(self, msg):
        current_time =  datetime.datetime.now().strftime("%S.%f")
        self.get_logger().info(current_time + ":"+ msg.data)
        #print(msg.data) 
        self.ser.write(msg.data.encode())
        
    
    def cmd_callback(self, msg):
        self.ser.write(msg.data.encode())
        
    def em_callback(self, msg): #we receive EM only to brake
        self.get_logger().info("EM: STOP!")
        start_time =time.time()
        rt=10 #time during which the follower stay still until it again will follow
        stop = '0 0\n'
        while (rt>0):                       
            self.ser.write(stop.encode()) 
            rt=1-time.time() +start_time  
                
        
    def platooning_callback(self, msg):
        if msg.data[-1] =="0":
            current_time =  datetime.datetime.now().strftime("%S.%f")
            stop = '0 0\n'
            self.ser.write(stop.encode()) 
            #print(current_time + ": received EM:STOP!") 
            self.get_logger().info(current_time + ": received EM:STOP!")  
        
    def get_sensorData_callback(self):
        #print("get_sensorData_callback", self.ser.in_waiting)
        if (self.ser.in_waiting > 0):
        
            
            try:
                data1=self.ser.readline().decode()
                current_time =  datetime.datetime.now().strftime("%S.%f")
                self.get_logger().info(current_time +":" +data1)  
                #print(current_time+" "+data1)
            except: # catch error and ignore it
                print('Can not read data')
               
            
            #print(data)
            # self.sensor_readings_file.write(data)
            #data = data1.split(sep="_")
            
            # Validate data
            #print("Received data")
            '''
            if len(data1) != 0:
                msg_enc = String()
                msg_enc.data = current_time + "_" + data1
                #msg_enc.data = current_time + "_" + data[0]
                #msg_enc.data = data[0]
                self.publisher_odom.publish(msg_enc)
                #msg_imu = String()
                # msg_imu.data = current_time + "_" + data[1]
                #msg_imu.data = data[1]
                #self.publisher_imu.publish(msg_imu)
            else:
                msg_enc = String()
                # msg_enc.data = current_time + "_" + data[1]
                #msg_enc.data = data[0]
                #self.publisher_odom.publish(msg_enc)
                self.get_logger().info('Missed Data!:'+str(len(data1))+': ' +data1)
                '''
                

def main(args=None):
    rclpy.init(args=args)
    s = Sparkie()
    s.ser.flush()
    rclpy.spin(s)
    #sparkie_node.sensor_readings_file.close()
    s.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()


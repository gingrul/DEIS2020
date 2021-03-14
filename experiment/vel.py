#! usr/bin/env/python3

import serial
import datetime
import time


class Sensors():

    def __init__(self):
        
        self.ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=0.5)
        #self.sensor_readings_file = open("data/sensor-readings.csv", "w")
        #self.get_logger().info('Sensors initialized!')
      
    def get_sensorData_callback(self):
        #print("get_sensorData_callback", self.ser.in_waiting)
        flag=0
        if (flag==0):
           if (self.ser.in_waiting !=0):
               print("I'm in waiting!")   
               try:
                   data=self.ser.readline().decode()
                   #print(data)
               except: # catch error and ignore it
                   print('uh oh')
                 
            
            # self.sensor_readings_file.write(data)
               data_split = data.split(sep="_")
            # current_time = self.get_clock().now().to_msg()
            # Validate data
               print("Received data: "+data)
               flag=1
               if len(data_split) == 5:
                
                   msg_1 = data[1]
                   msg_2 = data[2]
                #print(msg_1+' '+msg_2)
               else:                              
                   print('Not 5: Missed Data!'+ data)
         
                

def main(args=None):
    
    s = Sensors()
    s.ser.flush()
    i=1
    time.sleep(2)
    while (i<=5):
        print(i)
        i=i+1
        line = '50 50\n'
        s.ser.write(line.encode())
        
        s.ser.flush() 
        s.get_sensorData_callback()  
        s.ser.flush() 
        time.sleep(0.1) 
    s.ser.flush()
    
    time.sleep(0.2)
    line = '0 0\n'
    s.ser.write(line.encode()) 
    s.get_sensorData_callback()  
    time.sleep(0.2)
    s.get_sensorData_callback()  
    
if __name__ == '__main__':
    main()


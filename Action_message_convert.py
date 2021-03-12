##!/usr/bin/env python3

import re


class MessageTransformer(object):
    
    def splitter_actions(self,msg):
#         pattern = re.compile(r";|,")
#         delimiters = ",", " ", "[", "]"
        split_string = str(msg).split(',')
        action_time=split_string[0];
        robot_id=split_string[1];
        speeds=split_string[2];
        
        #print(robot_id)
        return action_time, robot_id, speeds
    
#     def splitter_robot(self,msg):
#         pattern = re.compile(r";|,|[|] ")
# #         delimiters = ",", " ", "[", "]"
#         split_string = msg.split(msg)
#         action_id=split_string[0];
#         robot_id=split_string[1];
#         return robot_id
    

def main():

     print("Starting..")  
     q=MessageTransformer()
     t=q.splitter_actions("46.663919,-2,255 255")
     print(t[0])
     print(t[1])
     print(t[2])

if __name__ == '__main__':
     main()

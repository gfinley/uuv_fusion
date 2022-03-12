#!/usr/bin/env python
import requests
import rospy
from nav_msgs.msg import Odometry
import time

#aws ARN for API gateway
url = 'https://o9rlqe9h21.execute-api.us-west-2.amazonaws.com/V1'

class goal:
    def __init__(self, x,y,z):
        self.x = x
        self.y = y
        self.z = z

goal = goal(20,20,-50)
command = [0,0,0,0,0,0,0]

#make the node to pull topics
def callback(data):
    reward = calclate_reward(goal,data)
    myobj = {
            'pose'      : data.pose.pose,
            'positionX' : data.pose.pose.position.x,
            'positionY' : data.pose.pose.position.y,
            'positionZ' : data.pose.pose.position.z,
            'reward'    : reward
            }
    x = requests.post(url, data = myobj)
    #command[0] = command[0] + int(x.text[0])
    print(x.text)
    print("the current reward is" + str(reward))
    
    #add the publishment of commands to fusion
    
    
    time.sleep(1)
            
def calclate_reward(goal, data):
    #simple calculation based on difference
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    current_z = data.pose.pose.position.y
    
    velocity_x = data.twist.twist.linear.x
    velocity_y = data.twist.twist.linear.y
    velocity_z = data.twist.twist.linear.z
    
    #reward from position
    reward_position = 100 - abs(goal.x - current_x) - abs(goal.y - current_y) - abs(goal.z - current_z)
    reward_speed = 50 - abs(0-velocity_x) - abs(0-velocity_y) - abs(0-velocity_z)
    
    reward_total = reward_position + reward_position
    
    return reward_total
    
    #MAKE REQUEST TO API GATEWAY


    
if __name__ == '__main__':
    #add the code for the RL learning
    
    rospy.init_node('aws_gateway')
    rate = rospy.Rate(4)
    rospy.Subscriber("fusion/pose_gt",Odometry, callback)
    
    while not rospy.is_shutdown():
        rate.sleep()
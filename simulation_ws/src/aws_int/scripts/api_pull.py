#!/usr/bin/env python
import requests
import rospy
from nav_msgs.msg import Odometry
import time

#aws ARN for API gateway
url = 'https://o9rlqe9h21.execute-api.us-west-2.amazonaws.com/V1'

#make the node to pull topics
def callback(data):
    myobj = {
            'positionX' : data.pose.pose.position.x,
            'positionY' : data.pose.pose.position.y,
            'positionZ' : data.pose.pose.position.z,
            }
    #MAKE REQUEST TO API GATEWAY
    x = requests.post(url, data = myobj)
    print(x.text)
    time.sleep(1)
    
#
# myobj = {'positionX': '22.3',
#        'positionY' : "-34"
#}

    
if __name__ == '__main__':
    rospy.init_node('aws_gateway')
    rate = rospy.Rate(4)
    rospy.Subscriber("fusion/pose_gt",Odometry, callback)
    
    while not rospy.is_shutdown():
        rate.sleep()
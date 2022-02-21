#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from multiprocessing import Value, Lock


#global variables
new_bow_port_thruster = Value('B',0)
new_bow_stbd_thruster = Value('B',0)
new_vert_port_thruster = Value('B',0)
new_vert_stbd_thruster = Value('B',0)
new_aft_port_thruster = Value('B',0)
new_aft_stbd_thruster = Value('B',0)
new_aft_vert_thruster = Value('B',0)

bow_port_thruster = Value('d',0.0)
bow_stbd_thruster = Value('d',0.0)
vert_port_thruster = Value('d',0.0)
vert_stbd_thruster = Value('d',0.0)
aft_port_thruster = Value('d',0.0)
aft_stbd_thruster = Value('d',0.0)
aft_vert_thruster = Value('d',0.0)

def bowPortThrusterCb(msg):
    #print('Received bow port msg')
    new_bow_port_thruster.value = 1
    bow_port_thruster.value = msg.data

def bowStbdThrusterCb(msg):
    #print('Received bow starboard msg')
    new_bow_stbd_thruster.value = 1
    bow_port_thruster.value = msg.data

def vertPortThrusterCb(msg):
    #print('Received vert port msg')
    new_vert_port_thruster.value = 1
    vert_port_thruster.value = msg.data

def vertStbdThrusterCb(msg):
    #print('Received vert stbd msg')
    new_vert_stbd_thruster.value = 1
    vert_stbd_thruster.value = msg.data

def aftPortThrusterCb(msg):
    #print('Received aft port msg')
    new_aft_port_thruster.value = 1
    aft_port_thruster.value = msg.data

def aftStbdThrusterCb(msg):
    #print('Received aft starboard msg')
    new_aft_stbd_thruster.value = 1
    aft_stbd_thruster.value = msg.data

def aftVertThrusterCb(msg):
    #print('Received aft vert msg')
    new_aft_vert_thruster.value = 1
    aft_vert_thruster.value = msg.data


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('ros_msg_converter_fusion')

    #rosparams
    rate = rospy.get_param("~rate",100.0)

    #listening for these messages
    bow_port_thruster_topic = rospy.get_param("/bow_port_thruster_topic", "/bow_port_thruster")
    bow_stbd_thruster_topic = rospy.get_param("/bow_stbd_thruster_topic", "/bow_stbd_thruster")
    vert_port_thruster_topic = rospy.get_param("/vert_port_thruster_topic","/vert_port_thruster")
    vert_stbd_thruster_topic = rospy.get_param("/vert_stbd_thruster_topic","/vert_stbd_thruster")
    aft_port_thruster_topic = rospy.get_param("/aft_port_thruster_topic", "/aft_port_thruster")
    aft_stbd_thruster_topic = rospy.get_param("/aft_stbd_thruster_topic", "/aft_stbd_thruster")
    aft_vert_thruster_topic = rospy.get_param("/aft_vert_thruster_topic", "/aft_vert_thruster")

    #publishing these messages
    bow_port_prop_topic = rospy.get_param("/bow_port_thruster_topic", "/fusion/thrusters/3/input")    
    bow_stbd_prop_topic = rospy.get_param("/bow_stbd_thruster_topic", "/fusion/thrusters/0/input")
    vert_port_prop_topic = rospy.get_param("/vert_port_thruster_topic", "/fusion/thrusters/4/input")
    vert_stbd_prop_topic = rospy.get_param("/vert_stbd_thruster_topic", "/fusion/thrusters/1/input")
    aft_port_prop_topic = rospy.get_param("/aft_port_thruster_topic", "/fusion/thrusters/5/input")    
    aft_stbd_prop_topic = rospy.get_param("/aft_stbd_thruster_topic", "/fusion/thrusters/2/input")
    aft_vert_prop_topic = rospy.get_param("/aft_vert_thruster_topic", "/fusion/thrusters/6/input")

    # ROS Subscribers
    rospy.Subscriber(bow_port_thruster_topic, Float64, bowPortThrusterCb);  
    rospy.Subscriber(bow_stbd_thruster_topic, Float64, bowStbdThrusterCb);
    rospy.Subscriber(vert_port_thruster_topic, Float64, vertPortThrusterCb);
    rospy.Subscriber(vert_stbd_thruster_topic, Float64, vertStbdThrusterCb);
    rospy.Subscriber(aft_port_thruster_topic, Float64, aftPortThrusterCb);  
    rospy.Subscriber(aft_stbd_thruster_topic, Float64, aftStbdThrusterCb);
    rospy.Subscriber(aft_vert_thruster_topic, Float64, aftVertThrusterCb);

    # ROS publishers
    pub_bow_port_thruster = rospy.Publisher(bow_port_prop_topic, FloatStamped, queue_size=10, latch=True)
    pub_bow_stbd_thruster = rospy.Publisher(bow_stbd_prop_topic, FloatStamped, queue_size=10, latch=True)
    pub_vert_port_thruster = rospy.Publisher(vert_port_prop_topic, FloatStamped, queue_size=10, latch=True)
    pub_vert_stbd_thruster = rospy.Publisher(vert_stbd_prop_topic, FloatStamped, queue_size=10, latch=True)
    pub_aft_port_thruster = rospy.Publisher(aft_port_prop_topic, FloatStamped, queue_size=10, latch=True)
    pub_aft_stbd_thruster = rospy.Publisher(aft_stbd_prop_topic, FloatStamped, queue_size=10, latch=True)
    pub_aft_vert_thruster = rospy.Publisher(aft_vert_prop_topic, FloatStamped, queue_size=10, latch=True)


    r = rospy.Rate(rate)
    msg = FloatStamped()

    while not rospy.is_shutdown():
        if (new_bow_port_thruster.value):
            msg.data = bow_port_thruster.value
            pub_bow_port_thruster.publish(msg)
            #rospy.loginfo("pub converted bow_port_thruster %f", msg.data)
        if (new_bow_stbd_thruster.value):
            msg.data = bow_stbd_thruster.value
            pub_bow_stbd_thruster.publish(msg)
            #rospy.loginfo("pub converted bow_stbd_thruster %f", msg.data)
        if (new_vert_port_thruster.value):
            msg.data = vert_port_thruster.value
            pub_vert_port_thruster.publish(msg)
            #rospy.loginfo("pub converted vert_port_thruster %f", msg.data)  
        if (new_vert_stbd_thruster.value):
            msg.data = vert_stbd_thruster.value
            pub_vert_stbd_thruster.publish(msg)
            #rospy.loginfo("pub converted vert_stbd_thruster %f", msg.data)
        if (new_aft_port_thruster.value):
            msg.data = aft_port_thruster.value
            pub_aft_port_thruster.publish(msg)
            #rospy.loginfo("pub converted aft_port_thruster %f", msg.data)  
        if (new_aft_stbd_thruster.value):
            msg.data = aft_stbd_thruster.value
            pub_aft_stbd_thruster.publish(msg)
            #rospy.loginfo("pub converted aft_stbd_thruster %f", msg.data) 
        if (new_aft_vert_thruster.value):
            msg.data = aft_vert_thruster.value
            pub_aft_vert_thruster.publish(msg)
            #rospy.loginfo("pub converted aft_vert_thruster %f", msg.data)              
        r.sleep()


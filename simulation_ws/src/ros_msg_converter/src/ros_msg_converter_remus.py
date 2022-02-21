#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from multiprocessing import Value, Lock


#global variables
new_fwd_thruster = Value('B',0)
new_fwd_vert_xb = Value('B', 0)
new_fwd_horiz_xb = Value('B', 0)
new_aft_vert_xb = Value('B', 0)
new_aft_horiz_xb = Value('B', 0)
new_rudder_fin_msg = Value('B',0)
new_pitch_fin_msg = Value('B',0)

fwdPropSpeed = Value('d', 0.0)
fwdVertPropSpeed = Value('d', 0.0)
fwdHorizPropSpeed = Value('d', 0.0)
aftVertPropSpeed = Value('d', 0.0)
aftHorizPropSpeed = Value('d', 0.0)
rudderFinSetting = Value('d', 0.0)
pitchFinSetting = Value('d', 0.0)

def fwdThrusterCb(msg):
    #print('Received Forward Prop msg')
    new_fwd_thruster.value = 1
    fwdPropSpeed.value = msg.data

def xbFwdVertThrusterCb(msg):
    #print('Received xbForwardVerticalThruster msg')
    new_fwd_vert_xb.value = 1
    fwdVertPropSpeed.value = msg.data

def xbFwdHorizThrusterCb(msg):
    #print('Received xbForwardHorizThruster msg')
    new_fwd_horiz_xb = 1
    fwdHorizPropSpeed.value = msg.data

def xbAftVertThrusterCb(msg):
    #print('Received xbAftVerticalThruster msg')
    new_aft_vert_xb = 1
    aftVertPropSpeed.value = msg.data

def xbAftHorizThrusterCb(msg):
    #print('Received xbAftHorizThruster msg')
    new_aft_horiz_xb = 1
    aftHorizPropSpeed.value = msg.data

def rudderFinCb(msg):
    #print('Received rudder fin msg')
    new_rudder_fin_msg.value = 1
    topFinPosition.value = msg.data

def pitchFinCb(msg):
    #print('Received pitch fin msg')
    new_pitch_fin_msg.value = 1

if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('ros_msg_converter_remus')

    #rosparams
    rate = rospy.get_param("~rate",100.0)

    #listening for these messages
    fwd_thruster_topic = rospy.get_param("/fwd_thruster_topic", "/fwd_thruster")
    fwd_vert_thruster_topic = rospy.get_param("/fwd_vert_thruster_topic", "/fwd_vert_thruster")
    fwd_horiz_thruster_topic = rospy.get_param("/fwd_horiz_thruster_topic","/fwd_horiz_thruster")
    aft_vert_thruster_topic = rospy.get_param("/aft_vert_thruster_topic", "/aft_vert_thruster")
    aft_horiz_thruster_topic = rospy.get_param("/aft_horiz_thruster_topic","/aft_horiz_thruster")
    rudder_fin_topic = rospy.get_param("/rudder_fin_topic", "/rudder_fin")
    pitch_fin_topic = rospy.get_param("/pitch_fin_topic", "/pitch_fin")

    #publishing these messages
    fwd_prop_topic = rospy.get_param("/fwd_thruster_topic", "/remus/thrusters/0/input")
    xb_fwd_vert_topic = rospy.get_param("/xb_fwd_vert_thruster_topic", "/remus/thrusters/1/input")
    xb_fwd_horiz_topic = rospy.get_param("/xb_fwd_horiz_thruster_topic","/remus/thrusters/2/input")
    xb_aft_vert_topic = rospy.get_param("/xb_aft_vert_thruster_topic", "/remus/thrusters/3/input")
    xb_aft_horiz_topic = rospy.get_param("/xb_aft_horiz_thruster_topic","/remus/thrusters/4/input")
    top_rudder_fin_topic = rospy.get_param("/top_rudder_fin_topic", "/remus/fins/0/input")
    bottom_rudder_fin_topic = rospy.get_param("/bottom_rudder_fin_topic", "/remus/fins/2/input")
    port_pitch_fin_topic = rospy.get_param("/port_pitch_fin_topic", "/remus/fins/3/input")
    stbd_pitch_fin_topic = rospy.get_param("/stbd_pitch_fin_topic", "/remus/fins/1/input")

    # ROS Subscribers
    rospy.Subscriber(fwd_thruster_topic, Float64, fwdThrusterCb); 
    rospy.Subscriber(fwd_vert_thruster_topic, Float64, xbFwdVertThrusterCb);
    rospy.Subscriber(fwd_horiz_thruster_topic, Float64, xbFwdHorizThrusterCb);
    rospy.Subscriber(aft_vert_thruster_topic, Float64, xbAftVertThrusterCb);
    rospy.Subscriber(aft_horiz_thruster_topic, Float64, xbAftHorizThrusterCb);
    rospy.Subscriber(rudder_fin_topic, Float64, rudderFinCb);
    rospy.Subscriber(pitch_fin_topic, Float64, pitchFinCb);

    # ROS publishers
    pub_fwd_thruster = rospy.Publisher(fwd_prop_topic, FloatStamped, queue_size=10, latch=True)
    pub_fwd_vert = rospy.Publisher(xb_fwd_vert_topic, FloatStamped, queue_size=10, latch=True)
    pub_fwd_horiz = rospy.Publisher( xb_fwd_horiz_topic, FloatStamped, queue_size=10, latch=True)
    pub_aft_vert = rospy.Publisher(xb_aft_vert_topic, FloatStamped, queue_size=10, latch=True)
    pub_aft_horiz = rospy.Publisher(xb_aft_horiz_topic, FloatStamped, queue_size=10, latch=True)
    pub_top_rudder = rospy.Publisher(top_rudder_fin_topic, FloatStamped, queue_size=10, latch=True)
    pub_bottom_rudder = rospy.Publisher(bottom_rudder_fin_topic, FloatStamped, queue_size=10, latch=True)
    pub_port_pitch = rospy.Publisher(port_pitch_fin_topic, FloatStamped, queue_size=10, latch=True)
    pub_stbd_pitch = rospy.Publisher(stbd_pitch_fin_topic, FloatStamped, queue_size=10, latch=True)

    r = rospy.Rate(rate)
    msg = FloatStamped()

    while not rospy.is_shutdown():
        if (new_fwd_thruster.value):
            msg.data = fwdPropSpeed.value
            pub_fwd_thruster.publish(msg)
            #rospy.loginfo("pub converted fwd_thruster %f", msg.data)
        if (new_fwd_vert_xb.value):
            msg.data = fwdVertPropSpeed.value
            pub_fwd_vert.publish(msg)
            #rospy.loginfo("pub converted fwd_vert_xb %f", msg.data)
            new_fwd_vert_xb.value = 0
        if (new_fwd_horiz_xb.value):
            msg.data = fwdHorizPropSpeed.value
            pub_fwd_horiz.publish(msg)
            new_fwd_horiz_xb.value = 0
            #rospy.loginfo("pub converted fwd_horiz_xb %f", msg.data)
        if (new_aft_vert_xb.value):
            msg.data = aftVertPropSpeed.value
            pub_aft_vert.publish(msg)
            new_aft_vert_xb.value = 0
            #rospy.loginfo("pub converted aft_vert_xb %f", msg.data)
        if (new_aft_horiz_xb.value):
            new_aft_horiz_xb.value = 0
            msg.data = aftHorizPropSpeed.value
            pub_aft_horiz.publish(msg)
            new_aft_horiz_xb.value = 0            
            #rospy.loginfo("pub converted aft_horiz_xb %f", msg.data)
        if (new_rudder_fin_msg.value):
            new_rudder_fin_msg.value = 0
            msg.data = rudderFinSetting.value
            pub_top_rudder.publish(msg)
            pub_bottom_rudder.publish(msg)
            rospy.loginfo("pub converted rudder msg %f", msg.data)
        if (new_pitch_fin_msg.value):
            new_pitch_fin_msg.value = 0
            msg.data = pitchFinSetting.value
            pub_port_pitch.publish(msg)
            pub_stbd_pitch.publish(msg)
            rospy.loginfo("pub converted rudder msg %f", msg.data)

        r.sleep()
            


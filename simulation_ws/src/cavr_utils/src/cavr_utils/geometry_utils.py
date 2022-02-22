#!/usr/bin/env python
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import euler_matrix, quaternion_matrix
from math import pi, sqrt
from numpy import dot, array, append
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, PoseStamped

def euler_from_quat(qx, qy, qz, qw):
    yaw, pitch, roll = euler_from_quaternion((qx, qy, qz, qw),'rzyx')
    return roll, pitch, yaw

def quat_from_euler(roll, pitch, yaw):
    qx, qy, qz, qw = quaternion_from_euler(yaw, pitch, roll, 'rzyx')
    return qx, qy, qz, qw

def R_from_euler(roll, pitch, yaw):
    return euler_matrix(yaw,pitch,roll,'rzyx')
    
def euler_from_quat_q(q):
    roll, pitch, yaw = euler_from_quat(q.x, q.y, q.z, q.w)
    return roll, pitch, yaw

def euler_from_quat_deg(qx, qy, qz, qw):
    roll, pitch, yaw = euler_from_quat(qx, qy, qz, qw)
    return roll*180.0/pi, pitch*180.0/pi, yaw*180.0/pi

def quat_from_euler_deg(roll, pitch, yaw):    
    qx, qy, qz, qw = quat_from_euler(roll*pi/180.0, pitch*pi/180.0, yaw*pi/180.0)
    return qx, qy, qz, qw

def quat_q_from_euler(roll, pitch, yaw):
    q = Quaternion()    
    q.x, q.y, q.z, q.w = quat_from_euler(roll, pitch, yaw)
    return q

def angleInRangeMPItoPI(angle):
    if angle > pi:
        while angle > pi:
            angle = angle - 2*pi
        return angle
    elif angle <= -pi:
        while angle <= -pi:
            angle = angle + 2*pi
        return angle
    else:
        return angle

def angleInRange0to2PI(angle):
    if angle >= 2*pi:
        while angle >= 2*pi:
            angle = angle - 2*pi
        return angle
    elif angle < 0:
        while angle < 0:
            angle = angle + 2*pi
        return angle
    else:
        return angle

def zeroPitchRoll(qx, qy, qz, qw):
    r,p,y = euler_from_quat(qx, qy, qz, qw)
    return quat_from_euler(0.0, 0.0, y)

def zeroPitchRollPose(pose):
    r,p,y = euler_from_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    qx, qy, qz, qw = quat_from_euler(0.0, 0.0, y)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def transformStampedToPoseStamped(transform):
    ps = PoseStamped()
    ps.header = transform.header
    ps.pose.position.x = transform.transform.translation.x
    ps.pose.position.y = transform.transform.translation.y
    ps.pose.position.z = transform.transform.translation.z
    ps.pose.orientation = transform.transform.rotation
    return ps

def poseToStampedTransform(pose, stamp, from_frame, to_frame):
    tf = TransformStamped()
    tf.transform.translation.x = pose.position.x
    tf.transform.translation.y = pose.position.y
    tf.transform.translation.z = pose.position.z
    tf.transform.rotation.x = pose.orientation.x
    tf.transform.rotation.y = pose.orientation.y
    tf.transform.rotation.z = pose.orientation.z
    tf.transform.rotation.w = pose.orientation.w
    tf.header.stamp = stamp
    tf.header.frame_id = from_frame
    tf.child_frame_id = to_frame
    return tf

def stateToStampedTransform(state, from_frame, to_frame):
    tf = TransformStamped()
    tf.transform.translation.x = state.pose.pose.position.x
    tf.transform.translation.y = state.pose.pose.position.y
    tf.transform.translation.z = state.pose.pose.position.z
    tf.transform.rotation.x = state.pose.pose.orientation.x
    tf.transform.rotation.y = state.pose.pose.orientation.y
    tf.transform.rotation.z = state.pose.pose.orientation.z
    tf.transform.rotation.w = state.pose.pose.orientation.w
    tf.header.stamp = state.header.stamp
    tf.header.frame_id = from_frame
    tf.child_frame_id = to_frame
    return tf

def vector3ToStampedTransform(vector3, stamp, from_frame, to_frame):
    tf = TransformStamped()
    tf.transform.translation.x = vector3.x
    tf.transform.translation.y = vector3.y
    tf.transform.translation.z = vector3.z
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0
    tf.header.stamp = stamp
    tf.header.frame_id = from_frame
    tf.child_frame_id = to_frame
    return tf

def positionToStampedTransform(x, y, z, stamp, from_frame, to_frame):
    tf = TransformStamped()
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0
    tf.header.stamp = stamp
    tf.header.frame_id = from_frame
    tf.child_frame_id = to_frame
    return tf

def transRotToPose(trans, rot):
    p = Pose()
    p.position.x = trans[0]
    p.position.y = trans[1]
    p.position.z = trans[2]
    p.orientation.x = rot[0]
    p.orientation.y = rot[1]
    p.orientation.z = rot[2]
    p.orientation.w = rot[3]
    return p

# distance between geometry_msgs/Pose objects

def distanceBetweenPoses(pose1, pose2, dim=3):
    if dim==2:
        return sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)
    else:
        return sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)

# distance between python arrays (list or tuples)

def distanceBetweenPoints(p1, p2):
    sum = 0.0
    for i in range(len(p1)):
        sum = sum+(p1[i]-p2[i])**2
    return sqrt(sum)

def vectorMagnitude(v):
    sum = 0.0
    for i in range(len(v)):
        sum = sum+(v[i])**2
    return sqrt(sum)

# velocity magnitude from a geometry_msgs/Vector3
def velocityMagnitude(comps):
    return sqrt(comps.x**2 + comps.y**2 + comps.z**2)

# coordinate transforms: points
def transformPointB2W_RPY(pb, d, r, p, y):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and rotation (roll, pitch, yaw) from world to body    
    # pw = d + R*pb
    R = R_from_euler(r,p,y)
    return array(d)+R[0:3,0:3].dot(pb)
    
def transformPointW2B_RPY(pw, d, r, p, y):
    # transform a vector from world to body frame, given a translation 
    # (in the world frame) and rotation (roll, pitch, yaw) from world to body
    # pb = R'*pw - R'*d
    R = R_from_euler(r,p,y)
    return dot(R[0:3,0:3].transpose(),array(pw)-array(d))

def transformPointB2W_q(pb, d, q):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and quaternion (qx,qy,qz,qw) from world to body
    # pw = d + R*pb
    R = quaternion_matrix(q) # homogeneous matrix
    return array(d)+R[0:3,0:3].dot(pb)
    
def transformPointW2B_q(pw, d, q):
    # transform a vector from world to body frame, given a translation 
    # (in the world frame) and quaternion (qx,qy,qz,qw) from world to body
    # pb = R'*pw - R'*d
    R = quaternion_matrix(q) # homogeneous matrix
    return dot(R[0:3,0:3].transpose(),array(pw)-array(d))

def transformPointB2W_quat(pb, d, q):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and Quaternion from world to body
    # pw = d + R*pb
    R = quaternion_matrix((q.x,q.y,q.z,q.w)) # homogeneous matrix
    return array(d)+R[0:3,0:3].dot(pb)
    
def transformPointW2B_quat(pw, d, q):
    # transform a vector from world to body frame, given a translation 
    # (in the world frame) and Quaternion from world to body
    # pb = R'*pw - R'*d
    R = quaternion_matrix((q.x,q.y,q.z,q.w)) # homogeneous matrix
    return dot(R[0:3,0:3].transpose(),array(pw)-array(d))

# coordinate transforms: vectors
def transformVelB2W_RPY(vb, r, p, y):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and rotation (roll, pitch, yaw) from world to body    
    # vw = R*vb
    R = R_from_euler(r,p,y)
    return dot(R[0:3,0:3],array(vb))

def transformVelW2B_RPY(vw, r, p, y):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and rotation (roll, pitch, yaw) from world to body
    # vb = R'*vw
    R = R_from_euler(r,p,y)
    return dot(R[0:3,0:3].transpose(),array(vw))

def transformVelB2W_q(vb, q):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and quaternion (qx,qy,qz,qw) from world to body
    # vw = R*vb
    R = quaternion_matrix(q) # homogeneous matrix
    return dot(R[0:3,0:3],array(vb))

def transformVelW2B_q(vw, q):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and quaternion (qx,qy,qz,qw) from world to body
    # vb = R'*vw
    R = quaternion_matrix(q) # homogeneous matrix
    return dot(R[0:3,0:3].transpose(),array(vw))

def transformVelB2W_quat(vb, q):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and Quaternion from world to body
    # vw = R*vb
    R = quaternion_matrix((q.x,q.y,q.z,q.w)) # homogeneous matrix
    return dot(R[0:3,0:3],array(vb))

def transformVelW2B_quat(vw, q):
    # transform a vector from body to world frame, given a translation 
    # (in the world frame) and Quaternion from world to body
    # vb = R'*vw
    R = quaternion_matrix((q.x,q.y,q.z,q.w)) # homogeneous matrix
    return dot(R[0:3,0:3].transpose(),array(vw))
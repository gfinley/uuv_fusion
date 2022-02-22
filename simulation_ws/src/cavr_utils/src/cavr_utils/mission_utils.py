#!/usr/bin/env python
import struct
from geometry_msgs.msg import Pose, Point
#from numpy import uint8
from cavr_msgs.msg import MissionGoalGoal
import time

# mission goal
class MissionGoal:
    def __init__(self, name='goal', id=1, frame_id='local_map', pose=Pose(), rad=5.0, dur=0.0, type='waypoint', flag=MissionGoalGoal.EXEC_NONE, heave_mode='none', vel_mode='none', heave_cmd=2.0, head_mode='none', vel_cmd=0.0):
        self.name = name
        self.id = id
        self.frame_id = frame_id
        self.pos = [pose.position.x, pose.position.y, pose.position.z]
        self.ori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        self.rad = rad
        self.dur = dur
        self.type = type
        self.flag = flag
        self.heave_mode = heave_mode
        self.head_mode = head_mode
        self.updateFlagHeaveMode(self.heave_mode)
        self.updateFlagHeadMode(self.head_mode)      
        self.heave_cmd = heave_cmd
        self.vel_cmd = vel_cmd
        self.updateFlagVelMode(self.vel_cmd > 0.0001 or self.vel_cmd < -0.0001)
        
    def setDataPose(self, name='goal', id=1, frame_id='local_map', pose=Pose(), rad=5.0, dur=0.0, type='waypoint', flag=MissionGoalGoal.EXEC_NONE, heave_mode='none', heave_cmd=2.0, head_mode='none', vel_cmd=0.0):
        self.setData(name, id, frame_id, [pose.position.x, pose.position.y, pose.position.z], [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], rad, dur, type, flag, heave_mode, heave_cmd, head_mode, vel_cmd)

    def setData(self, name='goal', id=1, frame_id='local_map', pos=[0.0,0.0,0.0], ori=[0.0,0.0,0.0,1.0], rad=5.0, dur=0.0, type='waypoint', flag=MissionGoalGoal.EXEC_NONE, heave_mode='none', heave_cmd=2.0, head_mode='none', vel_cmd=0.0):
        self.name = name
        self.id = id
        self.frame_id = frame_id
        self.pos = pos
        self.ori = ori
        self.rad = rad
        self.dur = dur
        self.type = type
        self.flag = flag
        self.heave_mode = heave_mode
        self.updateFlagHeaveMode(self.heave_mode)
        self.updateFlagHeadMode(self.head_mode)
        self.heave_cmd = heave_cmd
        self.head_mode = head_mode
        self.vel_cmd = vel_cmd
        self.updateFlagVelMode(self.vel_cmd > 0.0001 or self.vel_cmd < -0.0001)
        
    def setDataDict(self, d):
        self.pos = d['pos']
        self.ori = d['ori']
        self.name = d['name']
        self.id = d['id']
        self.frame_id = d['frame_id']
        self.type = d['type']
        self.rad = d['rad']
        self.dur = d['dur']
        if 'flag' in d:
            self.flag = d['flag']
        else:
            self.flag = MissionGoalGoal.EXEC_NONE
        if 'heave_mode' in d:
            self.heave_mode = d['heave_mode']
        else:
            self.heave_mode = 'none'            
        self.updateFlagHeaveMode(self.heave_mode)
        if 'heave_cmd' in d:
            self.heave_cmd = d['heave_cmd']
        else:
            self.heave_cmd = 2.0
        if 'head_mode' in d:
            self.head_mode = d['head_mode']
        else:
            self.head_mode = 'none'
        self.updateFlagHeadMode(self.head_mode)            
        if 'vel_cmd' in d:
            self.vel_cmd = d['vel_cmd']
        else:
            self.vel_cmd = 0.0
        self.updateFlagVelMode(self.vel_cmd > 0.0001 or self.vel_cmd < -0.0001)
    
    def getDataPose(self):
        return self.name, self.id, self.frame_id, self.getPose(), self.rad, self.dur, self.getTypeEnum(), self.flag, self.heave_cmd, self.vel_cmd
        
    def setRadius(self, rad):
        self.rad = rad
        
    def getRadius(self):
        return self.rad

    # set type from enum
    def setTypeEnum(self, type):
        if type == MissionGoalGoal.WAYPOINT:
            self.type = 'waypoint'
        elif type == MissionGoalGoal.ORBIT:
            self.type = 'orbit'
        elif type == MissionGoalGoal.STATION_KEEP:
            self.type = 'station_keep'
        elif type == MissionGoalGoal.END_OF_MISSION:
            self.type = 'end_of_mission'
        elif type == MissionGoalGoal.TELE_OP:
            self.type = 'tele_op'
        elif type == MissionGoalGoal.DYNAMIC_POSITION:
            self.type = 'dynamic_position'
        elif type == MissionGoalGoal.HOVER:
            self.type = 'hover'
        elif type == MissionGoalGoal.HOVER_AND_TRANSLATE:
            self.type = 'hover_and_translate'
        elif type == MissionGoalGoal.HOME:
            self.type = 'home'
        else:
            self.type = 'undefined'

    # set type from string
    def setType(self, type):
        self.type = type

    def setFlag(self, flag):
        self.flag = flag

    def updateFlagHeaveMode(self, heave_mode):
        if heave_mode == 'depth':
            # unset the altitude flag
            if self.flag & MissionGoalGoal.EXEC_ALTCMD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_ALTCMD
            # set the depth flag
            if self.flag & MissionGoalGoal.EXEC_DEPTHCMD == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_DEPTHCMD
        elif heave_mode == 'altitude':
            # unset the depth flag
            if self.flag & MissionGoalGoal.EXEC_DEPTHCMD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_DEPTHCMD
            # set the altitude flag
            if self.flag & MissionGoalGoal.EXEC_ALTCMD == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_ALTCMD
        else:
            # unset the depth flag
            if self.flag & MissionGoalGoal.EXEC_DEPTHCMD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_DEPTHCMD
            # unset the altitude flag
            if self.flag & MissionGoalGoal.EXEC_ALTCMD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_ALTCMD

    def updateFlagHeadMode(self, head_mode):
        if head_mode == 'forward' or head_mode == 'none':
            # unset the left, right, reverse flags
            if self.flag & MissionGoalGoal.EXEC_PATH_LEFT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_LEFT
            if self.flag & MissionGoalGoal.EXEC_PATH_RIGHT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_RIGHT
            if self.flag & MissionGoalGoal.EXEC_PATH_REVERSE > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_REVERSE
            # set the forward flag
            if self.flag & MissionGoalGoal.EXEC_PATH_FORWARD == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_PATH_FORWARD
        elif head_mode == 'left':
            # unset the forward, right, reverse flags
            if self.flag & MissionGoalGoal.EXEC_PATH_FORWARD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_FORWARD
            if self.flag & MissionGoalGoal.EXEC_PATH_RIGHT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_RIGHT
            if self.flag & MissionGoalGoal.EXEC_PATH_REVERSE > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_REVERSE
            # set the left flag
            if self.flag & MissionGoalGoal.EXEC_PATH_LEFT == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_PATH_LEFT
        elif head_mode == 'right':
            # unset the forward, left, reverse flags
            if self.flag & MissionGoalGoal.EXEC_PATH_FORWARD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_FORWARD
            if self.flag & MissionGoalGoal.EXEC_PATH_LEFT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_LEFT
            if self.flag & MissionGoalGoal.EXEC_PATH_REVERSE > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_REVERSE
            # set the left flag
            if self.flag & MissionGoalGoal.EXEC_PATH_RIGHT == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_PATH_RIGHT
        elif head_mode == 'reverse':
            # unset the forward, right, reverse flags
            if self.flag & MissionGoalGoal.EXEC_PATH_FORWARD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_FORWARD
            if self.flag & MissionGoalGoal.EXEC_PATH_RIGHT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_RIGHT
            if self.flag & MissionGoalGoal.EXEC_PATH_LEFT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_LEFT
            # set the left flag
            if self.flag & MissionGoalGoal.EXEC_PATH_REVERSE == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_PATH_REVERSE
        else:
            rospy.logwarn("heading mode not recognized: %s. Executing in FORWARD mode")
            # unset the left, right, reverse flags
            if self.flag & MissionGoalGoal.EXEC_PATH_LEFT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_LEFT
            if self.flag & MissionGoalGoal.EXEC_PATH_RIGHT > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_RIGHT
            if self.flag & MissionGoalGoal.EXEC_PATH_REVERSE > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_PATH_REVERSE
            # set the forward flag
            if self.flag & MissionGoalGoal.EXEC_PATH_FORWARD == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_PATH_FORWARD
                
    def updateFlagVelMode(self, flag):
        if flag:
            # set the vel flag
            if self.flag & MissionGoalGoal.EXEC_VELCMD == 0:
                self.flag = self.flag | MissionGoalGoal.EXEC_VELCMD
        else:
            # unset the vel flag
            if self.flag & MissionGoalGoal.EXEC_VELCMD > 0:
                self.flag = self.flag ^ MissionGoalGoal.EXEC_VELCMD

    def setPose(self, pose):
        self.pos = [pose.position.x, pose.position.y, pose.position.z]
        self.ori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    def getPosition(self):
        point = Point()
        point.x = self.pos[0]
        point.y = self.pos[1]
        point.z = self.pos[2]
        return point

    def getPose(self):
        pose = Pose()
        pose.position.x = self.pos[0]
        pose.position.y = self.pos[1]
        pose.position.z = self.pos[2]
        pose.orientation.x = self.ori[0]        
        pose.orientation.y = self.ori[1]        
        pose.orientation.z = self.ori[2]        
        pose.orientation.w = self.ori[3]        
        return pose
    
    def getFrameId(self):
        return self.frame_id

    def getFlag(self):
        return self.flag

    def getType(self):
        return self.type

    def getTypeEnum(self):
        if self.type == 'waypoint':
            return MissionGoalGoal.WAYPOINT
        elif self.type == 'orbit':
            return MissionGoalGoal.ORBIT
        elif self.type == 'station_keep':
            return MissionGoalGoal.STATION_KEEP
        elif self.type == 'end_of_mission':
            return MissionGoalGoal.END_OF_MISSION
        elif self.type == 'tele_op':
            return MissionGoalGoal.TELE_OP
        elif self.type == 'dynamic_position':
            return MissionGoalGoal.DYNAMIC_POSITION
        elif self.type == 'hover':
            return MissionGoalGoal.HOVER
        elif self.type == 'home':
            return MissionGoalGoal.HOME
        elif self.type == 'hover_and_translate': 
            return MissionGoalGoal.HOVER_AND_TRANSLATE
        else:
            return MissionGoalGoal.UNDEFINED
    
    def setFrameId(self, frame_id):
        self.frame_id = frame_id
        
    def toDict(self):
        dict = {}
        pos = []
        for i in range(len(self.pos)):
            pos.append(float(self.pos[i]))
        dict['pos'] = pos
        ori = []
        for i in range(len(self.ori)):
            ori.append(float(self.ori[i]))
        dict['ori'] = ori
        dict['name'] = self.name
        dict['id'] = self.id
        dict['frame_id'] = self.frame_id
        dict['type'] = self.type
        dict['rad'] = self.rad
        dict['dur'] = self.dur
        dict['flag'] = self.flag
        dict['heave_mode'] = self.heave_mode
        dict['heave_cmd'] = self.heave_cmd
        dict['head_mode'] = self.head_mode
        dict['vel_cmd'] = self.vel_cmd
        return dict
        

class MissionControl:
    # Mission Control types
    ADD_GOAL = 1
    HOME = 2
    UNHOME = 3
    DELETE_GOAL = 4
    REORDER_GOALS = 5
    REQUEST_ADD = 6
    REQUEST_DELETE = 7
    READ_FROM_FILE = 8
    SET_HOME = 9
    CLEAR_MISSION = 10
    DUMP_TO_FILE = 11
    UPDATE_GOAL = 12
    REQUEST_UPDATE = 13
    SET_HOME_POSE = 14
    REQUEST_UPDATE_ALL = 15

def getMsgLen(req):
    if req == MissionControl.HOME:
        return 2
    elif req == MissionControl.UNHOME:
        return 2
    elif req == MissionControl.DELETE_GOAL:
        return 3
    elif req == MissionControl.UPDATE_GOAL:
        return 31
    elif req == MissionControl.REORDER_GOALS:
        return 0 # variable length...
    elif req == MissionControl.REQUEST_ADD:
        return 2
    elif req == MissionControl.REQUEST_DELETE:
        return 3
    elif req == MissionControl.REQUEST_UPDATE:
        return 31
    elif req == MissionControl.REQUEST_UPDATE_ALL:
        return 31
    elif req == MissionControl.SET_HOME:
        return 2
    elif req == MissionControl.SET_HOME_POSE:
        return 14
    elif req == MissionControl.CLEAR_MISSION:
        return 2
    elif req == MissionControl.READ_FROM_FILE:
        return 0 # variable length...
    elif req == MissionControl.DUMP_TO_FILE:
        return 0 # variable length...
    elif req == MissionControl.ADD_GOAL:
        return 31
    else:
        return None

# parse the header portion (not the data)
# expects the entire buffer
# returns a flag [0 = unsuccessful, 1 = successful], data, status message
def parseHeader(buf):
    header = None
    stat = ''
    if len(buf) > 1:
        header = struct.unpack('=BB', buf[:2])
    else:
        stat = 'Not enough data in buffer'
    return header, stat

# parse the data portion (not the header)
# expects the entire buffer
def parseData(header, buf):
    data = struct.unpack('=Bfffffff', buf[2:])
    data = None
    stat = ''
    print("Checking len of buf:")
    print(len(buf))
    if len(buf) >= header[1]:
        msgNum = header[0]
        rospy.loginfo("msgNum: %d", msgNum)
        if msgNum == MissionControl.ADD_GOAL:
            data = struct.unpack('=Bfffffff', buf[2:]) # id, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w
        elif msgNum == MissionControl.HOME:
            data = []
        elif msgNum == MissionControl.UNHOME:
            data = []
        elif msgNum == MissionControl.DELETE_GOAL:
            data = struct.unpack('=B', buf[2:]) # id
        elif msgNum == MissionControl.UPDATE_GOAL:
            data = struct.unpack('=Bfffffff', buf[2:]) # id, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w
        elif msgNum == MissionControl.REORDER_GOALS:
            fmt = ''
            for x in range(0,header[1]-2):
                fmt = fmt+'c'
            data = struct.unpack(fmt, buf[2:]) # msg
        elif msgNum == MissionControl.REQUEST_ADD:
            data = []
        elif msgNum == MissionControl.REQUEST_DELETE:
            data = struct.unpack('=B', buf[2:]) # id
        elif msgNum == MissionControl.REQUEST_UPDATE:
            data = struct.unpack('=Bfffffff', buf[2:]) # id, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w
        elif msgNum == MissionControl.SET_HOME:
            data = []
        elif msgNum == MissionControl.SET_HOME_POSE:
            data = struct.unpack('=fff',buf[2:])
        elif msgNum == MissionControl.CLEAR_MISSION:
            data = []
        elif msgNum == MissionControl.READ_FROM_FILE:
            fmt = ''
            for x in range(0,header[1]-2):
                fmt = fmt+'c'
            data = struct.unpack(fmt, buf[2:]) # msg
        elif msgNum == MissionControl.DUMP_TO_FILE:
            fmt = ''
            for x in range(0,header[1]-2):
                fmt = fmt+'c'
            data = struct.unpack(fmt, buf[2:]) # msg
            
        else:
            stat = 'Message type not recognized. MSgNum = {0}'.format(msgNum)
    else:
        stat = 'Not enough data in buffer'
    return data, stat

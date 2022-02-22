#!/usr/bin/env python
import struct

# define the bits for vehStatus message (uint32)
PAUSE = 2**0 # 1
STOP = 2**1 # 2
INTER_ACTIVE = 2**2 # 4
MANUAL_OVERRIDE = 2**3 # 8
MANUAL_OVERRIDE_POSSIBLE_RESET = 2**4 # 16
LOG = 2**5 #32
DYNPOS = 2**6 #64
ENABLE_USBL_DVL = 2**7 #128
EXEC_VELCMD = 2**8
EXEC_ALTCMD = 2**9
EXEC_DEPTHCMD = 2**10
AUTONOMOUS = 2**11
EXEC_PATH_FORWARD = 2**12
EXEC_PATH_REVERSE = 2**13
EXEC_PATH_LEFT = 2**14
EXEC_PATH_RIGHT = 2**15
END_OF_MISSION = 2**16
HOME = 2**17
EXEC_ALTCMD_FILE = 2**18
EXEC_DEPTHCMD_FILE = 2**19
EXEC_RUDCMD = 2**20
EXEC_THRCMD = 2**21

# Define system requests (uint8_t)
# Use 0-199 for system wide and 200-256 for vehicle specific 
GOAL_CONTINUE = 1
MISSION_RESET = 2
MISSION_LOAD_FILE = 3
MISSION_SAVE_FILE = 4
MISSION_STAR = 5
MISSION_LAWN_N = 6
MISSION_LAWN_S = 7
MISSION_LAWN_E = 8
MISSION_LAWN_W = 9
MISSION_VEH = 10
MISSION_BOX = 11
MAP_SET_VEH_POS = 12

# Vehicle specific
# VLBV
GSS_CAL_DEPTH = 200
GSS_SET_VEH_POS = 201
GSS_SET_MAG_DECL = 202

# REMUS
# SEAFOX
# SCANEAGLE

# define controllers to be used (uint8)
CONTROLLER_NONE = 0
CONTROLLER_TRAVERSE = 1
CONTROLLER_TELE_OP = 2
CONTROLLER_DYN_POS = 3
CONTROLLER_STATION_KEEP = 4
CONTROLLER_HOVER = 5
CONTROLLER_HOVER_AND_TRANSLATE = 6

def getMsgLen(req):
    if req == GOAL_CONTINUE:
	return 2
    elif req == MISSION_RESET:
        return 2
    elif req == MISSION_LOAD_FILE:
        return 128
    elif req == MISSION_SAVE_FILE:
        return 128
    elif req == MISSION_STAR:
        return 7 # [msg, len, # rays, len of rays]
    elif req == MISSION_BOX:
        return 6 # [msg, len, len of legs]
    elif req == MISSION_LAWN_N:
        return 11 # [msg, len, # legs, leg len, leg spacing]
    elif req == MISSION_LAWN_E:
        return 11 # [msg, len, # legs, leg len, leg spacing]
    elif req == MISSION_LAWN_S:
        return 11 # [msg, len, # legs, leg len, leg spacing]
    elif req == MISSION_LAWN_W:
        return 11 # [msg, len, # legs, leg len, leg spacing]
    elif req == MISSION_VEH:
        return 2 # [msg, len]
    elif req == MAP_SET_VEH_POS:
        return 2 # [msg, len]
    elif req == GSS_CAL_DEPTH:
        return 2 # [msg, len]
    elif req == GSS_SET_VEH_POS:
        return 2 # [msg, len]
    elif req == GSS_SET_MAG_DECL:
        return 2 # [msg, len]
    elif req == GSS_UPDATE_INS:
        return 2 # [msg, len]
    else:
        return 0 # variable length...

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
    data = []
    stat = ''
    if len(buf) >= header[1]:
        msgNum = header[0]
        if msgNum == GOAL_CONTINUE:
            data = []
        elif msgNum == MISSION_RESET:
            data = []
        elif msgNum == MISSION_SAVE_FILE:
            fmt = ''
            for x in range(0,header[1]-2):
                fmt = fmt+'c'
            data = struct.unpack(fmt, buf[2:]) # msg
        elif msgNum == MISSION_LOAD_FILE:
            fmt = ''
            for x in range(0,header[1]-2):
                fmt = fmt+'c'
            data = struct.unpack(fmt, buf[2:]) # msg
        elif msgNum == MISSION_STAR:
            data = struct.unpack('=Bf', buf[2:])
        elif msgNum == MISSION_BOX:
            data = struct.unpack('=f', buf[2:])
        elif msgNum == MISSION_LAWN_N:
            data = struct.unpack('=Bff', buf[2:])
        elif msgNum == MISSION_LAWN_E:
            data = struct.unpack('=Bff', buf[2:])
        elif msgNum == MISSION_LAWN_S:
            data = struct.unpack('=Bff', buf[2:])
        elif msgNum == MISSION_LAWN_W:
            data = struct.unpack('=Bff', buf[2:])
        elif msgNum == MISSION_VEH:
            data = []
        elif msgNum == MAP_SET_VEH_POS:
            data = []
        elif msgNum == GSS_CAL_DEPTH or msgNum == GSS_SET_VEH_POS or msgNum == GSS_SET_MAG_DECL or msgNum == GSS_UPDATE_INS:
            data = []   
        else:
            stat = 'Message type not recognized. MSgNum = {0}'.format(msgNum)
    else:
        stat = 'Not enough data in buffer'
    return data, stat


#!/usr/bin/env python
import struct
from binascii import hexlify, unhexlify
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped
from cavr_msgs.msg import AComms

def convertFromRemusFormat(modemPacket):
    # converts ascii text representation used by RECON to original binary data
    binaryStr = unhexlify(modemPacket)
    return binaryStr

def convertToRemusFormat(binaryStr):
    # converts binary data to its ascii text representation -- needed for RECON
    modemPacket = hexlify(binaryStr)
    return modemPacket

def packHomeMsg(hdrSecs,hdrNsecs,north,east,down):
    structFormat = '=3sIIddf' # 31-byte message (no type data)
    
    # function alias
    structPack = struct.Struct(structFormat).pack
    
    binaryString = structPack('HOM',hdrSecs,hdrNsecs,north,east,down)
    
    return binaryString

def isHomeMsg(binaryString):
    if binaryString[0:3] == 'HOM':
        return True
    else:
        return False

def parseHomeMsgIntoPose(binaryStr):
    structFormat = '=3sIIddf' # 31-byte message (no type data)
    
    # function alias
    structUnpack = struct.Struct(structFormat).unpack
    data = structUnpack(binaryStr);
    
    homePose = PoseStamped()
    
    # Debug
    print 'Parsing %s message...' % (data[0])
    
    homePose.header.seq = 0
    homePose.header.stamp.secs = data[1]
    homePose.header.stamp.nsecs = data[2]
    homePose.header.frame_id = 'world_ned'
    homePose.pose.position.x = data[3]
    homePose.pose.position.y = data[4]
    homePose.pose.position.z = data[5]
    homePose.pose.orientation.x = 0.0
    homePose.pose.orientation.y = 0.0
    homePose.pose.orientation.z = 0.0
    homePose.pose.orientation.w = 0.0
    
    return homePose
    
def parseHomeMsgIntoString(binaryStr):
    
    homePose = parseHomeMsgIntoPose(binaryStr)
    
    homePoseString = AComms()
    
    homePoseString.header = homePose.header
    
    homePoseString.data = 'update_home,%d,%d,%d,%s,%f,%f,%f,%f,%f,%f,%f' \
        %(homePose.header.seq,
          homePose.header.stamp.secs,
          homePose.header.stamp.nsecs,
          homePose.header.frame_id,
          homePose.pose.position.x,
          homePose.pose.position.y,
          homePose.pose.position.z,
          homePose.pose.orientation.x,
          homePose.pose.orientation.y,
          homePose.pose.orientation.z,
          homePose.pose.orientation.w)
    
    return homePoseString
    
def parseHomeMsgIntoData(binaryString):
    structFormat = '=3sIIddf' # 31-byte message (no type data)
    
    # function alias
    structUnpack = struct.Struct(structFormat).unpack
    
    binaryData = structUnpackPack(binaryString)
    
    type = binaryData[0]
    hdrSecs = binaryData[1]
    hdrNsecs = binaryData[2]
    homeNorth = binaryData[3]
    homeEast = binaryData[4]
    homeDown = binaryData[5]
    
    return hdrSecs, hdrNsecs, homeNorth, homeEast, homeDown
    

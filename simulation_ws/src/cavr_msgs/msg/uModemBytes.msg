# 32-byte messages for transmission over WHOI uModem
time stamp

# type of message
uint8 type
uint8 VEHICLE_STATE = 0
uint8 CONTACT_INFO = 1

# position
uint8 depth      # [meters]
int32 latitude   # [-90,90]   [degrees]
int32 longitude  # (-180,180] [degrees]

# orientation
uint16 heading   # [0,360) [degrees]

# uncertainty (std deviation)
uint16 sigmaNorth # [meters]
uint16 sigmaEast  # [meters]
uint16 sigmaDepth # [meters]

# size/extent data of contact(unused--set to zero--for VEHICE_STATE messages)
uint16 sizeNorth # [meters]
uint16 sizeEast  # [meters]
uint16 sizeDepth # [meters]

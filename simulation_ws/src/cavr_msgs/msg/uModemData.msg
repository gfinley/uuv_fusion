# 32-byte messages for transmission over WHOI uModem
time stamp

# type of message
uint8 type
uint8 VEHICLE_STATE = 0
uint8 CONTACT_INFO = 1

# position
float32 depth      # [meters]
float32 latitude   # [-90,90]   [degrees]
float32 longitude  # (-180,180] [degrees]

# orientation
float32 heading   # [0,360) [degrees]

# uncertainty (std deviation)
float32 sigmaNorth # [meters]
float32 sigmaEast  # [meters]
float32 sigmaDepth # [meters]

# size/extent data of contact(unused--set to zero--for VEHICE_STATE messages)
float32 sizeNorth # [meters]
float32 sizeEast  # [meters]
float32 sizeDepth # [meters]

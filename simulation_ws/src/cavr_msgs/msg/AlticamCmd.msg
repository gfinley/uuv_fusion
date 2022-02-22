std_msgs/Header header
geometry_msgs/Point point

# Alticam Command Type
uint8 cameraCmdType
uint8 POWER_CMD = 0            # point.x = 0 (OFF) or point.x = 1 (ON)
uint8 DIRECT_RATES = 1         # point.x = panRate, point.y = tiltRate
uint8 DIRECT_ANGLES = 2        # point.x = panAngle, point.y = tiltAngle
uint8 STABILIZED_RATES = 4     # point.x = panRate, point.y = tiltRate
uint8 STABILIZED_ANGLES = 8    # point.x = azimuth, point.y = elevation
uint8 HOLD_ON_REL_TGT_NED = 16 # point.x = north, point.y = east, point.z = alt

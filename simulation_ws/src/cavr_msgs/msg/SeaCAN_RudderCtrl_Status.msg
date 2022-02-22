# SeaCAN Message 0x0044 (Rudder Controller Status)
Header header

# Angles in radians
float64 currentHeading          # from SeaCAN IMU
float64 currentRudderAngle
float64 requestedHeading

# Rudder Controller Info
uint8 currentMode
uint8 waypointNumber
uint8 RUDDER_CONTROL_MODE = 1
uint8 HEADING_CONTROL_MODE = 2
uint8 TRACK_MODE = 4
uint8 WAYPONT_MODE = 8
uint8 END_OF_WAYPOINTS = 255

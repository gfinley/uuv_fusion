# SeaCAN Message 0x0035 (Rudder Command)
Header header

# Angles in radians
float64 rudderAngleCommand

# Rudder Controller Info
uint8 rudderModeCommand
uint8 RUDDER_CONTROL_MODE = 1
uint8 HEADING_CONTROL_MODE = 2
uint8 TRACK_MODE = 4
uint8 WAYPOINT_MODE = 8
# Message type to publish/record commands sent to the ScanEagle

std_msgs/Header header

# ScanEagle Command Type
uint8 commandType
uint8 CMD_TURN_RATE = 1        
uint8 CMD_ORBIT     = 2
uint8 CMD_CAMERA    = 3

# ScanEagle Command Parameters
float64 param_1
float64 param_2
float64 param_3
float64 param_4


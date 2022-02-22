# SeaCAN Message 0x0107 (Throttle Status)
Header header

# Throttle Info
uint16 throttlePercent
uint8 throttleStatus

# ThrottleStatusBits
uint8 PROPORTIONAL_RPM_MODE = 1
uint8 THROTTLE_PERCENT_MODE = 2 # this is the one we use!
uint8 ENGINE_START_BIT = 4
uint8 ENGINE_STOP_BIT = 8
uint8 DESELECT_ACTUATOR_BIT = 32
uint8 SELECT_ACTUATOR_BIT = 64
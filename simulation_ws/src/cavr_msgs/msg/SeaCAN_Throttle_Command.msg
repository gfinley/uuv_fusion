# SeaCAN Message 0x0055 (Throttle Command)
Header header

# Throttle Command
uint16 throttlePercent

# EngineStatusBits
uint8 engineCommand
uint8 ENGINE_START_BIT = 4
uint8 IGNITION_OFF_BIT = 8      # kill engine
uint8 IGNITION_ON_BIT  = 0      # initial condition

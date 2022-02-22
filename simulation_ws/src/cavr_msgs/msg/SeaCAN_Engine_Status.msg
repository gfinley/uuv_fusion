# SeaCAN Message 0x0118 (Engine Status)
Header header

# Engine Info
uint16 engineRPM
uint8 engineStatus

# EngineStatusBits
uint8 ENGINE_START_BIT = 4
uint8 IGNITION_OFF_BIT = 8      # kill engine
uint8 IGNITION_ON_BIT  = 0      # initial condition

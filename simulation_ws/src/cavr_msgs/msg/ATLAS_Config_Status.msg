# ATLAS System Configuration Status
# REF: SSUP ICD 2.3.3.7
Header header

# State Definitions
uint8 SYSTEM_STATE_INITIALIZATION = 0
uint8 SYSTEM_STATE_STANDBY        = 1
uint8 SYSTEM_STATE_OPERATING      = 2
uint8 SYSTEM_STATE_BUILT_IN_TEST  = 3

uint8 COMPONENT_STATE_OFFLINE     = 0
uint8 COMPONENT_STATE_OPERATIONAL = 1
uint8 COMPONENT_STATE_DEGRADED    = 2
uint8 COMPONENT_STATE_FAILED      = 3

uint8 NAVIGATION_STATE_INACTIVE   = 0
uint8 NAVIGATION_STATE_GOOD       = 1

uint8 ENVIRONMENT_STATE_INACTIVE  = 0
uint8 ENVIRONMENT_STATE_GOOD      = 1

uint8 RECORDER_STATE_PAUSED       = 0
uint8 RECORDER_STATE_RECORDING    = 1
uint8 RECORDER_STATE_FAILED       = 2

uint8 LEAK_STATE_LEAK_DETECTED    = 0
uint8 LEAK_STATE_GOOD             = 1

uint8 THERMAL_STATE_OVERTEMP      = 0
uint8 THERMAL_STATE_WARNING       = 1
uint8 THERMAL_STATE_GOOD          = 2

# Estimated nominal average power dissipation (Watts)
float32 powerLoad

# Time stamp for this system configuration status message (UTC seconds)
float64 time

# Sonar system state
uint8 systemState

# Receive array component state
uint8 receiveState

# Processor component state
uint8 processorState

# Transmitter component state
uint8 transmitterState

# Navigation data stream state
uint8 navigationState

# Environment data stream state
uint8 environmentState

# Leak detector state
uint8 leakState

# Recorder state
uint8 recorderState

# Recorder utilization (%)
float32 recorderUsage

# Receiver temperature (degrees C)
float32 receiverTemp

# Transmitter temperature (degrees C)
float32 transmitterTemp

# Processor temperature (degrees C)
float32 processorTemp

# System thermal state
uint8 thermalState


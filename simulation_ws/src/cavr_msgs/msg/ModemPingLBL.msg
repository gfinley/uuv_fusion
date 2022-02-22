# Message to ping REMUS LBL transponders via the uModem
Header header

# interrogation channel (1-4)
uint8 channel

# synchronization flag
# 0 = default: xmit immediately
# 1 = xmit on rising edge of pps hardware input)
uint8 syncFlag

# synch time-out in [ms]
# 0 = default: unused
uint8 syncTimeout

# max time to listen for LBL reply in [ms]
uint16 replyTimeout

# tranponder flags
# 0 = ignore beacon
# 1 = enable detection for beacon
uint8 alphaFlag
uint8 bravoFlag
uint8 charlieFlag
uint8 deltaFlag
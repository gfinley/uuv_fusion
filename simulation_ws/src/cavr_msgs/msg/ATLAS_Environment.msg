# Environmental Data for ATLAS read from CTD Cast File
Header header

# data for the Environment Data message
float32 waterTempC
float32 salinityPPT
float32 soundSpdMPS

# data for the Sound Velocity Profile message
uint32  numPoints
float64 svpTime
float64 svpLatitude
float64 svpLongitude
float32[] depthM
float32[] velocityMPS
# Raw navigation feature description
Header header # this frame id refers to the frame within which the data is defined (i.e., sensor, or vehicle, etc). 
string ref_frame_id # this frame id refers to the frame within which the feature lives (i.e., inertial, moving, etc)
int32 id # optional
uint16 peak_signal
geometry_msgs/PoseWithCovariance pose # in vehicle frame (i.e., relative to the platform)
geometry_msgs/Polygon geometry
string name # optional

uint8 type
uint8 TYPE_GLASS_SPHERE = 0
uint8 TYPE_SONAR_REFLECTOR = 1
uint8 TYPE_MINE_SHAPE = 2
uint8 TYPE_MISO = 3
uint8 TYPE_BORDER = 4
uint8 TYPE_PYLON = 5
uint8 TYPE_FEATURE = 6
uint8 TYPE_UNDEFINED = 7

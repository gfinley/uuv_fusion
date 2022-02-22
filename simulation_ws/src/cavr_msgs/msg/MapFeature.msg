# Tracked navigation feature description
Header header # this frame id refers to the frame within which the data is defined (i.e., sensor, or vehicle, etc). 
string ref_frame_id # this frame id refers to the frame within which the feature lives (i.e., inertial, moving, etc)
string name
string type_str
uint8 type
int32 id
int32 detections
uint8[] peak_signal
geometry_msgs/PoseWithCovariance pose # relative to the reference frame (ned)
geometry_msgs/Polygon geometry
float32 radius # radius
float32 altitude # altitude 
float32 height  # obstacle height
float64[] n   # northern most point
float64[] e   # eastern most point
float64[] s   # southern most point
float64[] w   # western most point




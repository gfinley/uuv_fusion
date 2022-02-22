# Tracked navigation feature description
Header header # this frame id refers to the frame within which the data is defined (i.e., sensor, or vehicle, etc). 
string ref_frame_id # this frame id refers to the frame within which the feature lives (i.e., inertial, moving, etc)
string name # optional
bool known # flag: is the location of this feature known in the ref frame?
int32 id
int32 detections
uint8[] peak_signal
geometry_msgs/PoseWithCovariance pose # relative to the reference frame
geometry_msgs/Polygon geometry
geometry_msgs/Pose known_pose # known pose of the feature (if available)

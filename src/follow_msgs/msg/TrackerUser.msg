# TrackerUser: Contains a snapshot of a single users tracking data. an ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint. 
std_msgs/Header header
#uint16 uid
int16 tracker_id
#string[] frames
geometry_msgs/Vector3 center_of_mass
geometry_msgs/Vector3 min_point
geometry_msgs/Vector3 max_point
#bool direction
#float64 offset_x
#float64 offset_y
string ns

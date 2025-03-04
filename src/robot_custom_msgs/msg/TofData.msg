## Mapping of 4x4 ToF_Matrix to sensor indices
# Row-major order mapping:
# Row [0]: Matrix [00, 01, 02, 03] => Sensor indices (left: [03, 02, 01, 00], right: [12, 13, 14, 15])
# Row [1]: Matrix [04, 05, 06, 07] => Sensor indices (left: [07, 06, 05, 04], right: [08, 09, 10, 11])
# Row [2]: Matrix [08, 09, 10, 11] => Sensor indices (left: [11, 10, 09, 08], right: [04, 05, 06, 07])
# Row [3]: Matrix [12, 13, 14, 15] => Sensor indices (left: [15, 14, 13, 12], right: [00, 01, 02, 03])

builtin_interfaces/Time timestamp

# Row-major order 4x4 matrix (16 elements)
float64 top             # Single value for the top sensor
float64[16] bot_left    # Left sensor 4x4 matrix
float64[16] bot_right   # Right sensor 4x4 matrix

# Optional status fields for additional state information
uint8 top_status
uint8 bot_status

# AMCL pose
float64 robot_x
float64 robot_y
float64 robot_angle
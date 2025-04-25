builtin_interfaces/Time timestamp

# front - front_L - back_L - back - back_R - front_R
# Cliff Detection Flag
bool ff
bool fl
bool fr
bool bb
bool bl
bool br

# Bottom IR sensor ADC data
int16 adc_ff
int16 adc_fl
int16 adc_fr
int16 adc_bb
int16 adc_bl
int16 adc_br

# AMCL pose
float64 robot_x
float64 robot_y
float64 robot_angle
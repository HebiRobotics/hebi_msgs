# Used only for time-stamping the message.
std_msgs/Header header

# Linear displacement components in m
float64 dx
float64 dy
float64 dz
# Angular displacement components in rad
float64 droll
float64 dpitch
float64 dyaw

# Duration to execute the jog command
float64 duration
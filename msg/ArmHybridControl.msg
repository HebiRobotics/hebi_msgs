# ArmHybridControl.msg
# For hybrid motion-force control of arm
# Axes with zero wrench elements will use pose control, while axes with non-zero wrench elements will use force control.
geometry_msgs/Pose pose # pose values of force control axes are irrelevant
geometry_msgs/Twist twist # currently not utilized
geometry_msgs/Wrench wrench # non-zero and zero for force control and position control, respectively
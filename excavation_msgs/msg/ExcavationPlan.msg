# ExcavationPlan.msg
# The complete pre-planned excavation sequence

std_msgs/Header header

# Plan metadata
uint32 total_scoops
float64 total_target_volume       # m^3

# Ordered list of scoop primitives
excavation_msgs/ScoopAction[] scoops

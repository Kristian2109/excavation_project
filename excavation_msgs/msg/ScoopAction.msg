# ScoopAction.msg
# Describes a single scoop primitive

uint32 scoop_id                     # Sequential index in the excavation plan

# Arm waypoints (in world frame)
geometry_msgs/Pose entry_pose       # Arm pose before entering terrain
geometry_msgs/Pose[] scoop_path     # Waypoints during the scoop
geometry_msgs/Pose exit_pose        # Arm pose after lifting out

# Cells affected by this scoop
uint32[] affected_cell_indices      # Flat indices into the 3D grid
float64 expected_removed_volume     # Expected removed volume in m^3

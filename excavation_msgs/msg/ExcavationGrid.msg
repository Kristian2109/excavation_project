# ExcavationGrid.msg
# Summary of the 3D excavation grid state

std_msgs/Header header

# Grid parameters
float64 resolution              # Cell size in metres
uint32 size_x                   # Grid cells along X
uint32 size_y                   # Grid cells along Y
uint32 size_z                   # Grid cells along Z

# Grid origin (world frame)
geometry_msgs/Point origin

# Volume statistics
uint32 total_cells              # Total cells in target area
uint32 excavated_cells          # Cells already excavated
uint32 remaining_cells          # Cells still to excavate
float64 remaining_volume        # Remaining volume in m^3
float64 completion_fraction     # 0.0 … 1.0

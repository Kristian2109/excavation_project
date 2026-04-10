# MissionStatus.msg
# Current state of the mission

std_msgs/Header header

# Mission state
uint8 IDLE               = 0
uint8 MOVING_TO_WORK_POS = 1
uint8 EXCAVATING         = 2
uint8 COMPLETED          = 3
uint8 FAILED             = 4

uint8 state

# Progress
uint32 current_scoop_id        # Scoop currently being executed
uint32 total_scoops            # Total scoops in the plan
float64 remaining_volume       # m^3
float64 completion_fraction    # 0.0 … 1.0
string status_text             # Human-readable status

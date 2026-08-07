# Gait phase from the host-side Muto trajectory generator. Foot values are the
# continuous IK targets before nearest-degree controller quantization. This is
# not measured foot contact. A STANCE value means the trajectory target is on
# the nominal ground plane; it does not prove contact or no slip.

uint8 SWING=0
uint8 STANCE=1
uint8 UNKNOWN=255

# Leg array order:
# 0 right_front, 1 right_middle, 2 right_rear,
# 3 left_rear, 4 left_middle, 5 left_front.
std_msgs/Header header
uint64 sequence
string mode
uint8 phase_index
uint8 cycle_length
bool cycle_complete
uint8[6] leg_state
# Foot targets are expressed in header.frame_id using ROS REP-103 axes and
# millimetres: x forward, y left, z up.
float32[6] foot_x_mm
float32[6] foot_y_mm
float32[6] foot_z_mm

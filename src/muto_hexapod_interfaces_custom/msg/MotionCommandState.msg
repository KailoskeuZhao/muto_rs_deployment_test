# Mapping of one requested physical body twist onto the Muto gait.
#
# `predicted_twist` is feed-forward mapping output, not measured odometry. In
# geometric mode it is commanded gait kinematics from continuous amplitudes; in
# calibrated/legacy mode it is the selected measured-profile value.

std_msgs/Header header
geometry_msgs/Twist requested_twist
geometry_msgs/Twist predicted_twist
# Rounded diagnostics for the latest amplitudes selected by the SI mapper.
# These may wait for a gait boundary before becoming active.
int16 x_level
int16 y_level
int16 z_level
string mode
# Rounded diagnostics for the latest motor phase, kept separate from the
# selected request.
int16 active_x_level
int16 active_y_level
int16 active_z_level
string active_mode
bool replacement_pending
# Upper active-mapping/mechanical clipping, separate from level projection.
bool saturated
bool x_saturated
bool y_saturated
bool yaw_saturated
# Any requested-to-executable mismatch, including clipping and quantization.
bool projected
bool quantized
bool x_projected
bool y_projected
bool yaw_projected
bool x_projected_to_zero
bool y_projected_to_zero
bool yaw_projected_to_zero
bool unsupported
bool unsupported_combination
string detail
string calibration_profile
# The mapping is valid only for this configured gait-phase rate.
float32 calibration_phase_rate_hz
float32 configured_phase_rate_hz
# Rolling diagnostic measurement; it does not rescale an active command.
float32 observed_phase_rate_hz

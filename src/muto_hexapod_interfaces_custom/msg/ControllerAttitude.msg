# Vendor-fused Euler attitude returned by the Muto baseboard endpoint 0x60.
# The timestamp is the host receive time because the controller supplies no
# acquisition timestamp or sequence number. frame_id is intentionally empty
# until the controller axis signs, Euler order, and reference are validated.
std_msgs/Header header

# Decoded exactly as the vendor library does: signed centidegrees / 100.
float64 roll_deg
float64 pitch_deg
float64 yaw_deg

# Unscaled byte from the controller. The vendor documentation does not state
# its unit, even though it is labelled as temperature.
uint8 temperature_raw

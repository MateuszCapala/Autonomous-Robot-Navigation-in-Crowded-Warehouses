# Array of tracked humans, sorted by distance to robot ascending
# Padded to M_max=5 with dummy entries (distance=999) if fewer detected
std_msgs/Header header

HumanState[] humans
uint32 num_confirmed         # actual confirmed tracks (not counting dummies)

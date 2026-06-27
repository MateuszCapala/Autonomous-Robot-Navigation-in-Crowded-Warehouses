# Single tracked human state with velocity and predictions
std_msgs/Header header

uint32 id                   # track id
float64 x                   # position x [m] in odom frame
float64 y                   # position y [m] in odom frame
float64 vx                  # velocity x [m/s]
float64 vy                  # velocity y [m/s]
float64 distance_to_robot   # current distance [m]

# N=30 step predictions (constant velocity model)
float64[] pred_x
float64[] pred_y

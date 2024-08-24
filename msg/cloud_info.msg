# Cloud Info
Header header 

int64 imuAvailable
int64 odomAvailable

float32 imuRollInit
float32 imuPitchInit
float32 imuYawInit

# Initial guess
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 edge_deskewed   # edge cloud deskewed
sensor_msgs/PointCloud2 surf_deskewed   # surf cloud deskewed
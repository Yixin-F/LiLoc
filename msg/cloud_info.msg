# Cloud Info
Header header 

float32 startOrientation
float32 endOrientation
float32 orientationDiff

int64 imuAvailable
int64 odomAvailable

float32 imuRollInit
float32 imuPitchInit
float32 imuYawInit

# Initial guess for next ndt
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
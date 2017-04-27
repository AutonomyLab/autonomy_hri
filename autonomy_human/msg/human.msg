uint8 STATUS_UNKNOWN=0
uint8 STATUS_LOST=1
uint8 STATUS_TRACKING=2

Header header
uint8 status

uint32 numFaces
sensor_msgs/RegionOfInterest faceROI
sensor_msgs/RegionOfInterest rightHROI
sensor_msgs/RegionOfInterest leftHROI
uint32 faceScore
float32[2] flowScore

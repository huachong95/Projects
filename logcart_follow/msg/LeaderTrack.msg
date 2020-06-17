sensor_msgs/LaserScan obstacles # distance of obstacles in metres
geometry_msgs/Point32 centroid  # position of leader
uint8 leaderTrackStatus
uint8 LEADERTRACKSTATUS_LEADER_NONE=0
uint8 LEADERTRACKSTATUS_LEADER_IDENTIFIED=1
uint8 LEADERTRACKSTATUS_LEADER_TRACKING=2
uint8 LEADERTRACKSTATUS_LEADER_LOST=3

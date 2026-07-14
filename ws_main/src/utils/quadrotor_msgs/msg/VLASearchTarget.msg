uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_LIDAR=1
uint8 SOURCE_MOGE=2

std_msgs/Header header
uint32 task_session_id
uint32 observation_batch_id
uint32 request_id
uint8 observation_index
bool success
uint8 source
geometry_msgs/Pose pose
string error

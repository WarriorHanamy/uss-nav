uint8      drone_id
uint8      source_task_id
float32[3] goal

# Waypoint window (multi-goal batch). waypoints is a flattened xyz array with
# at most 3 points in travel order; the last point duplicates `goal` (yaw fields
# apply to the final waypoint only). Empty array => single-goal legacy behavior.
uint32    batch_id
float32[] waypoints
float32    yaw
bool       look_forward
bool       goal_to_follower
bool       yaw_low_speed

# yaw限速模式。保留yaw_low_speed用于兼容旧发布者，新代码优先使用yaw_mode。
uint8 YAW_MODE_NORMAL=0
uint8 YAW_MODE_LOW_SPEED=1
uint8 YAW_MODE_PANORAMA=2
uint8 yaw_mode

# yaw路径模式。默认使用最短角路径，全景旋转使用保持指定方向的连续角路径。
uint8 YAW_PATH_SHORTEST=0
uint8 YAW_PATH_KEEP_DIRECTION=1
uint8 yaw_path_mode

uint8 SOURCE_TASK_EXPLORATION=2
uint8 SOURCE_TASK_COUNTING=8
uint8 SOURCE_TASK_VLA_SEARCH=9

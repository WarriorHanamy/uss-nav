# VLA_Swarm Target Estimator

该包提供 VLA_Swarm bbox 三维定位的主备链路：

```text
/vla_swarm/bbox
  -> bbox_lidar_target_estimator
  -> 成功：/vla_swarm/target
  -> 失败：/vla_swarm/bbox/fallback
  -> moge_target_estimator.py
  -> /vla_swarm/target
```

`VLASwarmBBox` 携带任务 session、Observation 批次、请求 ID、观察方向和采集时间。
两个 estimator 均按照采集时间匹配历史传感器数据，不使用 LLM 返回时的实时位姿。
Observation 0-3 来自同一个前视相机的分时采集，`observation_index` 不是物理相机编号。
机体转角由历史 odometry 提供，相机到机体的固定外参通过
`camera_in_base_rpy_deg` 和 `camera_in_base_t_xyz` 配置，不能按 Observation 序号重复旋转。
MoGe 直接订阅 `/vla_swarm/observation` 中的固化压缩图像。
LiDAR 节点也订阅该话题，并按 session、批次和序号保留采集位姿及最近点云快照，避免 LLM
返回较慢时普通历史缓存已经淘汰拍照时数据。

启动示例：

```bash
roslaunch box_odom_estimator vla_swarm_target_estimator.launch \
  odom_topic:=/ekf_quat/ekf_odom \
  cloud_topic:=/drone_0_laserMapping/cloud_registered \
  camera_info_topic:=/drone_0/camera/color/info \
  observation_topic:=/vla_swarm/observation \
  moge_root:=/path/to/MoGe \
  moge_model_path:=/path/to/model.pt
```

仿真使用 `ego_planner/launch/obj_nav.launch` 时，上述 estimator 会被直接启动，默认点云为
`/livox/lidar_world`，CameraInfo 为 `/drone_0/camera/color/info`，里程计沿用
`obj_nav.launch` 的 `odom_topic`。可通过顶层 `vla_swarm_cloud_topic`、
`vla_swarm_camera_info_topic`、`vla_swarm_moge_root` 和
`vla_swarm_moge_model_path` 参数覆盖。

MoGe 模型只在 LiDAR 失败并收到 fallback bbox 后加载。`moge_root` 和
`moge_model_path` 必须由部署环境提供，仓库不保存模型或机器绑定路径。

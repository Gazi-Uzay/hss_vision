# hss_vision

Camera driver node that publishes `/camera/image_raw` (sensor_msgs/Image) and `/camera/camera_info` (hss_interfaces/CameraInfo).

## Build
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select hss_vision
source install/setup.bash
```

## Run
```bash
ros2 launch hss_vision camera_driver.launch.py params:=$(ros2 pkg prefix hss_vision)/share/hss_vision/config/camera_params.yaml
# or without file params:
ros2 run hss_vision camera_driver
```

## Change params at runtime
```bash
ros2 param set /camera_driver video_source "rtsp://user:pass@192.168.1.10/stream1"
ros2 param set /camera_driver frame_rate 15.0
ros2 param set /camera_driver camera_info_url "file:///home/v/calibs/cam.yaml"
```

## Notes
- Ensure `hss_interfaces` contains `msg/CameraInfo.msg` with fields compatible to be filled by the mapper in `camera_driver_node.py`.
- For best RTSP performance, consider `backend: gstreamer` and pass a GStreamer pipeline as `video_source`.

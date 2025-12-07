# Run examples in `rtabmap_examples` package
Step 1: Launch the Camera (Infra Mode) Run this in your first terminal. We turn off the emitter (laser dots) because they confuse the visual odometry algorithm.
```
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_color:=false \
    depth_module.emitter_enabled:=0 \
    enable_sync:=true
```
Step 2: Launch RTAB-Map + RViz Run this in your second terminal.

* rgb_topic: We trick RTAB-Map into using the Infrared image as if it were the "RGB" image.

* rviz:=true: This answers your second question—it automatically opens the visualization window.
```
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/camera/infra1/image_rect_raw \
    camera_info_topic:=/camera/camera/infra1/camera_info \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    frame_id:=camera_link \
    approx_sync:=false \
    wait_imu_to_init:=false \
    rviz:=true
```
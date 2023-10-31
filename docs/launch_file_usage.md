# launch file usage

We have provide many pre-defined launch files on launch folder to configure and launch `orbbec_camera_node`. Use can directly run them by execute below
command on `colcon` workspace（such as `~/ros2_ws`）.

```bash
. ./install/setup.bash
ros2 launch orbbec_camera astra.launch.py # replace astra.launch.py with other launch file for your camera.
```

## Parameters

The following are all available launch parameters:

* `connection_delay`: The delay time in milliseconds for reopening the device.
  Some devices, such as Astra mini, require a longer time to initialize and
  reopening the device immediately can cause
  firmware crashes when hot plugging.
* `enable_point_cloud`: Enables the point cloud.
* `enable_colored_point_cloud`: Enables the RGB point cloud.
* `point_cloud_qos`, `[color|depth|ir]_qos,``[color|depth|ir]_camera_info_qos`: ROS2 Message Quality of Service (QoS)
  settings. The possible values
  are `SYSTEM_DEFAULT`, `DEFAULT`,`PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA`
  and are case-insensitive. These correspond to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`,
  `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`,
  and `SENSOR_DATA`,
  respectively.
* `enable_d2c_viewer`: Publishes the D2C overlay image (for testing only).
* `device_num`: The number of devices. This must be filled in if multiple cameras are required.
* `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
* `ir_width`, `ir_height`, `ir_fps`: The resolution and frame rate of the IR stream.
* `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
* `enable_color`: Enables the RGB camera.
* `enable_depth`: Enables the depth camera.
* `enable_ir`: Enables the IR camera.
* `depth_registration`: Enables hardware alignment the depth frame to color frame.
  This field is required when the `enable_colored_point_cloud` is set to `true`.
* `usb_port`: The USB port of the camera. This is required when multiple cameras are used.
* `enable_accel` : Enables the accelerometer.
* `accel_rate`: The frequency of the accelerometer, the optional values
  are `1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`. The specific value depends on the current camera.
* `accel_range` : The range of the accelerometer, the optional values are `2g`,`4g`,`8g`,`16g`. The specific value
  depends on the current camera.
* `enable_gyro`: Whether to enable the gyroscope.
* `gyro_rate` : The frequency of the gyroscope, the optional values
  are `1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`. The specific value depends on the current camera.
* `gyro_range` : The range of the gyroscope, the optional values
  are `16dps`,`31dps`,`62dps`,`125dps`,`250dps`,`500dps`,`1000dps`,`2000dps`. The specific value depends on the current
  camera.
* `enumerate_net_device` : Whether to enable the function of enumerating network devices. True means enabled, false means disabled.
  This feature is only supported by Femto Mega and Gemini 2 XL devices. When accessing these devices through the network, the IP address of the device needs to be configured in advance. The enable switch needs to be set to true.

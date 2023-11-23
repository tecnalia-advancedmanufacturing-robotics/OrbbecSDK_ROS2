# Services of orbbec_camera_node

This document describes the services of `orbbec_camera_node`. Before using those services, please make sure the `orbbec_camera_node` is running.

## Services

We have list all the services of `orbbec_camera_node` here. But different devices may have different services supported. Please refer to the description of the device you are using, witch can be found in the `orbbec_camera/launch` directory.
>For example: the description of Orbbec Gemini 2 device is [here: orbbec_camera/launch/gemini2_desc.md](../orbbec_camera/launch/gemini2_desc.md).

The name of the following service already expresses its function. However, it should be noted that the corresponding `set_[ir|depth|color]_xxx` and `get_[ir|depth|color]_xxx` **services are only available if you set** `enable[ir|depth|color]` to `true` in the stream that corresponds to the argument of the launch file.

| service | description | srv |
| -- | -- | -- |
| /camera/get_sdk_version | get the version of Orbbec SDK | orbbec_camera_msgs/srv/GetString |
| /camera/get_device_info | get device info(pid, serial number, firmware version, etc.) | orbbec_camera_msgs/srv/GetDeviceInfo |
| /camera/get_auto_white_balance | get the auto white balance status of the color camera | std_srvs/srv/GetBool |
| /camera/set_auto_white_balance | set the auto white balance status of the color camera | std_srvs/srv/SetBool |
| /camera/get_white_balance | get the white balance value of the color camera | orbbec_camera_msgs/srv/GetInt32 |
| /camera/set_white_balance | set the white balance value of the color camera | orbbec_camera_msgs/srv/SetInt32 |
| /camera/set_color_auto_exposure | set the auto exposure status of the color camera | std_srvs/srv/SetBool |
| /camera/get_color_exposure | get the exposure value of the color camera, unit: 100us | orbbec_camera_msgs/srv/GetInt32 |
| /camera/set_color_exposure | set the exposure value of the color camera, unit: 100us; set the auto exposure status of the color camera to false first| orbbec_camera_msgs/srv/SetInt32 |
| /camera/get_color_gain | get the color gain value of the color camera | orbbec_camera_msgs/srv/GetInt32 |
| /camera/set_color_gain | set the color gain value of the color camera | orbbec_camera_msgs/srv/SetInt32 |
| /camera/set_depth_auto_exposure | set the auto exposure status of the depth camera | std_srvs/srv/SetBool |
| /camera/get_depth_exposure | get the exposure value of the depth camera, unit: 100us | orbbec_camera_msgs/srv/GetInt32 |
| /camera/set_depth_exposure | set the exposure value of the depth camera, unit: 100us; set the auto exposure status of the depth camera to false first | orbbec_camera_msgs/srv/SetInt32 |
| /camera/get_depth_gain | get the depth gain value of the depth camera | orbbec_camera_msgs/srv/GetInt32 |
| /camera/set_depth_gain | set the gain value of the depth camera | orbbec_camera_msgs/srv/SetInt32 |
| /camera/set_ir_auto_exposure | set the auto exposure status of the infrared camera | std_srvs/srv/SetBool |
| /camera/get_ir_exposure | get the exposure value of the infrared camera, unit: 100us | orbbec_camera_msgs/srv/GetInt32 |
| /camera/set_ir_exposure | set the exposure value of the infrared camera, unit: 100us; set the auto exposure status of the depth camera to false first  | orbbec_camera_msgs/srv/SetInt32 |
| /camera/get_ir_gain | get the gain value of the infrared camera | orbbec_camera_msgs/srv/GetInt32 |
| /camera/set_ir_gain | set the gain value of the infrared camera | orbbec_camera_msgs/srv/SetInt32 |
| /camera/set_fan_work_mode | set the fan work mode | orbbec_camera_msgs/srv/SetInt32 |
| /camera/set_laser_enable | set the laser to on(true) or off(false)  | std_srvs/srv/SetBool |
| /camera/set_floor_enable | set the floor lamp to on(true) or off(false) | std_srvs/srv/SetBool |
| /camera/set_ldp_enable | set the ldp to enable(true) or disable(false) | std_srvs/srv/SetBool |
| /camera/get_ldp_status | Get the trigger state of the laser protection. When the state is true, the protection function is turned on, and the laser will be forced to turn off | std_srvs/srv/GetBool |
| /camera/toggle_color | enable/disable the color camera | std_srvs/srv/SetBool |
| /camera/toggle_depth | enable/disable the depth camera | std_srvs/srv/SetBool |
| /camera/toggle_ir | enable/disable the infrared camera| std_srvs/srv/SetBool |

## Usage

The fully usage of service please refer to the official [Wiki](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).

Shortly, you can refer to the following command to call the service:

``` bash
# ros2 service call <service_name> <service_type> <arguments>

# sdk version:
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/GetString "{}"

# orbbec_camera_msgs/GetDeviceInfo:
ros2 service call /camera/get_device_info orbbec_camera_msgs/GetDeviceInfo "{}"

# std_srvs/SetBool & std_srvs/GetBool:
ros2 service call /camera/set_auto_white_balance std_srvs/SetBool "{data: true}"
ros2 service call /camera/get_auto_white_balance std_srvs/GetBool "{}"

# orbbec_camera_msgs/SetInt32 & orbbec_camera_msgs/GetInt32:
ros2 service call /camera/set_white_balance orbbec_camera_msgs/SetInt32 "{data: 500}"
ros2 service call /camera/get_white_balance orbbec_camera_msgs/GetInt32 "{}"
```
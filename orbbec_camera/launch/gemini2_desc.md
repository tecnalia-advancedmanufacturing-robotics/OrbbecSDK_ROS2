# Device Description Of Orbbec Gemini 2

## Device-specific parameters of launch file for Orbbec Gemini 2

Here, We exclusively list the parameters that are specific to the device and supported by the Orbbec Gemini 2.**Please read [docs/launch_file_parameters.md](../../docs/launch_file_parameters.md) at first to understand the basic parameters.**

* `product_id`
The product ID of Orbbec Gemini 2 is `0x0660`

* `color_width`x`color_height`
The resolution of color camera, supported value:

  * 1920 x 1080
  * 1280 x 720
  * 640 x 480
  * 640 x 360, default

* `color_fps`
The frame rate of color camera, supported value:
  * 15, default
  * 30
  * 60

* `color_format`
supported color formats:
  * `MJPG`, default
  * `YUYV`

* `enable_color_auto_exposure`
Enables/disables auto exposure of color camera, supported value:
  * `true`, default
  * `false`

* `depth_width`x`depth_height`
The resolution of depth camera, supported value:
  * 1280 x 800
  * 640 x 400, default
  * 320 x 200

* `depth_fps`
The frame rate of depth camera, supported value:
  * 15, default
  * 30
  * 60, only for 640 x 480 and 320 x 240 resolution in 'Binned Sparse Default' Mode

* `depth_format`
supported depth formats:
  * `Y14`, default
  * `RLE`

* `enable_depth_auto_exposure`
Enables/disables auto exposure of depth camera, supported value:
  * `true`, default
  * `false`

* `ir_width`x`ir_height`
The resolution of infrared camera, supported profile:
  * 1280 x 800
  * 640 x 400, default
  * 320 x 200

* `ir_fps`
The frame rate of infrared camera, supported profile:
  * 15, default
  * 30
  * 60, only for 640 x 480 and 320 x 240 resolution in 'Binned Sparse Default' Mode

* `ir_format`
supported infrared formats:
  * `Y8`, default
  * `MJPG`

* `enable_ir_auto_exposure`
Enables/disables auto exposure of infrared camera, supported value:
  * `true`, default
  * `false`

* `accel_rate`
The sample rate of accelerometer in Hz, the value must be same as `gyro_rate`, supported value:
  * 100, default
  * 200
  * 500
  * 1000

* `accel_range`
The measuring range of accelerometer, supported value:
  * `4g`, default

* `gyro_rate`
The sample rate of gyroscope, the value must be same as `accel_rate`, supported value:
  * 100, default
  * 200
  * 500
  * 1000

* `gyro_range`
The range of gyroscope, supported value:
  * `1000dps`, default

* `sync_mode`
The synchronization mode of the device, supported value:
  * free_run, default
  * standalone
  * primary
  * secondary-synced
  * software triggering
  * hardware triggering

* `depth_delay_us`
The delay time of depth frame，must set it to 0.

* `color_delay_us`
The delay time of color frame，must set it to 0.

* `ir_delay_us`
The delay time of infrared frame，must set it to 0.

* `trigger2image_delay_us`
The delay time of trigger to image frame，must set it to 0.

* `trigger_out_enabled`
Enables/disables trigger signal output, supported value:
  * `true`
  * `false`, default

* `trigger_out_delay_us`
Trigger signal output delay time, must set it to 0.

* `enable_frame_sync`
Enables/disables frame synchronization, supported value:
  * `true` , default
  * `false`

* `enable_soft_filter`
Enables/disables the soft filter of depth image, supported value:
  * `true`
  * `false`, default

* `soft_filter_max_diff`
Integer, parameter of the soft filter. Please keep the default value: -1.

* `soft_filter_speckle_size`
Integer, parameter of the soft filter. Please keep the default value: -1.

* `depth_work_mode`
The depth work mode, supported value:
  * "Unbinned Dense Default"
  * "Unbinned Sparse Default"
  * "Binned Sparse Default"
  * "Obstacle Avoidance", default

## Device-specific service for Orbbec Gemini 2

Here, We exclusively list the parameters that are specific to the device and supported by the Orbbec Gemini 2.**Please read [docs/services_description.md](../../docs/services_description.md) at first to understand the basic services.**

Shortly usage:

``` bash
# ros2 service call <service_name> <service_type> <arguments>

# sdk version:
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/GetString "{}"

# orbbec_camera_msgs/GetDeviceInfo:
ros2 service call /camera/get_device_info orbbec_camera_msgs/GetDeviceInfo "{}"

# for std_srvs/SetBool & std_srvs/GetBool:
ros2 service call /camera/set_auto_white_balance std_srvs/SetBool "{data: true}"
ros2 service call /camera/get_auto_white_balance std_srvs/GetBool "{}"

# for orbbec_camera_msgs/SetInt32 & orbbec_camera_msgs/GetInt32:
ros2 service call /camera/set_white_balance orbbec_camera_msgs/SetInt32 "{data: 500}"
ros2 service call /camera/get_white_balance orbbec_camera_msgs/GetInt32 "{}"
```

* `/camera/get_auto_white_balance` and `/camera/set_auto_white_balance`
The service to get and set the auto white balance of color camera.
  * service type: `std_srvs/srv/GetBool` and `std_srvs/srv/SetBool`
  * default value: true

* `/camera/get_white_balance` and `/camera/set_white_balance`
The service to get and set the white balance of color camera.
  * service_type: `orbbec_camera_msgs/srv/SetInt32` and `orbbec_camera_msgs/srv/GetInt32`
  * minium value: 2800
  * maximum value: 6800
  * default value:
  * step: 10

* `/camera/set_color_auto_exposure`
The service to set the auto exposure of color camera.
  * service type:  `std_srvs/srv/SetBool`
  * default value: true

* `/camera/get_color_exposure` and `/camera/set_color_exposure`
The service to get and set the exposure of color camera.
  * service type: `orbbec_camera_msgs/srv/SetInt32` and `orbbec_camera_msgs/srv/GetInt32`
  * minium value: 0
  * maximum value: 33000
  * default value:
  * step: 1

* `/camera/set_color_gain` and `/camera/get_color_gain`
The service to get and set the gain of color camera.
  * service type: `orbbec_camera_msgs/srv/SetInt32` and `orbbec_camera_msgs/srv/GetInt32`
  * minium value: 1
  * maximum value: 255
  * default value:
  * step: 1

* `/camera/set_depth_auto_exposure`
The service to set the auto exposure of depth camera.
  * service type:  `std_srvs/srv/SetBool`
  * default value: true

* `/camera/get_depth_exposure` and `/camera/set_depth_exposure`
The service to get and set the exposure of depth camera.
  * service type: `orbbec_camera_msgs/srv/SetInt32` and `orbbec_camera_msgs/srv/GetInt32`
  * minium value: 200
  * maximum value: 500
  * default value:
  * step: 1

* `/camera/set_depth_gain` and `/camera/get_depth_gain`
The service to get and set the gain of depth camera.
  * service type: `orbbec_camera_msgs/srv/SetInt32` and `orbbec_camera_msgs/srv/GetInt32`
  * minium value: 1000
  * maximum value: 15000
  * default value:
  * step: 100

* `/camera/set_ir_auto_exposure`
The service to set the auto exposure of infrared camera.
  * service type:  `std_srvs/srv/SetBool`
  * default value: true

* `/camera/get_ir_exposure` and `/camera/set_ir_exposure`
The service to get and set the exposure of infrared camera.
  * service type: `orbbec_camera_msgs/srv/SetInt32` and `orbbec_camera_msgs/srv/GetInt32`
  * minium value: 200
  * maximum value: 500
  * default value:
  * step: 1

* `/camera/set_ir_gain` and `/camera/get_ir_gain`
The service to get and set the gain of infrared camera.
  * service type: `orbbec_camera_msgs/srv/SetInt32` and `orbbec_camera_msgs/srv/GetInt32`
  * minium value: 1000
  * maximum value: 15000
  * default value:
  * step: 100

* `/camera/set_fan_work_mode`
The service to set the fan work mode.
  * service type: `orbbec_camera_msgs/srv/SetInt32`
  * minium value: 0
  * maximum value: 1
  * default value: 0
  * step: 1

* `/camera/set_laser_enable`
The service to turn on/off the laser, it will be turn on automatically when the depth/ir camera is enabled.
  * service type: `std_srvs/srv/SetBool`
  * default value: false

* `/camera/set_floor_enable`
The service to turn on/off the floor lamp.
  * service type: `std_srvs/srv/SetBool`
  * default value: false

* `/camera/set_ldp_enable`
The service to turn on/off the LDP. After enabling the LDP, this laser will be turned off when the object is to close to the camera.
  * service type: `std_srvs/srv/SetBool`
  * default value: true

* `/camera/get_ldp_status`
The service to get the LDP trigger status. After enabling the LDP, this feature will be triggered when the object is to close to the camera.
  * service type: `std_srvs/srv/GetBool
  * default value: false

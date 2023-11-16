# launch file parameters

This document describes the parameters of launch file for `orbbec_camera_node`.

## 1.Predefined Launch file

We have provided many predefined launch file for different device on `orbbec_camera/launch` directory. You can use it directly or modify it to fit your device.

Using predefined launch file to launch the `orbbec_camera_node`:

``` bash
ros2 launch orbbec_camera gemini2.launch.py
# replace to other predefined launch file or your customized launch file for your device
```

## 2.Parameters

We have list all supported parameters here, but different device may have different supported parameters. If you want modify the predefined launch file, or create a new launch file for your device, **please refer to the predefined launch file and the description doc of you device to ensure what parameters can be used and what value can be set.**

### 2.1 Global parameters

1. `camera_name`
The name of the camera. In fact it is use as the namespace of the node，therefor we can launch multiple node for different device without conflict.
attention: Change this parameter will change the namespace of the node, we should use the right namespace to access the node. More info please refer to [ROS namespace](http://wiki.ros.org/Names).

2. `log_level`
Use to set the log level for the Orbbec SDK. Orbbec SDK is the base library of the camera driver and it will output the log to the console and file. The log level can be set to `debug`, `info`, `warn`, `error`, `fatal`.

### 2.2 Device parameters

Device parameters is used to set which device to connect. You can try to run `libusb` command or run `ros2 run orbbec_camera list_devices_node` to get those info of your device.

1. `vendor_id`
The vendor id of the device. For orbbec camera, the vendor id is always 0x02bc5.

2. `product_id`
The product id of the device. If you don't want to use this parameter to control which device to connect, you can set it to'0x0000' or empty.

3. `serial_number`
The serial number of the device. If you don't want to use this parameter to control which device to connect, you can set it to empty.

4. `usb_port`
The usb port of the device. It is the id of your physical usb port, it will not be changed every time you re-plug-in your device. If you don't want to use this parameter to control which device to connect, you can set it to empty.

### 2.3 Camera parameters

The camera parameters is used to control the camera stream. Include the enabled/disabled of the camera, the resolution, fps, format, etc.
Different devices support different camera types and parameters, please choose the appropriate parameters according to your actual situation.
You can run `ros2 run orbbec_camera list_camera_profile_mode_node` to get the supported camera types and parameters of your device and the support the resolution, fps and format of each camera type.

1. `enable_[color|depth|ir]`
Boolean, Enables/disables the color, depth or ir camera.

2. `[color|depth|ir]_width`
Integer, The width of the color, depth or ir image，uint: pixel.

3. `[color|depth|ir]_height`
Integer, The height of the color, depth or ir image，uint: pixel.

4. `[color|depth|ir]_fps`
Integer, The fps of the color, depth or ir image， unit: fps（frame per second）.

5. `[color|depth|ir]_format`
String, The format of the color, depth or ir image. eg: `Y16`, `Y8`, `RGB`, `MJPG`, etc.

6. `[color|depth|ir]_qos`
ROS2 Message quality of service of the color, depth or ir image.

7. `[color|depth|ir]_camera_info_qos`
ROS2 Message quality of service of the color, depth or ir camera info.

8. `[color|ir]_auto_exposure`
Boolean, enables/disables the auto exposure of the color or ir camera. The ir and depth is the same physical sensor, so the auto exposure of ir and depth is the same.

### 2.4 pointcloud parameters

1. `enable_point_cloud`
Boolean, enables/disables the the publish of point cloud.

2. `enable_colored_point_cloud`
Boolean, enables/disables the colored point cloud. Set `enable_point_cloud` to true first.

3. `depth_registration`
Boolean, enables/disables the hardware alignment the depth frame to color frame. This field is required when the `enable_colored_point_cloud` is set to true.

4. `point_cloud_qos`
ROS2 Message quality of service of the point cloud.

### 2.5 IMU parameters

1. `enable_accel`
Enables the accelerometer.

2. `accel_rate`
The sample rate of the accelerometer, unit: Hz. The optional values are 1.5625hz,3.125hz,6.25hz,12.5hz,25hz,50hz, 100hz,200hz,500hz,1khz,2khz,4khz,8khz,16khz,32khz. The specific value depends on the current camera.

3. `accel_range`
The measuring range of the accelerometer, the optional values are 2g,4g,8g,16g. The specific value depends on the current camera.

4. `enable_gyro`
Enables the gyroscope.

5. `gyro_rate`
The sample rate of the gyroscope, unit: Hz. The optional values are 1.5625hz,3.125hz,6.25hz,12.5hz,25hz,50hz, 100hz,200hz,500hz,1khz,2khz,4khz,8khz,16khz,32khz. The specific value

6. `gyro_range`
The measuring range of the gyroscope, the optional values are 250dps,500dps,1000dps,2000dps. The specific value depends on the current camera.

### 2.6 Multiple device synchronization parameters

As the multiple device synchronization is a very difficult feature, please read the product manual to connect multiple devices and ensure the correctness of the parameters.

1. `sync_mode`
Device synchronization mode. The optional values are `FREE_RUN`, `STANDALONE`, `PRIMARY`, `SECONDARY`, `SECONDARY_SYNCED`, `SOFTWARE_TRIGGERING`, `HARDWARE_TRIGGERING`.

2. `depth_delay_us`
The delay time of the depth image capture after receiving the capture command or trigger signal in microseconds.

3. `color_delay_us`
The delay time of the color image capture after receiving the capture command or trigger signal in microseconds.

4. `trigger2image_delay_us`
The delay time of the image capture after receiving the capture command or trigger signal in microseconds.

5. `trigger_out_enabled`
Enables/disables the trigger signal output.

6. `trigger_out_delay_us`
The delay time of the trigger signal output after receiving the capture command or trigger signal in microseconds.

### 2.7 Others

1. `publish_tf`
Boolean, enables/disables the publish of [transforms]( http://wiki.ros.org/tf).

2. `tf_publish_rate`
Double, the rate of the publish of dynamic transforms, unit: Hz. The `publish_tf` should be set to true first. If `tf_publish_rate` set to 0, the static transforms will be published.

3. `enable_frame_sync`
Boolean, enables/disables the frame synchronization between different type images.

4. `enable_publish_extrinsic`
Boolean, enables/disables the publish of extrinsic parameters.

5. `enable_ldp`
Boolean, enables/disables the laser protection. After enable this feature, the laser will be turned off when the object is too close to the device.

6. `depth_work_mode`
String, the depth work mode of the camera. Run `ros2 run orbbec_camera list_depth_work_mode_node` to get the supported depth work modes of your device.

7. `enable_soft_filter`
Boolean, enables/disables the soft filter of depth image.

8. `soft_filter_max_diff`
Integer, parameter of the soft filter. Please keep the default value.

9. `soft_filter_speckle_size`
Integer, parameter of the soft filter. Please keep the default value.

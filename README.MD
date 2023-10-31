# Orbbec SDK ROS2

The Orbbec SDK ROS2 is a wrapper for the Orbbec 3D camera that provides seamless integration with the ROS2 environment. It supports ROS2 Foxy, Galactic, and Humble distributions.

## 1.Quick start

### 1.1 Environment setup

* Install ROS2: Please refer to the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) guidance.

* Command auto-complete[optional]: put the following two lines into your `.bashrc` or `.zshrc`.

    ```bash
    eval "$(register-python-argcomplete3 ros2)"
    eval "$(register-python-argcomplete3 colcon)"
    ```

* Install deb dependencies

    ```bash
    # assume you have sourced ROS environment, same blow
    sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
    ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager
    ```

### 1.2 Build

* Create `colcon` workspace

    ```bash
    mkdir -p ~/ros2_ws/src
    ```

* Get source code

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
    ```

* Build

    ```bash
    cd ~/ros2_ws/
    # build release, Default is Debug
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
    ```

### 1.3 Run

* Install the udev rules file for usb device access

    ```bash
    cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
    sudo bash install_udev_rules.sh
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

* source environment ( **do this every time you open a new terminal** )

    ```bash
    cd ~/ros2_ws/
    . ./install/setup.bash
    ```

* Launch camera node

    ```bash
    # On terminal 1
    ros2 launch orbbec_camera astra.launch.py # or other launch file, see below table
    ```

* Launch rviz2 to view stream

    ```bash
    # On terminal 2
    rviz2
    ```

    After rviz2 is launched, add `/camera/depth/image_raw/image` topic in `Displays` tab, then you can see the depth stream.

## 2.Nodes

The Orbbec SDK ROS2 contains 4 nodes, which are packaged in the `orbbec_camera` package.

### 2.1 orbbec_camera_node

This node is the main node to configure device and get data stream from device. It always need to be launched with a launch file, because it needs to be configured with many parameters, such as image resolution, image format, etc.

Launch this node with pre-defined launch file:

``` bash
ros2 launch orbbec_camera astra.launch.py # more pre-defined launch file, see below table
```

#### 2.1.1 Pre-defined launch files

You can configure and launch the `orbbec_camera_node `using a ROS launch file. There are many pre-defined launch files written in Python for different device.

| **products**     | **firmware version**             |**launch file**          |**descriptor**                                           |
| ---              | ---                              | ---                     | ---                                                     |
| Femto            | 1.6.7                            | femto.launch.py         | [ReadMe](orbbec_camera/launch/femto.launch.md)          |
| Femto W          | 1.1.8                            | femto.launch.py         | [ReadMe](orbbec_camera/launch/femto.launch.md)          |
| Femto Bolt       | 1.0.6  (unsupported ARM32)       | femto_bolt.launch.py    | [ReadMe](orbbec_camera/launch/femto_bolt.launch.md)     |
| Femto Mega       | 1.1.7  (ubuntu20.04,ubuntu22.04) | femto_mega.launch.py    | [ReadMe](orbbec_camera/launch/femto_mega.launch.md)     |
| Gemini           | 3.0.18                           | gemini.launch.py        | [ReadMe](orbbec_camera/launch/gemini.launch.md)         |
| Gemini E         | 3460                             | gemini_e.launch.py      | [ReadMe](orbbec_camera/launch/gemini_e.launch.md)       |
| Gemini E Lite    | 3606                             | gemini_e_lite.launch.py | [ReadMe](orbbec_camera/launch/gemini_e_lite.launch.md)  |
| Gemini 2         | 1.4.60 /1.4.76                   | gemini2.launch.py       | [ReadMe](orbbec_camera/launch/gemini2.launch.md)        |
| Gemini 2 L       | 1.4.32                           | gemini2L.launch.py      | [ReadMe](orbbec_camera/launch/gemini2L.launch.md)       |
| Gemini 2 XL      | Obox: V1.2.5  VL:1.4.54          | gemini2XL.launch.py     | [ReadMe](orbbec_camera/launch/gemini2XL.launch.md)      |
| Astra+           | 1.0.22/1.0.21/1.0.20/1.0.19      | astra_adv.launch.py     | [ReadMe](orbbec_camera/launch/astra_adv.launch.md)      |
| Astra Mini Pro   | 1007                             | astra.launch.py         | [ReadMe](orbbec_camera/launch/astra.launch.md)          |
| Astra Mini S Pro | 1.0.05                           | astra.launch.py         | [ReadMe](orbbec_camera/launch/astra.launch.md)          |
| Astra 2          | 2.8.20                           | astra2.launch.py        | [ReadMe](orbbec_camera/launch/astra2.launch.md)         |
| DaBai            | 2436                             | dabai.launch.py         | [ReadMe](orbbec_camera/launch/dabai.launch.md)          |
| DaBai DW         | 2606                             | dabai_dw.launch.py      | [ReadMe](orbbec_camera/launch/dabai_dw.launch.md)       |
| DaBai DCW        | 2460                             | dabai_dcw.launch.py     | [ReadMe](orbbec_camera/launch/dabai_dcw.launch.md)      |

Different devices have different supported parameters, parameter value range and default parameter values. Please refer to the ReadMe file of each launch file for more information.

You can modify those parameters value in the pre-defined file, or create a new custom launch file yourself. For further information, please refer to [docs/launch_file_usage.md](docs/launch_file_usage.md).

#### 2.1.2 Topics

The `orbbec_camera_node` use topic to pass camera info, image data, and point cloud data. After `orbbec_camera_node` launched, typing this to list all available topics:

``` shell
ros2 topic list -t |grep camera
```

All available topics:

* `/camera/color/camera_info`: The color camera info.
* `/camera/color/image_raw`: The color stream image.
* `/camera/depth/camera_info`: The depth stream image.
* `/camera/depth/image_raw`: The depth stream image
* `/camera/depth/points`: The point cloud, only available when  `enable_point_cloud` is `true`.
* `/camera/depth_registered/points`: The colored point cloud, only available when  `enable_colored_point_cloud`
  is `true`.
* `/camera/ir/camera_info`: The IR camera info.
* `/camera/ir/image_raw`: The IR stream image

*Note: You can use rviz2 to subscribe those topics to see the data.*

#### 2.1.3 Services

After `orbbec_camera_node` launched, you can use services to set or get parameters of device, such as toggle stream
on/off, set/get exposure time, set/get gain, etc.

For example:

``` bash
# get color camera exposure time
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
# set color camera exposure time
ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 '{data:156}'
```

Please refer to [docs/services.md](docs/services.md) for more information about services and list of all supported services.


### 2.2 list_devices_node

This node is used to list all connected devices, and print out the information of each device.

``` bash
# make should you device has been connected and has not been open by other node before run this.
ros2 run orbbec_camera list_devices_node
```

### 2.3 list_camera_profile_mode_node

This node is used to list all supported camera and profile (resolution, frame rate, image format) of the default device (the first device enumerated).

``` bash
# make should you device has been connected and has not been open by other node before run this.
ros2 run orbbec_camera list_camera_profile_mode_node
```

### 2.4 list_depth_work_mode_node

For some models (Gemini 2, Gemini 2 L and Gemini 2 XL), there are multiple depth work modes. This node is used to list all supported depth work mode for default device (the first device enumerated).

``` bash
# make should you device has been connected and has not been open by other node before run this.
ros2 run orbbec_camera list_depth_work_mode_node
```

## 3. Advance

### 3.1 Multiple devices

Using multiple devices to capture camera data from multiple angles and positions can provide more information and diversity to enhance the performance of application algorithms, please refer to [docs/multiple_devices.md](docs/multiple_devices.md) to configure and launch multiple devices.

### 3.2 Use hardware decoder to decode JPEG

#### 3.2.1 rockchip and Amlogic

Depends on `rockchip-mpp-dev` and `rockchip-rga-dev`, not all systems have these two packages, the names may be
different, please search by yourself.
Open `CMakeLists.txt` and set `USE_RK_HW_DECODER` to `ON`.

#### 3.2.2 Nvidia Jetson

Depends on: `jetson_multimedia_api`,`libyuv`.
Open `CMakeLists.txt` and set `USE_NV_HW_DECODER` to `ON`.

### 3.3 Depth work mode switch

* The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
* Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch.py file's support.
* The default depth work mode configuration of xxx.launch.py is the camera's default configuration. If you need to modify it, you can switch to the corresponding mode as needed.
* The specific camera depth work mode support types can be found in the comments of the depth mode.

```python
    # Depth work mode support is as follows:
    # Unbinned Dense Default
    # Unbinned Sparse Default
    # Binned Sparse Default
    DeclareLaunchArgument('depth_work_mode', default_value='')
```

* View depth work modes:

```bash
ros2 run orbbec_camera list_depth_work_mode_node
```

### 3.4 Configuration of depth NFOV and WFOV modes

For the Femto Mega and Femto Bolt devices, the NFOV and WFOV modes are implemented by configuring the resolution of Depth and IR in the launch file.
In launch file, depth_width、depth_height、ir_width、ir_height represents the resolution of the depth  and the resolution of the IR.
The frame fps and resolution of IR must be consistent with the depth. The correspondence between different modes and resolutions is as follows:

* NFOV unbinned: 640 x 576.
* NFOV binned: 320 x 288.
* WFOV unbinned: 1024 x 1024.
* WFOV binned: 512 x 512.

### 3.5 DDS Tuning

The default DDS settings (Galactic) may not be optimal for data transmission， which may cause the data stream frame rate to fall short of the configuration requirements.

Please refer to [docs/dds_tuning.md](docs/dds_tuning.md) to optimize the DDS settings.

## 4. Frequently Asked Questions

* it's possible that the power supply is insufficient.
  To avoid this, do not connect all cameras to the same hub and use a powered hub instead.

* It's also possible that the resolution is too high.
  To resolve this, try lowering the resolution.

Why are there so many launch files here

* The reason for the presence of multiple launch
  files is due to the fact that the default resolutions and image formats of different cameras vary.
  To make it easier to use, the launch files have been separated for each camera.

## 5. License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

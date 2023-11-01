# Orbbec SDK ROS2

Orbbec SDK ROS2 是一个 Orbbec SDK 的 ROS2 封装，为 Orbbec 3D 相机提供无缝集成。支持 ROS2 Foxy, Galactic, 和 Humble 发行版。

## 1.快速开始

### 1.1 环境设置

* 安装 ROS2: 请参考官方安装指导[ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。
* 命令自动补全[可选]: 把下面的两行代码放到 `.bashrc` 或者 `.zshrc` 中。

    ```bash
    eval "$(register-python-argcomplete3 ros2)"
    eval "$(register-python-argcomplete3 colcon)"
    ```

* 安装依赖

    ```bash
    # assume you have sourced ROS environment, same blow
    sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
    ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager
    ```

### 1.2 编译

* 创建 `colcon` 工作空间

    ```bash
    mkdir -p ~/ros2_ws/src
    ```

* 获取源码

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
    ```

* 编译

    ```bash
    cd ~/ros2_ws/
    # build release, Default is Debug
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
    ```

### 1.3 运行

* 安装 udev 规则文件，以获得 usb 设备访问权限

    ```bash
    cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
    sudo bash install_udev_rules.sh
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

* 配置环境(**每次打开新终端都要执行**)

    ```bash
    cd ~/ros2_ws/
    source ./install/setup.bash  # or source ./install/setup.zsh if you are using zsh
    ```

* 启动相机节点

    ```bash
    # On terminal 1
    ros2 launch orbbec_camera astra.launch.py # or other launch file, see below table
    ```

* 启动 rviz2 查看数据流

    ```bash
    # On terminal 2
    rviz2
    ```

    rviz2 启动后，在 `Displays` 标签中添加 `/camera/depth/image_raw/image` topic，即可看到深度数据流。

## 2.Nodes

Orbbec SDK ROS2 提供4个节点，它们都被打包在 `orbbec_camera` 包中。

### 2.1 orbbec_camera_node

这个是主要 node，用来配置设备，获取数据并发布数据流。它需要被一个启动文件启动，因为需要配置很多参数，比如图像分辨率，图像格式等

通过于定义好的 launch 文件启动该 node:

``` bash
ros2 launch orbbec_camera astra.launch.py # more pre-defined launch file, see below table
```

#### 2.1.1 预定义 launch 文件

你可以通过 ROS launch 文件配置和启动 `orbbec_camera_node`。Orbbec SDK ROS2 提供了许多预定义的 launch 文件，用于不同的设备。

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

不同设备支持参数、参数值范围和默认参数值会有不同，请参考每个launch文件的ReadMe文件了解更多信息。

你可以通过修改于定义 launch 文件中的参数值来调整配置，或者重新自定义创建一个新的launch文件。 更多信息，请参考[docs/launch_file_usage.md](docs/launch_file_usage.md)。

#### 2.1.2 Topics

`orbbec_camera_node` 通过 topic 发布相机信息、图像数据、点云数据。`orbbec_camera_node` 启动后，通过如下命令查看所有可用的 topic:

``` shell
ros2 topic list -t |grep camera
```

所有可用的 topic:

* `/camera/color/camera_info`: 彩色相机信息
* `/camera/color/image_raw`: 彩色相机图像
* `/camera/depth/camera_info`: 深度相机信息
* `/camera/depth/image_raw`: 深度相机图像
* `/camera/depth/points`: 点云数据，仅当 `enable_point_cloud` 为 `true` 时可用
* `/camera/depth_registered/points`: 彩色点云数据，仅当 `enable_colored_point_cloud` 为 `true
* `/camera/ir/camera_info`: IR相机信息
* `/camera/ir/image_raw`: IR相机图像

*Note: 你可以打开rviz2，订阅这些topic来查看相关数据。*

#### 2.1.3 Services

After `orbbec_camera_node` launched, you can use services to set or get parameters of device, such as toggle stream
on/off, set/get exposure time, set/get gain, etc.
`orbbec_camera_node` 启动后, 你可以使用 service 来设置或获取设备参数，如打开/关闭流，设置/获取曝光时间，设置/获取增益，等等。

* 用例:

    ``` bash
    # get color camera exposure time
    ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
    # set color camera exposure time
    ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 '{data:156}'
    # you can replace to other available service get/set more parameters
    ```

* 获取所有可用的 service:

    ```  bash
    ros2 service list -t | grep camera
    ```

请参考 [docs/services.md](docs/services.md) 获取更多关于 service 信息。

### 2.2 list_devices_node

该 node 用于列出所有连接的设备，并打印出每个设备的详细信息。

``` bash
# make should you device has been connected and has not been open by other node before run this.
ros2 run orbbec_camera list_devices_node
```

### 2.3 list_camera_profile_mode_node

该 node 用于列出默认设备(第一个枚举到的设备)所有支持的相机和支持的分辨率、帧率、图像格式。

``` bash
# make should you device has been connected and has not been open by other node before run this.
ros2 run orbbec_camera list_camera_profile_mode_node
```

### 2.4 list_depth_work_mode_node
对于某些型号设备(Gemini 2, Gemini 2 L 和 Gemini 2 XL)，有多个深度工作模式。该 node 用于列出所认设备(第一个枚举到的设备)所有支持的深度工作模式。

``` bash
# make should you device has been connected and has not been open by other node before run this.
ros2 run orbbec_camera list_depth_work_mode_node
```

## 3. 高级使用

### 3.1 多设备使用

使用多个设备采集不同角度和位置的相机数据，可以提供更多信息，从而增加应用算法性能，请参考 [docs/multiple_devices.md](docs/(docs/multiple_devices.md) 配置和启动多个设备。

### 3.2 通过硬件解码器解码JPEG

#### 3.2.1 rockchip and Amlogic

依赖: `rockchip-mpp-dev` 和 `rockchip-rga-dev`，并不是所有的系统都有这些包，包名可能不同，请自行搜索。
Open `CMakeLists.txt` and set `USE_RK_HW_DECODER` to `ON`.

#### 3.2.2 Nvidia Jetson

依赖: `jetson_multimedia_api`,`libyuv`.
打开 `CMakeLists.txt` 将 `USE_NV_HW_DECODER` 设置为 `ON`.

### 3.3 深度模式切换

* The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
* Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch.py file's support.
* The default depth work mode configuration of xxx.launch.py is the camera's default configuration. If you need to modify it, you can switch to the corresponding mode as needed.
* The specific camera depth work mode support types can be found in the comments of the depth mode.

* Gemini 2, Gemini 2 L, and Gemini 2 XL 设备支持切换深度模式。
* 在启动设备之前，可以通过配置 xxx.launch.py 文件切换支持的深度模式。
* 在xxx.launch.py 文件中，配置的默认深度模式即为设备默认配置。如果需要修改，可以切换到需要的模式。

    ```python
        # Depth work mode support is as follows:
        # Unbinned Dense Default
        # Unbinned Sparse Default
        # Binned Sparse Default
        DeclareLaunchArgument('depth_work_mode', default_value='')
    ```

* 可以通过运行`list_depth_work_mode_node`获取设备支持的深度模式:

    ```bash
    ros2 run orbbec_camera list_depth_work_mode_node
    ```

### 3.4 配置深度 NFOV 和 WFOV 模式

对于Femto Mega 和 Femto Bolt 设备，NFOV 和 WFOV 模式是通过在launch文件中配置Depth 和 IR的分辨率实现的。
在launch文件中，depth_width、depth_height、ir_width、ir_height 分别代表Depth 和 IR的分辨率。
帧率与IR分辨率必须保持一致，不同模式与分辨率的对应关系如下：

* NFOV unbinned: 640 x 576.
* NFOV binned: 320 x 288.
* WFOV unbinned: 1024 x 1024.
* WFOV binned: 512 x 512.

### 3.5 DDS 调优

默认的DDS设置（Galactic）可能不是数据传输的最佳设置，这可能导致数据流帧率低于配置要求。
请参考 [docs/dds_tuning.md](docs/dds_tuning.md) 优化 DDS 设置。

## 4. 常见问题

* 供电不足导致设备无法正常工作。
  避免将所有相机连接到同一个hub，使用同一个USB hub 供电。

* 分辨率过高
  如果分辨率设置过高，当系统性能不足时,可能会出现帧率下降的现象。

* 为什么有这么多的预定义 launch 文件?
  不同设备支持的配置参数及参数值的可配置范围不尽相同，因此需要不同的预定义 launch 文件。

## 5. License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

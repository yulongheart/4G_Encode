### README

---

#### 概述

本仓库包含两个Python脚本，专为管理和处理从ROS（机器人操作系统）环境中收集的点云数据、图像和里程计数据而设计。脚本支持将这些数据编码到二进制文件中进行存储，并在需要时进行解码。

#### 文件

1. **`Encode/SaveAndEncode.py`**：
    - **用途**：(设备端)此脚本从ROS主题中捕获数据，进行消息同步，并将数据保存到二进制文件中。它收集图像、点云和里程计数据，基于时间戳同步它们，并将其编码到`.bin`文件中。
    - **主要功能**：
        - **数据同步**：使用消息过滤器同步图像、点云和里程计数据，并设置时间容差。
        - **数据编码**：将同步的数据（包括图像、点云和JSON元数据）编码为二进制格式，并保存到磁盘。
        - **里程计处理**：在编码时加入最近的里程计数据。如果在时间容差内没有可用的里程计数据，输出JSON中的相关字段将设置为`null`。此外，当有里程计数据时，`fire_point`字段有10%的概率被赋值为位姿信息。
        - **时间控制**：按照用户定义的间隔定期保存数据，以确保一致的数据收集。
        - **日志记录**：记录匹配到的里程计数据，以帮助跟踪数据同步过程。

2. **`Encode/Decode.py`**：
    - **用途**：(服务器)此脚本用于解码由`SaveAndEncode.py`生成的二进制文件。它提取并重构原始图像、点云和JSON元数据。
    - **主要功能**：
        - **二进制解码**：读取`.bin`文件，解码存储的数据，并重构原始内容，包括图像（JPEG格式）、点云（ASCII PCD格式）和JSON元数据。
        - **数据组织**：将解码后的数据组织到特定目录（如`JSON`、`JPG`、`3D`、`2D`）中，方便访问和分析。
        - **批处理**：自动处理指定目录中的所有`.bin`文件，实现高效的批量数据解码。

3. **`Encode/DataPublisher.py`**：
    - **用途**：(服务器)此脚本将`Decode.py`解码后的文件发布为ros topic，用于三维建模。

4. **`4G/4G_send.py`**：
    - **用途**：(设备端)此脚本将`SaveAndEncode.py`编码文件发送。

5. **`4G/4G_receive.py`**：
    - **用途**：(服务器)此脚本将`4G_send.py`发送文件接收保存。

6. **`run.sh`**：
    - **用途**： 此脚本为整体启动文件，
                启动roscore->
                设备端运行FastLio、OccupiedMap、yjzb_qt,编码->
                4G发送接收文件->
                服务器段运行解码，pcd3map三维建模。

#### 依赖要求

- **ROS**：这些脚本依赖于ROS进行消息传递和数据收集。
- **Python**：脚本使用Python编写，并需要标准的Python环境以及以下库：
    - `rospy`
    - `message_filters`
    - `sensor_msgs`
    - `nav_msgs`
    - `numpy`
    - `cv2`（OpenCV）
    - `PIL`（Pillow）
    - `struct`
    - `json`
    - `collections`

在运行脚本之前，请确保这些依赖项已安装在您的环境中。



# ROS小车多功能控制系统

这是一个基于ROS的机器人控制系统，集成了巡线、目标跟踪和避障换道等多种功能。系统使用Kinect2相机作为视觉传感器，通过不同的功能包实现不同的控制功能。

## 项目结构

项目包含五个ROS功能包：

### 1. autolabor_pro1_driver
- 功能：机器人底盘驱动
- 主要文件：
  - `launch/driver.launch`：底盘驱动启动文件
  - `src/autolabor_driver.cpp`：底盘驱动实现
- 订阅话题：
  - `/cmd_vel`：接收运动控制指令

### 2. iai_kinect2
- 功能：Kinect2相机驱动
- 主要文件：
  - `kinect2_bridge/launch/kinect2_bridge.launch`：相机驱动启动文件
- 发布话题：
  - `/kinect2/sd/image_color_rect`：RGB图像
  - `/kinect2/sd/image_depth`：深度图像

### 3. riki_line_follower
- 功能：巡线控制
- 主要文件：
  - `launch/riki_line.launch`：巡线功能启动文件
  - `src/detect.cpp`：线检测节点
  - `src/motion_node.cpp`：运动控制节点
- 发布话题：
  - `/cmd_vel`：运动控制指令

### 4. track_pkg
- 功能：目标跟踪
- 主要文件：
  - `launch/tracker.launch`：目标跟踪启动文件
  - `src/runtracker.cpp`：KCF跟踪器实现
- 订阅话题：
  - `/kinect2/sd/image_color_rect`：RGB图像
  - `/kinect2/sd/image_depth`：深度图像
- 发布话题：
  - `/cmd_vel`：运动控制指令

### 5. lane_change_control
- 功能：避障换道
- 主要文件：
  - `launch/lane_change.launch`：避障换道启动文件
  - `src/lane_change_controller.cpp`：避障控制器实现
- 订阅话题：
  - `/kinect2/sd/image_depth`：深度图像
- 发布话题：
  - `/cmd_vel`：运动控制指令

## 使用方法

### 准备工作
1. 确保硬件正确连接：
   - Kinect2相机已连接到USB3.0端口
   - 机器人底盘已连接（默认端口：/dev/ttyUSB0）
2. 检查设备权限：
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

### 功能启动方法

#### 1. 巡线功能
```bash
roslaunch riki_line_follower riki_line.launch
```
这将启动：
- 底盘驱动
- Kinect2相机
- 线检测节点
- 运动控制节点

#### 2. 目标跟踪功能
```bash
roslaunch track_pkg tracker.launch
```
这将启动：
- 底盘驱动
- Kinect2相机
- 目标跟踪节点

使用方法：
1. 等待RGB图像窗口显示
2. 用鼠标在图像中框选要跟踪的目标
3. 系统会自动跟踪选中的目标

#### 3. 避障换道功能
```bash
roslaunch lane_change_control lane_change.launch
```
这将启动：
- 底盘驱动
- Kinect2相机
- 避障控制节点

### 注意事项
1. 这三个功能是互斥的，每次只能运行一个功能
2. 如果启动失败，请检查：
   - USB设备连接状态
   - 设备权限设置
   - Kinect2相机工作状态
3. 可以使用以下命令检查话题：
   ```bash
   rostopic list  # 查看当前所有话题
   rostopic echo /cmd_vel  # 查看运动控制指令
   ```

## 系统架构
所有功能包都遵循相似的架构设计：
1. 底层驱动层：由autolabor_pro1_driver和iai_kinect2提供基础的硬件控制
2. 感知层：通过Kinect2相机获取环境信息（RGB图像和深度图像）
3. 决策控制层：各功能包的核心算法（巡线检测、目标跟踪、避障控制）
4. 执行层：通过发布`/cmd_vel`话题来控制机器人运动

每个功能包都是独立的模块，通过ROS的话题机制进行通信，保证了系统的模块化和可扩展性。

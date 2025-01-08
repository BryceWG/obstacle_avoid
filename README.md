# ROS小车多功能控制系统

这是一个基于ROS的机器人控制系统，集成了巡线、目标跟踪和变道等多种功能。系统支持使用Kinect2相机或普通USB摄像头作为视觉传感器。

## 项目结构

项目包含三个主要的ROS功能包：

### 1. autolabor_pro1_driver
- 功能：机器人底盘驱动
- 主要文件：
  - `launch/driver.launch`：底盘驱动启动文件
- 订阅话题：
  - `/cmd_vel`：接收运动控制指令

### 2. track_pkg
- 功能：目标跟踪
- 主要文件：
  - `launch/track_usb.launch`：目标跟踪启动文件（USB摄像头版本）
  - `src/runtracker.cpp`：KCF跟踪器实现
- 特点：
  - 使用KCF算法实现稳定跟踪
  - 基于目标框大小变化估算距离
  - 平滑的运动控制
- 订阅话题：
  - `/usb_cam/image_raw`：摄像头图像
- 发布话题：
  - `/cmd_vel`：运动控制指令

### 3. lane_change_control
- 功能：变道控制
- 主要文件：
  - `launch/lane_change.launch`：变道控制启动文件
  - `src/lane_change_controller.cpp`：变道控制器实现
- 特点：
  - 基于正弦函数的平滑变道轨迹
  - 三阶段变道过程（转出、直行、回正）
  - 可配置的变道参数
- 发布话题：
  - `/cmd_vel`：运动控制指令

## 使用方法

### 准备工作
1. 确保硬件正确连接：
   - USB摄像头已连接（默认/dev/video0）
   - 机器人底盘已连接（默认/dev/ttyUSB0）
2. 检查设备权限：
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/video0
   ```

### 功能启动方法

#### 1. 目标跟踪功能
```bash
roslaunch track_pkg track_usb.launch
```
使用方法：
1. 等待RGB图像窗口显示
2. 用鼠标在图像中框选要跟踪的目标
3. 系统会自动跟踪选中的目标，并根据目标大小调整速度

可调参数（在launch文件中）：
- `linear_speed`：基础线速度
- `angular_speed`：基础角速度
- `video_device`：摄像头设备号

#### 2. 变道功能
```bash
roslaunch lane_change_control lane_change.launch
```
特点：
- 启动后自动执行一次变道动作
- 平滑的速度和转向控制
- 可配置的变道参数

可调参数（在launch文件中）：
- `linear_speed`：直行速度（默认0.2m/s）
- `max_angular_speed`：最大角速度（默认0.3rad/s）
- `lane_width`：车道宽度（默认0.6m）
- `total_time`：变道总时长（默认5.0s）

### 注意事项
1. 这些功能是互斥的，每次只能运行一个功能
2. 如果启动失败，请检查：
   - USB设备连接状态
   - 设备权限设置
   - 摄像头工作状态
3. 可以使用以下命令检查系统状态：
   ```bash
   rostopic list  # 查看当前所有话题
   rostopic echo /cmd_vel  # 查看运动控制指令
   rqt_image_view  # 查看摄像头图像
   ```
z
## 系统架构
系统采用模块化设计：
1. 底层驱动层：由autolabor_pro1_driver提供底盘控制
2. 感知层：通过USB摄像头获取环境信息
3. 决策控制层：
   - 跟踪功能：KCF目标跟踪算法
   - 变道功能：基于正弦函数的平滑轨迹规划
4. 执行层：通过`/cmd_vel`话题控制机器人运动

每个功能包都是独立的模块，通过ROS的话题机制进行通信，保证了系统的模块化和可扩展性。

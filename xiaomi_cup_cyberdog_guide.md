# 小米杯 CyberDog 开发与落地指导

## 1. 目标与结论

这份指导基于四份本地 PDF、当前仓库代码、以及公开可检索到的 CyberDog 官方资料整理而成，目标不是泛泛介绍，而是给出一套可以直接执行的方案。

先给结论：

1. 比赛开发环境应以 `Ubuntu 20.04 + Docker + ROS 2 Galactic` 为主线，不建议直接脱离赛事镜像重建整套底层环境。
2. 你当前仓库本质上是“上层任务逻辑 + 视觉/IMU感知 + LCM运动控制封装”，它默认底层容器已经提供 ROS2 话题、运动控制程序、以及比赛镜像中的依赖。
3. 最稳妥的开发路径是：
   - 先在赛事 Docker 中跑通仿真
   - 再在同一镜像内补 Python 依赖
   - 再验证摄像头/IMU/topic
   - 再接入你仓库的状态机和运动控制
   - 最后做楼梯、限高杆、二维码、黄线等专项调参
4. 你的仓库已经明确依赖以下输入接口：
   - RGB 图像：`/rgb_camera/image_raw`
   - D435 深度图：`/D435_camera/depth/image_raw`
   - IMU：`/imu`
   - LCM 下行：`udpm://239.255.76.67:7671?ttl=255`
   - LCM 上行：`udpm://239.255.76.67:7670?ttl=255`
5. 你的仓库已经实现了一套完整比赛状态机，入口在：
   - `right_all/main.py`
   - `right_all/robot_state_machine.py`
   - `right_all/motion.py`

## 2. 我分析了哪些资料

### 2.1 本地 PDF

1. `F:\小米杯\cyberdog_race说明文档.pdf`
2. `F:\小米杯\cyberdog_sim_Docker镜像使用说明.pdf`
3. `F:\小米杯\机器狗二次开发操作指南_比赛支持_.pdf`
4. `F:\小米杯\楼梯步态说明.pdf`

### 2.2 当前仓库的关键实现

1. `right_all/main.py`
2. `right_all/robot_state_machine.py`
3. `right_all/motion.py`
4. `right_all/detect/RGB_provider.py`
5. `right_all/detect/D435_provider.py`
6. `right_all/detect/IMU_provider.py`
7. `right_all/robot_control/robot_control_class.py`
8. `right_all/Gait_Def_downwalk.toml`
9. `right_all/Gait_Params_downwalk_full.toml`
10. `src/road_detect/*`

### 2.3 在线公开资料

以下资料主要用于核对官方环境路线、ROS2 版本、Docker 方式和官方代码入口：

1. Docker 官方 Ubuntu 安装文档  
   https://docs.docker.com/engine/install/ubuntu/
2. ROS 2 Galactic 官方文档  
   https://docs.ros.org/en/galactic/index.html
3. Xiaomi CyberDog ROS2 官方仓库  
   https://github.com/MiRoboticsLab/cyberdog_ros2
4. CyberDog ROS2 官方 Wiki 首页  
   https://github.com/MiRoboticsLab/cyberdog_ros2/wiki

说明：

1. “网上所有资料”无法做到数学意义上的穷尽，但上面已经覆盖公开可检索到的主线官方来源。
2. 真正和比赛可落地最相关的，仍然是赛事 PDF 和你手头这套镜像/源码。

## 3. 当前仓库的真实技术结构

你的代码并不是一个完整替代官方底层的软件栈，而是叠加在比赛环境上的任务层。

### 3.1 感知层

1. RGB：
   - `right_all/detect/RGB_provider.py`
   - 订阅 `/rgb_camera/image_raw`
   - 使用 `cv_bridge` 转 OpenCV 图像
   - 使用 `cv2.wechat_qrcode.WeChatQRCode` 做二维码识别
2. 深度：
   - `right_all/detect/D435_provider.py`
   - 订阅 `/D435_camera/depth/image_raw`
   - 直接读取深度图
   - `right_all/detect/D435_detect.py` 用于检测顶部障碍/限高杆一类目标
3. IMU：
   - `right_all/detect/IMU_provider.py`
   - 订阅 `/imu`
   - 用 `transforms3d` 将四元数转欧拉角

### 3.2 控制层

1. `right_all/motion.py` 通过 LCM 向底层运动控制发命令。
2. 通道定义在 `right_all/robot_control/robot_control_class.py`：
   - 发送：`239.255.76.67:7671`
   - 接收：`239.255.76.67:7670`
3. 步态控制不是 ROS topic 直接控电机，而是：
   - 上层状态机发布 `/robot/motion_cmd`
   - `MotionController` 把抽象动作转成 `robot_control_cmd_lcmt`
   - 再通过 LCM 发送给底层
4. 自定义楼梯/下坡步态文件通过 `user_gait_file` 下发：
   - `Gait_Def_downwalk.toml`
   - `Gait_Params_downwalk_full.toml`

### 3.3 任务层

`right_all/robot_state_machine.py` 已经实现比赛主流程，包含：

1. 起步
2. 初始二维码扫描
3. A 区 / B 区入库
4. S 弯
5. 限高杆
6. 石板路
7. 下坡/下台阶类动作
8. 返程

这意味着你现在最缺的不是“再写一套控制框架”，而是把环境、依赖、topic、步态参数、调试流程一次性对齐。

## 4. 第一部分：如何配环境

这一部分按“最稳妥方案”写。

### 4.1 宿主机环境

推荐宿主机：

1. Ubuntu 20.04
2. Docker 20.10.x
3. 能正常显示 GUI
4. 至少 16GB 内存
5. NVIDIA 显卡可选，但不是第一阶段必要条件

安装 Docker：

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker $USER
```

重新登录后验证：

```bash
docker --version
docker info
```

### 4.2 导入赛事镜像

本地文档明确给出比赛镜像导入与运行方式，核心思路是：

```bash
sudo docker load -i cyberdog_race.tar
```

如果你拿到的是其他名称，例如 `carpo_arm64.tar`，就替换文件名。

### 4.3 运行比赛容器

根据比赛文档，推荐形态类似：

```bash
xhost +
sudo docker run -it --shm-size="1g" --privileged=true \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /你的本地代码目录:/workspace \
  cyberdog_sim:v1
```

建议你把代码挂载进去，例如：

```bash
sudo docker run -it --shm-size="1g" --privileged=true \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/you/cyberdog_nudt:/workspace \
  cyberdog_sim:v1
```

这样修改代码不用反复拷贝。

### 4.4 为什么不建议脱离赛事镜像

因为赛事资料已经固定了：

1. ROS 2 版本：`Galactic`
2. 仿真方式：Gazebo + rviz2
3. 通信方式：ROS2 topic + LCM
4. 一部分底层控制程序和接口已经固化在镜像中

如果你自己重建，会在下面这些地方消耗大量时间：

1. ROS2 包版本不一致
2. `cv_bridge` 与 OpenCV ABI 冲突
3. LCM 类型文件和底层控制不匹配
4. 运动接口名、命名空间、topic 名称不一致

所以第一阶段一定要“以比赛镜像为准”。

## 5. 第二部分：如何在环境内装包

这是最容易踩坑的部分。

### 5.1 先区分三类依赖

在 CyberDog 比赛容器里，依赖分三层：

1. 系统层 apt 包
   - ROS2
   - `cv_bridge`
   - 图形库
   - 传感器消息
2. Python 包
   - `numpy`
   - `transforms3d`
   - `lcm`
   - 可能还包括 `opencv-contrib-python`
3. 赛事/官方源码编译产物
   - `colcon build`
   - 安装到 `/opt/ros2/cyberdog` 或工作区 `install/`

### 5.2 你这份仓库至少需要哪些 Python 依赖

根据代码扫描，最低要补齐：

```bash
pip3 install numpy transforms3d lcm
```

如果你在容器里缺 OpenCV Python：

```bash
pip3 install opencv-python
```

但这里要强调：

1. 你的代码用了 `cv2.wechat_qrcode.WeChatQRCode`
2. 这个接口属于 OpenCV contrib 模块
3. 单纯 `opencv-python` 可能没有这个模块

所以二维码功能推荐这样检查：

```bash
python3 - <<'PY'
import cv2
print(cv2.__version__)
print(hasattr(cv2, "wechat_qrcode"))
PY
```

如果输出 `False`，有两种方案：

#### 方案 A：继续用 WeChatQRCode

```bash
pip3 install opencv-contrib-python==4.8.1.78
```

风险：

1. 可能与容器内 `cv_bridge` 绑定的系统 OpenCV 发生 ABI 冲突
2. 如果冲突，表现通常是导入 `cv_bridge` 或 `cv2` 时崩溃

#### 方案 B：改用更稳妥的二维码方案

例如 `pyzbar` 或 OpenCV 的普通 `QRCodeDetector`。  
如果你后续要上真机并追求稳定性，我更建议改成普通 `QRCodeDetector` 或 `pyzbar`，因为它比 `opencv-contrib-python` 更少引入 ABI 风险。

### 5.3 ROS 相关依赖优先用 apt

如果比赛镜像没有以下包，优先用 apt 而不是 pip：

```bash
apt update
apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-galactic-cv-bridge \
  ros-galactic-image-transport \
  ros-galactic-sensor-msgs \
  ros-galactic-tf2-ros \
  ros-galactic-rosbag2-py
```

原因：

1. `cv_bridge` 和 ROS 消息包最好与系统 ROS 版本一致
2. pip 版替代通常更容易出兼容性问题

### 5.4 推荐安装顺序

进入容器后：

```bash
source /opt/ros/galactic/setup.bash
apt update
apt install -y python3-pip
pip3 install -U pip
pip3 install numpy transforms3d lcm
```

然后再验证：

```bash
python3 - <<'PY'
import rclpy
import cv2
import numpy
import transforms3d
import lcm
print("ok")
PY
```

### 5.5 如果容器不能联网

比赛容器常见问题是临时没有外网。此时使用离线方案：

1. 宿主机先下载 wheel
2. 挂载到容器
3. 用本地 wheel 安装

宿主机：

```bash
mkdir -p wheels
pip download -d wheels numpy transforms3d lcm
```

容器内：

```bash
pip3 install --no-index --find-links /workspace/wheels numpy transforms3d lcm
```

## 6. 第三部分：如何编译和加载官方/比赛工作区

比赛开发文档明确给了标准流程。

### 6.1 标准编译命令

```bash
source /opt/ros/galactic/setup.bash
cd /你的工作区
colcon build --merge-install --install-base /opt/ros2/cyberdog
```

或者只编译部分包：

```bash
colcon build --merge-install --packages-select <package_name> --install-base /opt/ros2/cyberdog
```

### 6.2 运行前必须 source

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

如果比赛镜像要求用 `/opt/ros2/cyberdog`：

```bash
source /opt/ros/galactic/setup.bash
source /opt/ros2/cyberdog/setup.bash
```

### 6.3 调试三件套

比赛 PDF 里明确建议先做这三步：

```bash
ros2 topic list
ros2 node list
ros2 service list
```

再进一步：

```bash
ros2 topic echo /imu
ros2 topic echo /rgb_camera/image_raw
ros2 topic echo /D435_camera/depth/image_raw
```

如果这些看不到，就先不要跑你的状态机。

## 7. 第四部分：如何调用摄像头

这里分真机和仿真两类。

### 7.1 真机官方方式

根据《机器狗二次开发操作指南_比赛支持_》中的说明，官方提供了 `camera_service` 服务用于打开/关闭 AI camera。

关键点：

1. 服务名：`camera_service`
2. 服务类型：`protocol/srv/CameraService`
3. 文档给了 C++ 示例
4. 打开成功后，再订阅图像 topic

文档示意流程是：

1. 创建 `camera_service` client
2. 先 stop 再 start
3. 成功后订阅 `"image"` topic

也就是说，在真机上你不能直接假定图像流一直开着，应该先显式拉起服务。

### 7.2 真机推荐检查流程

```bash
ros2 service list | grep camera
ros2 service type /camera_service
ros2 topic list | grep image
```

如果服务存在，先调用官方 demo 或你自己的 client。

### 7.3 你的仓库当前假设

你的 `right_all/detect/RGB_provider.py` 当前直接订阅：

```text
/rgb_camera/image_raw
```

这说明你的仓库不是使用 PDF 示例中的 `"image"`，而是假定系统里已经有一个标准化后的 RGB topic。  
因此你实际部署时要先验证：

```bash
ros2 topic list | grep rgb_camera
ros2 topic echo /rgb_camera/image_raw
```

如果真机只有官方 `"image"` 而没有 `/rgb_camera/image_raw`，你需要做一个转发节点。

### 7.4 图像转发适配方案

如果实际 topic 名字不一致，最稳妥的方法是做一层适配：

1. 订阅实际真机 topic
2. 原样转发到 `/rgb_camera/image_raw`
3. 你的业务代码不改

这样状态机、二维码、黄线检测都可以保持不变。

## 8. 第五部分：如何处理摄像头任务

你的仓库里，摄像头主要承担三类任务。

### 8.1 任务一：二维码识别

位置：

1. `right_all/detect/RGB_provider.py`
2. `src/head/qrcode_scan.py`
3. `src/QR_row/rgb.py`

实现方法：

1. 订阅 RGB 图像
2. 转成 BGR
3. 使用 `WeChatQRCode`
4. 输出 `A-1 / A-2 / B-1 / B-2` 等结果

建议：

1. 把二维码识别单独做成独立节点，先离线验证识别率
2. 再把结果喂给状态机
3. 不要一开始就边走边调二维码，排障太慢

离线验证顺序：

1. 先在录制图片上识别
2. 再在 rosbag/回放流上识别
3. 最后接入实时摄像头

### 8.2 任务二：黄线/区域颜色检测

位置：

1. `right_all/detect/RGB_detect.py`
2. `src/Reverse_Park/visual_detect_yellow.py`
3. `src/Flagstone_road/visual_detect_yellow.py`

现有逻辑是典型 HSV 阈值法。优点是快，缺点是受光照影响大。

建议做三项加固：

1. 入场先做白平衡/曝光稳定等待 1 到 2 秒
2. HSV 阈值不要写死一套，至少准备“强光/正常/偏暗”三套参数
3. 检测条件不要只看单帧，至少做 `N` 帧投票

### 8.3 任务三：深度图处理

位置：

1. `right_all/detect/D435_provider.py`
2. `right_all/detect/D435_detect.py`

当前实现主要是：

1. 读取 `/D435_camera/depth/image_raw`
2. 截取上方区域
3. 统计平均深度
4. 判断是否存在顶部障碍

这个方法适合限高杆/横杆类目标的“有无判定”，但不适合精细定位楼梯落脚点。

如果后续要做更强的楼梯视觉：

1. 先做深度 ROI 分层统计
2. 再做台阶边缘提取
3. 最后才考虑足端落点优化

不要一上来做复杂 3D 重建，比赛时间不够。

## 9. 第六部分：狗的运动控制应该怎么接

### 9.1 先理解你现在的控制链路

你的控制链路不是“视觉直接控制电机”，而是四层：

1. 感知层得到状态
2. 状态机决定动作模式
3. `MotionController` 将模式映射到步态/速度参数
4. LCM 发到底层运动控制

这条链路是正确的，建议保留。

### 9.2 当前仓库的动作抽象

`right_all/motion.py` 已经定义了这些抽象动作：

1. 站立
2. 趴下
3. 低头前进
4. 抬头后退
5. 左转
6. 右转
7. 左平移
8. 右平移
9. 圆弧行走
10. 石板路步态
11. 下行步态

这非常适合比赛。原因是状态机只管“做什么”，不管“每个电机怎么动”。

### 9.3 控制参数如何调

推荐按以下顺序调：

1. 只调 `vel_des`
2. 再调 `yaw_rate`
3. 再调 `step_height`
4. 最后才调 `pitch` 和自定义 gait

原因：

1. `vel_des` 和 `yaw_rate` 最直观
2. `pitch` 和 gait 参数耦合更强，容易把稳定性一起打坏

### 9.4 推荐的基础动作测试

先不要跑完整状态机，先单项验证。

#### 测试 1：站立

目标：

1. 确认 LCM 通信正常
2. 确认底层在响应命令

#### 测试 2：低速前进

建议：

1. `vel_x = 0.1`
2. `step_height = 0.04 ~ 0.06`

#### 测试 3：原地转向

建议：

1. `yaw_rate = 0.12 ~ 0.18`
2. 通过 IMU 闭环判断转满角度后停下

#### 测试 4：低头前进

用于：

1. 限高杆
2. 贴近地面视觉观察
3. 下坡/下台阶准备动作

### 9.5 IMU 闭环一定要保留

你的仓库已经在 `right_all/detect/IMU_detect.py` 和状态机里用 IMU 做方向判断。  
这是必须保留的，因为：

1. 仅靠定时转向，误差会不断累计
2. 地面摩擦、载荷和电池状态都会影响转角
3. 比赛场景里二维码和入库动作都依赖方位精度

原则：

1. 转向命令负责“动起来”
2. IMU 负责“什么时候停”

## 10. 第七部分：楼梯/下坡步态的具体建议

《楼梯步态说明》给出的重点不是让你现写一套 MPC，而是告诉你应该关注哪些参数。

文档核心点可以压缩为五件事：

1. 触地检测
2. 支撑平面估计
3. 摆腿轨迹和落足点调整
4. 前后腿相位切换对齐
5. 上下楼分开调参数

### 10.1 对比赛而言最可实现的路线

你现在最应该采用的是“简化落地版”：

1. 不重写底层控制器
2. 沿用现有 gait 框架
3. 通过自定义 gait 文件和上层状态机做适配
4. 使用 IMU + 深度图做触发，而不是做全自主视觉落足

### 10.2 现阶段建议采用的楼梯/下坡策略

#### 策略 A：盲走步态优先

适用：

1. 已知台阶尺度
2. 场地较固定
3. 时间有限

做法：

1. 使用自定义 gait 文件
2. 进入楼梯前先调整姿态
3. 以低速、较高抬脚、高支撑相占比通过

#### 策略 B：视觉只做触发，不做精确落脚

适用：

1. 需要识别“是否到达台阶/坡道”
2. 需要识别顶部横杆/限高
3. 但没有时间做高质量落足规划

做法：

1. RGB 检测区域线索
2. D435 深度做前方/上方障碍判定
3. IMU 做姿态闭环

### 10.3 你仓库已经具备的楼梯落地条件

1. 自定义 gait 下发已实现
2. `Gait_Def_downwalk.toml` 和 `Gait_Params_downwalk_full.toml` 已存在
3. `motion.py` 已经会在初始化时把 gait 文件通过 LCM 发给底层

所以你的楼梯/下坡工作重点应该放在：

1. gait 参数调优
2. 进入/退出楼梯状态判定
3. 速度与姿态过渡

而不是从零写通信框架。

## 11. 第八部分：仿真环境怎么用来对接你的代码

比赛说明文档已经明确：

1. 仿真基于 Gazebo
2. 提供 ROS2 topic 和 LCM 两套接口
3. 可以运行 `motion_manager` 把 ROS2 指令转换为 LCM

### 11.1 仿真启动

```bash
cd /home/cyberdog_sim
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```

或者分开启动：

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch cyberdog_gazebo race_gazebo.launch.py
```

控制程序：

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

可视化：

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch cyberdog_visual cyberdog_visual.launch.py
```

### 11.2 仿真里先验证什么

第一阶段只验证四件事：

1. `/imu`
2. `/scan`
3. 图像 topic
4. 运动命令能否让狗动起来

### 11.3 如何接你的仓库

建议顺序：

1. 先把 `right_all/detect/*` 的输入 topic 改成仿真里真实存在的 topic
2. 先只跑 provider 节点，确认能收到图像/IMU
3. 再单独跑 `motion.py`
4. 最后跑 `main.py`

这样排障最清晰。

## 12. 第九部分：信息检索能力应该怎么建设

这里我把“信息检索能力”分成两类。

### 12.1 开发期检索能力

也就是你在容器/真机内快速确认系统状态的能力。

必须掌握的命令：

```bash
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /rgb_camera/image_raw
ros2 topic hz /rgb_camera/image_raw
ros2 service list
ros2 node list
ros2 node info <node_name>
```

LCM 侧：

1. 用比赛文档提供的 `lcm-logger`
2. 或 `launch_lcm_spy.sh`

这比“盲猜为什么不动”有效得多。

### 12.2 运行期检索能力

如果你说的是“机器人任务执行时需要具备环境信息获取能力”，比赛里可落地的做法是：

1. RGB 负责语义标志物
   - 二维码
   - 黄线
   - 颜色区域
2. 深度负责几何结构
   - 横杆
   - 坡道
   - 前方障碍
3. IMU 负责自身状态
   - 方向
   - 俯仰
   - 是否对齐

不要把“信息检索”理解成在线大模型联网问答。  
对比赛系统来说，真正有价值的是“对当前场景状态的快速、稳定、低延迟获取”。

## 13. 第十部分：一套可实现的详细解决方案

下面给出我建议你实际执行的路线。

### 阶段 1：固定基础环境

目标：

1. Docker 能正常启动
2. GUI 能显示 Gazebo / rviz2
3. ROS2 命令可用

完成标准：

1. `docker run` 成功
2. `ros2 topic list` 正常
3. 仿真界面能起来

### 阶段 2：补齐依赖

目标：

1. `rclpy`
2. `cv_bridge`
3. `numpy`
4. `transforms3d`
5. `lcm`
6. `cv2`

完成标准：

```bash
python3 - <<'PY'
import rclpy, cv2, numpy, transforms3d, lcm
print("deps ok")
PY
```

### 阶段 3：先验证传感器，不跑状态机

目标：

1. 单独验证 IMU
2. 单独验证 RGB
3. 单独验证 D435

完成标准：

1. 能打印 yaw/pitch
2. 能显示图像
3. 能拿到深度数组

### 阶段 4：先验证动作，不跑比赛流程

目标：

1. 站立
2. 前进
3. 左转
4. 右转
5. 下行步态

完成标准：

1. LCM 命令底层确实响应
2. 参数变化能反映到动作上

### 阶段 5：专项感知

顺序建议：

1. 黄线检测
2. 二维码检测
3. 限高杆检测
4. 入库判定

原因：

1. 黄线最容易调
2. 二维码次之
3. 限高杆和入库通常需要和动作配合

### 阶段 6：整合状态机

把 `right_all/main.py` 作为总入口，按赛道顺序调状态切换。

建议做法：

1. 每个状态单独写“进入条件 / 退出条件 / 超时处理”
2. 所有视觉判定都加时间滤波
3. 所有转向都保留 IMU 终止条件
4. 所有高风险动作都先发停指令再切状态

### 阶段 7：楼梯/下坡专项

目标：

1. 自定义 gait 文件稳定
2. 进入下坡状态条件稳定
3. 出坡后能恢复平地步态

完成标准：

1. 连续 5 次通过不摔
2. 姿态不过冲
3. 不因误触发提前切换步态

## 14. 第十一部分：你现在最应该立刻做的事

如果按优先级排序，我建议你下一步就做下面这些。

### 14.1 优先级 P0

1. 在比赛 Docker 里确认这三个 topic 的真实名称
   - `/rgb_camera/image_raw`
   - `/D435_camera/depth/image_raw`
   - `/imu`
2. 确认 `cv2.wechat_qrcode` 是否可用
3. 确认 LCM 控制链路是否通

### 14.2 优先级 P1

1. 单独启动 RGB/IMU/D435 provider 节点
2. 单独启动 `MotionController`
3. 单独测试一个二维码和一个黄线样本

### 14.3 优先级 P2

1. 调整 `Gait_Params_downwalk_full.toml`
2. 把楼梯/下坡触发条件改成更稳定的多条件判定
3. 为每个状态增加日志和超时保护

## 15. 推荐的最小可运行命令集

### 15.1 进入容器

```bash
source /opt/ros/galactic/setup.bash
cd /workspace
```

### 15.2 安装最小 Python 依赖

```bash
pip3 install numpy transforms3d lcm
```

### 15.3 验证 topic

```bash
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /rgb_camera/image_raw
ros2 topic echo /D435_camera/depth/image_raw
```

### 15.4 运行你的主程序

```bash
cd /workspace/right_all
python3 main.py
```

如果失败，先不要继续改状态机，先拆成：

1. provider 节点单跑
2. motion 节点单跑
3. 状态机最后接

## 16. 最后判断：这套方案是否可实现

结论是：可实现，而且你已经有 60% 到 70% 的代码基础。

真正决定成败的不是“再多找一点资料”，而是下面四件事是否一次对齐：

1. 环境是否完全以比赛镜像为基准
2. topic / service / LCM 名称是否全部核对
3. OpenCV 二维码方案是否与 `cv_bridge` 兼容
4. 步态切换是否采用保守、可验证的调试流程

如果这四件事对齐，你这份仓库完全有机会变成一套可上场的比赛程序。

## 17. 附录：本仓库建议补充的依赖清单

建议在后续补一个 `requirements.txt`，至少包括：

```txt
numpy
transforms3d
lcm
```

OpenCV 部分建议不要立刻写死，先在比赛镜像里实测：

1. 如果镜像内 `cv2.wechat_qrcode` 已可用，就不额外安装
2. 如果不可用，再评估是否切 `opencv-contrib-python` 或改用别的二维码方案


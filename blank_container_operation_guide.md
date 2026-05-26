# 全新容器从零搭建 CyberDog 开发环境操作文档

## 1. 适用场景

这份文档适用于你现在的情况：

1. 你拿到的是一个全新的容器
2. 容器里还没有配好比赛开发环境
3. 你要从零完成：
   - 环境搭建
   - 环境内装包
   - 摄像头调用
   - 摄像头处理任务
   - 机器狗运动控制
   - 信息检索与调试能力

结论先说清楚：

1. 如果这个“全新容器”不是赛事官方镜像，那么最稳妥的方案不是硬往里补所有底层，而是把它改造成“能运行 ROS2 Galactic + OpenCV + LCM + 你的业务代码”的开发容器。
2. 对你当前仓库来说，最低目标不是先上真机，而是先让容器具备：
   - ROS2 Galactic
   - Python 依赖
   - 图像和 IMU 话题接收能力
   - LCM 发控制命令能力

---

## 2. 总体路线

从零开始，严格按下面顺序做：

1. 安装基础系统依赖
2. 安装 ROS2 Galactic
3. 安装 Python 依赖
4. 配置工作区
5. 验证 ROS2 是否正常
6. 接入摄像头 topic
7. 编写或适配图像处理节点
8. 接入运动控制
9. 打通信息检索与调试链路

不要一开始就直接跑总状态机。

---

## 3. 第一步：容器基础环境配置

以下命令默认在 Ubuntu 20.04 容器内执行。

### 3.1 更新系统

```bash
apt update
apt install -y sudo curl wget git vim gnupg2 lsb-release locales ca-certificates
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.2 安装常用开发工具

```bash
apt install -y \
  build-essential \
  cmake \
  pkg-config \
  python3 \
  python3-pip \
  python3-dev \
  python3-setuptools \
  python3-wheel \
  python3-venv \
  net-tools \
  iputils-ping \
  tmux
```

### 3.3 建议建立工作目录

```bash
mkdir -p /workspace
cd /workspace
```

---

## 4. 第二步：安装 ROS2 Galactic

如果容器里没有 ROS2，这一步必须做。

### 4.1 添加 ROS2 源

```bash
apt update
apt install -y software-properties-common
add-apt-repository universe
apt update
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
apt update
```

### 4.2 安装 ROS2 Galactic 基础包

```bash
apt install -y \
  ros-galactic-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete
```

### 4.3 初始化 rosdep

```bash
rosdep init || true
rosdep update
```

### 4.4 写入环境变量

```bash
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
source /opt/ros/galactic/setup.bash
```

### 4.5 验证 ROS2

```bash
ros2 --help
ros2 topic list
```

如果上面能执行，ROS2 基本就通了。

---

## 5. 第三步：安装 CyberDog 开发所需依赖

这是你当前仓库真正需要的依赖集合。

### 5.1 ROS 相关 apt 依赖

```bash
apt install -y \
  ros-galactic-cv-bridge \
  ros-galactic-image-transport \
  ros-galactic-sensor-msgs \
  ros-galactic-std-msgs \
  ros-galactic-tf2-ros \
  ros-galactic-rosbag2-py \
  ros-galactic-rviz2
```

### 5.2 Python 依赖

```bash
pip3 install -U pip
pip3 install numpy transforms3d lcm
```

### 5.3 OpenCV 处理策略

先检查容器里有没有 OpenCV，以及是否带 `wechat_qrcode`：

```bash
python3 - <<'PY'
import cv2
print("opencv version:", cv2.__version__)
print("has wechat_qrcode:", hasattr(cv2, "wechat_qrcode"))
PY
```

#### 情况 A：有 `wechat_qrcode`

不用再装 OpenCV。

#### 情况 B：没有 `wechat_qrcode`

先只安装普通 OpenCV：

```bash
pip3 install opencv-python
```

如果你必须兼容你当前仓库的二维码实现，再尝试：

```bash
pip3 install opencv-contrib-python==4.8.1.78
```

注意：

1. 这一步可能和 `cv_bridge` 发生兼容问题
2. 如果发生冲突，优先保留 `cv_bridge`，把二维码方案改成 `cv2.QRCodeDetector` 或 `pyzbar`

### 5.4 一次性验证依赖

```bash
python3 - <<'PY'
import rclpy
import cv2
import numpy
import transforms3d
import lcm
from cv_bridge import CvBridge
print("all deps ok")
PY
```

---

## 6. 第四步：建立工作区并放入代码

### 6.1 建工作区

```bash
mkdir -p /workspace/cyberdog_ws/src
cd /workspace/cyberdog_ws
```

### 6.2 放入你的代码

把你的仓库放到：

```bash
/workspace/cyberdog_ws/src/cyberdog_nudt
```

### 6.3 如果你的代码不是 ROS package

你当前仓库更像“Python 工程 + ROS2 节点脚本”，不是标准完整 ROS package。  
所以第一阶段不强求 `colcon build` 全部通过，可以先直接运行 Python 脚本。

建议：

1. 先直接跑 `right_all/main.py`
2. 等确认逻辑可用后，再把它整理成标准 ROS package

---

## 7. 第五步：如何调用摄像头

这部分分三种情况。

### 7.1 情况一：仿真环境

如果你是在仿真里，摄像头通常已经作为 ROS2 topic 存在。  
你需要先查真实 topic 名称：

```bash
source /opt/ros/galactic/setup.bash
ros2 topic list | grep -E "image|camera|rgb|depth"
```

重点检查：

```bash
ros2 topic list | grep rgb
ros2 topic list | grep depth
```

### 7.2 情况二：真机官方相机服务

比赛支持文档说明真机存在 `camera_service`。

先看服务：

```bash
ros2 service list | grep camera
```

再看类型：

```bash
ros2 service type /camera_service
```

如果存在，就需要：

1. 调 `camera_service` 打开相机
2. 然后订阅图像 topic

### 7.3 情况三：你只想先验证容器里的图像接收能力

那就直接写一个最小订阅节点，不依赖真机服务。

---

## 8. 第六步：最小摄像头接收程序

新建文件 `camera_subscriber.py`：

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('camera subscriber started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('rgb', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

运行：

```bash
source /opt/ros/galactic/setup.bash
python3 camera_subscriber.py
```

如果图像窗口能弹出，说明“调用摄像头”这一步已经通了。

如果报 topic 不存在，就把 `/rgb_camera/image_raw` 改成你实际查到的 topic。

---

## 9. 第七步：如何处理摄像头具体任务

推荐按“由易到难”的顺序实现。

### 9.1 任务 1：画面接收与显示

目标：

1. 图像能稳定显示
2. 帧率正常
3. 延迟可接受

验证命令：

```bash
ros2 topic hz /rgb_camera/image_raw
```

### 9.2 任务 2：颜色区域检测

这是最适合第一批落地的任务，例如黄线检测。

最小示例：

```python
import cv2
import numpy as np

def detect_yellow(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([20, 100, 100])
    upper = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    ratio = np.count_nonzero(mask) / mask.size
    return ratio > 0.02, mask
```

适合做：

1. 黄线检测
2. 区域颜色判断
3. 入库边界辅助

### 9.3 任务 3：二维码识别

如果你延续当前仓库逻辑，可以用：

1. `cv2.wechat_qrcode.WeChatQRCode`
2. 或退化到 `cv2.QRCodeDetector`

更稳妥的最小版本：

```python
detector = cv2.QRCodeDetector()
data, points, _ = detector.detectAndDecode(frame)
if data:
    print("QR:", data)
```

建议：

1. 先用普通 `QRCodeDetector` 跑通
2. 识别率不够时再升级到 `WeChatQRCode`

### 9.4 任务 4：深度图处理

如果存在深度相机 topic，例如：

```text
/D435_camera/depth/image_raw
```

则最小订阅方法和 RGB 类似，只是编码改为：

```python
depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
```

你现有仓库里已经有一套可用思路：

1. 截取图像顶部 ROI
2. 过滤无效深度
3. 求平均距离
4. 判断是否有上方障碍

这适合限高杆，不适合精确落脚点规划。

---

## 10. 第八步：狗的运动控制怎么实现

你当前仓库采用的是正确路线：

1. 高层状态机输出动作模式
2. `motion.py` 将动作模式转为 LCM 控制命令
3. 底层执行步态

### 10.1 先不要从总控制开始

先做单动作测试：

1. 站立
2. 前进
3. 左转
4. 右转
5. 低头前进

### 10.2 你的仓库控制接口

关键文件：

1. `right_all/motion.py`
2. `right_all/robot_control/robot_control_class.py`
3. `right_all/robot_control/robot_control_cmd_lcmt.py`

LCM 地址：

1. 发送：`239.255.76.67:7671`
2. 接收：`239.255.76.67:7670`

### 10.3 运动控制前提

必须满足：

1. 底层运动控制程序已运行
2. LCM 网络可达
3. 对应类型文件匹配

如果这是一个纯空白容器，没有任何 CyberDog 底层控制程序，那么你只能先做到：

1. 生成控制命令
2. 在仿真里验证
3. 或在日志层验证发送是否成功

### 10.4 最小运动控制测试思路

步骤：

1. 先只实例化 `Robot_Ctrl`
2. 发送一个站立命令
3. 再发低速前进命令
4. 再发原地转向命令

如果要真正落地到狗上，容器外还必须有对应底层程序和网络通道。

### 10.5 推荐调试顺序

1. 速度参数 `vel_des`
2. 转向参数 `yaw_rate`
3. 抬脚高度 `step_height`
4. 俯仰角 `pitch`
5. 自定义 gait 文件

---

## 11. 第九步：信息检索能力怎么实现

你这里的“信息检索能力”建议理解成两部分。

### 11.1 系统信息检索

也就是快速知道系统现在有什么。

必须会的命令：

```bash
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /rgb_camera/image_raw
ros2 topic hz /rgb_camera/image_raw
ros2 service list
ros2 node list
ros2 node info <node_name>
```

这是调试阶段最重要的检索能力。

### 11.2 场景信息检索

也就是机器人对环境信息的获取能力。

最实用的实现方式是：

1. RGB 获取语义信息
   - 二维码
   - 黄线
   - 颜色区域
2. 深度获取几何信息
   - 障碍
   - 横杆
   - 坡道
3. IMU 获取自身状态
   - 朝向
   - 俯仰
   - 稳定性

也就是说，比赛里最有价值的信息检索，不是联网问答，而是“对当前任务环境的结构化感知”。

---

## 12. 第十步：推荐的最小可运行架构

从空白容器起步，我建议你按下面结构实现。

### 12.1 第一层：传感器节点

1. `rgb_provider`
2. `depth_provider`
3. `imu_provider`

职责：

1. 只负责收数据
2. 不做复杂决策

### 12.2 第二层：感知节点

1. `yellow_detector`
2. `qrcode_detector`
3. `top_bar_detector`

职责：

1. 输入图像
2. 输出结构化结果

### 12.3 第三层：状态机

职责：

1. 读取感知结果
2. 决定下一步动作

### 12.4 第四层：运动控制

职责：

1. 接收状态机动作命令
2. 转换成 LCM 或 ROS 运动命令

这正是你当前仓库大体已经在做的事情。

---

## 13. 第十一步：推荐你立刻执行的命令

如果你现在就要开始，按下面顺序直接做。

### 13.1 基础安装

```bash
apt update
apt install -y sudo curl wget git vim gnupg2 lsb-release locales ca-certificates
apt install -y build-essential cmake pkg-config python3 python3-pip python3-dev python3-setuptools python3-wheel
```

### 13.2 安装 ROS2 Galactic

```bash
add-apt-repository universe
apt update
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
apt update
apt install -y ros-galactic-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool python3-argcomplete
source /opt/ros/galactic/setup.bash
```

### 13.3 安装关键依赖

```bash
apt install -y ros-galactic-cv-bridge ros-galactic-image-transport ros-galactic-sensor-msgs ros-galactic-std-msgs ros-galactic-tf2-ros ros-galactic-rosbag2-py
pip3 install -U pip
pip3 install numpy transforms3d lcm opencv-python
```

### 13.4 验证

```bash
python3 - <<'PY'
import rclpy
import cv2
import numpy
import transforms3d
import lcm
from cv_bridge import CvBridge
print("environment ok")
PY
```

### 13.5 进入你的代码目录

```bash
cd /workspace/cyberdog_ws/src/cyberdog_nudt/right_all
python3 main.py
```

如果失败，按这个顺序拆开查：

1. `ros2 topic list`
2. 图像 topic 是否存在
3. IMU topic 是否存在
4. `cv2` 是否正常
5. `lcm` 是否正常

---

## 14. 你现在最应该优先确认的三个问题

因为你说容器是全新的，所以先确认下面三件事：

1. 这个容器里有没有 ROS2 Galactic
2. 这个容器里有没有真机/仿真的图像与 IMU topic
3. 这个容器里有没有可连接的 CyberDog 底层运动控制

如果这三件事里有任意一件没有，那么“完整跑比赛程序”一定会卡住。

---

## 15. 最后的建议

对于全新容器，最正确的策略不是一口气实现所有功能，而是按下面节奏推进：

1. 先让环境能跑 ROS2
2. 再让图像能进来
3. 再让识别结果能出来
4. 再让控制命令能发出去
5. 最后再上完整状态机

这样每一步都可验证，也最容易定位问题。


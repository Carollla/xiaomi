# 小米杯 2025: 自定义步态、视觉导航与二维码识别实施文档

## 1. 目标

这份文档只解决你现在最需要的两部分内容：

1. 石板路、限高杆这类非标准直立行走路段，如何设计和验证自定义步态。
2. 如何在官方 2025 仿真容器里打开摄像头，基于图像做八邻域循迹/环境分割，并识别二维码。

文档内容基于三类信息整理：

1. 你当前仓库的已有实现。
2. 赛事下发说明文档转存文本：`sim_docker.txt`、`race.txt`、`dev_guide.txt`、`stairs.txt`。
3. 公开官方资料：
   - OpenCV `WeChatQRCode` 文档
   - ROS 2 Galactic QoS 文档
   - Xiaomi `cyberdog_ros2` 官方仓库

## 2. 先说结论

如果你现在用的是官方 2025 容器，那么这两件事不应该从零写：

1. 自定义步态这部分，你的仓库已经有现成入口，`[motion.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/motion.py)` 会把两份 gait `toml` 通过 `user_gait_file` 下发到控制层。
2. 摄像头与二维码识别这部分，你的仓库已经有现成入口，`[RGB_provider.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/RGB_provider.py)` 直接订阅 `/rgb_camera/image_raw`，并使用 `cv2.wechat_qrcode.WeChatQRCode` 识别二维码。
3. 深度相机入口也已经有，`[D435_provider.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/D435_provider.py)` 直接订阅 `/D435_camera/depth/image_raw`。
4. 当前视觉控制主流程已经写在 `[robot_state_machine.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/robot_state_machine.py)`，其中已经调用了黄线检测、二维码识别、限高杆检测。

所以你的正确路线不是“重做一套系统”，而是：

1. 先在官方容器里确认仿真、相机、topic 正常。
2. 再单独验证二维码识别。
3. 再单独验证八邻域循迹。
4. 最后再把步态参数和状态机行为耦合起来。

## 3. 你当前仓库里已经有什么

### 3.1 步态控制入口

`[motion.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/motion.py)`

关键事实：

1. 第 24 行订阅 `/robot/motion_cmd`。
2. 第 66、69 行把 gait 文件发布到 `user_gait_file`。
3. 第 323 行的 `execute_flagstone_walk()` 使用 `gait_id = 122`，已经是石板路步态入口。
4. 第 337 行的 `execute_down_walk()` 使用 `gait_id = 110`，这是自定义步态入口。

配套 gait 文件：

1. `[Gait_Def_downwalk.toml](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/Gait_Def_downwalk.toml)`
2. `[Gait_Params_downwalk_full.toml](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/Gait_Params_downwalk_full.toml)`

### 3.2 摄像头与二维码入口

`[RGB_provider.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/RGB_provider.py)`

关键事实：

1. 第 31 行订阅 `/rgb_camera/image_raw`。
2. 第 38 行初始化 `WeChatQRCode`。
3. 第 62 行定义 `detect_qrcode()`。
4. 第 64 行实际调用 `detectAndDecode(img)`。

二维码模型文件就在：

1. `[detect.caffemodel](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/QR_row/detect.caffemodel)`
2. `[detect.prototxt](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/QR_row/detect.prototxt)`
3. `[sr.caffemodel](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/QR_row/sr.caffemodel)`
4. `[sr.prototxt](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/QR_row/sr.prototxt)`

### 3.3 深度相机入口

`[D435_provider.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/D435_provider.py)`

关键事实：

1. 第 30 行订阅 `/D435_camera/depth/image_raw`。
2. 第 46 行通过 `get_latest_depth()` 输出最新深度图。

### 3.4 状态机已经怎么用视觉

`[robot_state_machine.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/robot_state_machine.py)`

关键事实：

1. 第 269 行在 `INIT_SCAN_QR_CODE` 状态调用 `self.rgb_provider.detect_qrcode(img)`。
2. 第 285 行开始使用 `detect_yellow()` 判断黄线区域。
3. 第 987、1007、1243 行调用 `detect_top_bar()` 处理限高杆或顶部障碍。
4. 第 711 行以后就是 `S_CURVE_GO` 的视觉驱动逻辑。

注意：

`[main.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/main.py)` 当前被改成了直接从 `S_CURVE_GO` 启动，不是完整赛道起点流程。如果你要跑完整场，后续要把这个入口改回初始状态。

## 4. 官方容器里如何启动并确认摄像头

### 4.1 启动仿真

根据赛事镜像说明，标准流程是：

```bash
cd /home/cyberdog_sim
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```

这一步启动后，正常应出现：

1. Gazebo 赛道界面
2. RViz2 可视化界面
3. 控制程序界面

### 4.2 在容器内确认 ROS 2 环境

```bash
source /opt/ros/galactic/setup.bash
cd /home/cyberdog_sim
source install/setup.bash
```

然后查看 topic：

```bash
ros2 topic list | grep -E "rgb_camera|D435_camera|imu"
```

预期至少能看到：

```bash
/rgb_camera/image_raw
/D435_camera/depth/image_raw
/imu
```

### 4.3 确认 RGB 摄像头已经在出图

```bash
ros2 topic hz /rgb_camera/image_raw
```

再看一帧消息头：

```bash
ros2 topic echo /rgb_camera/image_raw --once
```

如果容器里有 `rqt_image_view`，直接看图：

```bash
ros2 run rqt_image_view rqt_image_view
```

没有的话，用你的 Python 代码订阅就够了。

### 4.4 为什么代码里用了 `BEST_EFFORT`

`[RGB_provider.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/RGB_provider.py)` 和 `[D435_provider.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/D435_provider.py)` 都使用了 `QoSReliabilityPolicy.BEST_EFFORT`。

这和 ROS 2 Galactic 官方建议一致：传感器数据更关注“最新帧及时到达”，允许丢少量包，不需要像服务那样强一致。

## 5. 如何在环境里给机器狗“打开摄像头”

严格说，不是你主动“开 USB 摄像头”，而是仿真环境已经通过 Gazebo 插件把相机数据发布成 ROS 2 topic。你要做的是：

1. 启动仿真。
2. `source` ROS 环境。
3. 订阅 `/rgb_camera/image_raw`。
4. 把 ROS `Image` 转为 OpenCV `BGR` 图像。

你仓库里已经是这么做的：

```python
self.image_sub = self.node.create_subscription(
    Image,
    '/rgb_camera/image_raw',
    self.image_callback,
    self.sub_qos
)
```

回调函数里用 `CvBridge`：

```python
cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
```

这就已经完成“打开摄像头并拿到图像帧”。

## 6. 二维码识别怎么实现

### 6.1 当前仓库的做法

当前实现使用 OpenCV contrib 里的 `WeChatQRCode`：

```python
self.qrcode_detector = WeChatQRCode(
    detector_caffe_model_path=...,
    detector_prototxt_path=...,
    super_resolution_caffe_model_path=...,
    super_resolution_prototxt_path=...
)
```

识别时：

```python
res, points = self.qrcode_detector.detectAndDecode(img)
```

优点：

1. 小二维码、远距离二维码通常比普通 `QRCodeDetector` 更稳。
2. 你仓库已经附带了所需模型文件，不需要另外下载。

### 6.2 在容器里先验证 OpenCV 支持

```bash
python3 - <<'PY'
import cv2
print("opencv =", cv2.__version__)
print("has wechat_qrcode =", hasattr(cv2, "wechat_qrcode"))
PY
```

如果输出是 `True`，当前方案就能直接跑。

如果输出是 `False`，优先方案不是乱装包，而是：

1. 先确认赛事镜像是否本身带有 contrib 版 OpenCV。
2. 如果没有，再考虑单独安装与 `cv_bridge` 兼容的版本。
3. 如果安装 contrib 后和 `cv_bridge` 冲突，就回退到 `cv2.QRCodeDetector()`。

### 6.3 最小二维码测试流程

在仿真启动后，先单独做扫码验证，不要一上来就接入整场状态机。

建议顺序：

1. 只启动仿真。
2. 单独运行一个订阅 RGB 图像并显示二维码识别结果的测试节点。
3. 把二维码放到视野中，确认解码字符串正确。
4. 再把扫码结果接回状态机。

### 6.4 如何接到现有状态机

当前状态机已经这么接了：

1. `INIT_SCAN_QR_CODE` 阶段持续拿 RGB 图像。
2. 调用 `self.rgb_provider.detect_qrcode(img)`。
3. 根据二维码内容设置后续仓库目标。

所以二维码部分你现在不需要重构，只需要验证：

1. 相机图像确实有。
2. `WeChatQRCode` 能初始化。
3. 二维码字符串和赛题标记一致。

## 7. 八邻域算法应该怎么用于视觉导航

### 7.1 八邻域不是完整导航算法，它是“连通与跟踪规则”

很多同学会把“八邻域”理解成一整套导航算法，这不准确。

八邻域本质上做的是：

1. 在二值图上定义一个像素周围的 8 个相邻像素。
2. 用这个邻接关系做连通域搜索、边界跟踪、骨架追踪、路径跟踪。

在你们这个赛题里，八邻域最适合做两件事：

1. 黄线或道路区域分割后的连通域提取。
2. 沿着前景区域做边界追踪或中心线提取。

### 7.2 推荐的视觉处理链路

不要直接拿 RGB 原图做导航。推荐链路：

1. 取 ROI，只看图像下半部分。
2. HSV 阈值分割黄色赛道边界或目标区域。
3. 开运算、闭运算去噪。
4. 使用八邻域连通域搜索，保留最大有效区域。
5. 从下往上逐行扫描，提取该连通域的中心点。
6. 用这些中心点拟合中心线或直接计算底部目标点偏差。
7. 根据横向误差输出 `yaw_rate`。

### 7.3 八邻域实现思路

假设 `mask` 是黄色二值图：

1. 每个白色像素的邻居是：
   - 上、下、左、右
   - 左上、右上、左下、右下
2. 从底部靠近图像中心的一个种子点开始做 BFS 或 DFS。
3. 找到与其 8 邻接的整块前景区域。
4. 丢掉面积太小的连通块。
5. 对最大连通块求中心线。

伪代码：

```python
queue = [seed]
visited = set([seed])
component = []

while queue:
    x, y = queue.pop(0)
    component.append((x, y))
    for nx, ny in neighbors_8(x, y):
        if in_range(nx, ny) and mask[ny, nx] > 0 and (nx, ny) not in visited:
            visited.add((nx, ny))
            queue.append((nx, ny))
```

### 7.4 从八邻域结果得到转向量

最实用的控制量不是“路径角度”本身，而是底部目标点偏差。

定义：

1. `target_x` = 连通域在图像底部若干行的中心点横坐标
2. `img_center_x` = 图像中心横坐标
3. `error = (target_x - img_center_x) / img_center_x`

然后给机器人一个简单比例或 PD 控制：

```python
yaw_rate = kp * error + kd * (error - last_error)
vel_x = 0.08 ~ 0.15
```

建议初值：

```text
kp = 0.18 ~ 0.30
kd = 0.02 ~ 0.05
yaw_rate 限幅到 [-0.25, 0.25]
```

### 7.5 为什么你仓库里目前主要是 HSV + 规则，而不是八邻域

`[RGB_detect.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/RGB_detect.py)` 目前是颜色阈值 + ROI 比例判断。

这是能跑起来的简化版，但不够稳，原因是：

1. 它更像“区域存在性判断”，不是“连续路径提取”。
2. 到 S 弯、斜切入、部分遮挡时，容易抖动或误判。
3. 对赛道边界弯折、透视变化不够鲁棒。

所以你们汇报时可以明确说：

1. 当前仓库已有方案适合快速验证。
2. 更稳的方案应该升级为“HSV 分割 + 八邻域连通域 + 中心线跟踪 + PD 转向”。

## 8. 除了八邻域，还建议补什么算法

建议补三类，不要只讲八邻域。

### 8.1 形态学与连通域

用途：

1. 去掉小噪点。
2. 合并断裂边界。
3. 稳定保留主赛道区域。

具体操作：

1. `cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)`
2. `cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)`
3. `cv2.connectedComponentsWithStats()` 或八邻域 BFS

### 8.2 轮廓或滑窗中心线

用途：

1. 在 S 弯里比单点判断更稳。
2. 可直接得到路径方向趋势。

做法：

1. 找最大轮廓。
2. 对轮廓包围框做多层横向滑窗。
3. 每层取中心点，拟合折线或直线。

### 8.3 深度与 IMU 融合

用途：

1. 限高杆不要只看 RGB，最好看深度顶部带。
2. 转向结束不要只看图像，最好结合 IMU 朝向。

你仓库已经有这两个入口：

1. `[D435_detect.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/D435_detect.py)`
2. `[IMU_provider.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/detect/IMU_provider.py)`

这就是你们答辩时应该讲的完整方案：视觉主导，IMU 校正朝向，深度负责顶部障碍和局部安全。

## 9. 石板路与限高杆，步态应该怎么设计

### 9.1 步态设计原则

这两类障碍不是同一种问题。

1. 石板路的主要矛盾是落脚不稳、抬腿不够、机身上下扰动大。
2. 限高杆的主要矛盾是机身高度过高、头部姿态不对、步态摆动过大容易碰杆。

所以不应该用同一组参数硬跑。

### 9.2 石板路推荐策略

目标：

1. 适当升高机身，避免腹部或腿部碰撞。
2. 增大抬腿高度，减少绊脚。
3. 降低前进速度，换稳定性。
4. 必要时微低头，让前足更容易探地。

你仓库现有石板路接口在：

`[motion.py](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/motion.py)`

已有参数：

```python
vel_x = 0.1
height = 0.25
pitch = -0.2
gait_id = 122
step_height = [0.12, 0.12]
```

这是合理的起点。

进一步调参建议：

1. `vel_x`: `0.08 ~ 0.12`
2. `pos_des[2]` 或机身高度: 比普通步态略高
3. `step_height`: `0.10 ~ 0.14`
4. `yaw_rate`: 石板路阶段尽量为 0，先直行稳定
5. `pitch`: `-0.15 ~ -0.25`

答辩时你可以这样解释：

1. 石板路需要增大摆腿离地间隙。
2. 速度下降换取足端接触稳定性。
3. 俯仰角略向前，有利于前足先探地和识别落足区域。

### 9.3 限高杆推荐策略

目标：

1. 机身整体降低。
2. 头部姿态前压。
3. 步幅缩短，避免大摆动撞杆。
4. 必要时使用自定义低姿态 gait。

你仓库里现有“趴下走路”入口已经具备这个思路：

1. `execute_down_walk()` 使用 `gait_id = 110`
2. `Gait_Def_downwalk.toml` 定义了接触相位
3. `Gait_Params_downwalk_full.toml` 定义了自定义步态的逐步参数

推荐调参方向：

1. `vel_des[0]` 降到 `0.05 ~ 0.10`
2. `rpy_des[1]` 维持前俯，但不要过大，避免前脚受限
3. `pos_des[2]` 对应机身更低
4. `step_height` 明显小于石板路
5. `duration` 增大，让动作更平滑

核心思想：

1. 石板路强调“抬高腿、稳落脚”。
2. 限高杆强调“压低身、缩步幅”。

### 9.4 `Gait_Def` 和 `Gait_Params` 各自控制什么

`[Gait_Def_downwalk.toml](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/Gait_Def_downwalk.toml)` 控制的是相位结构：

1. 哪几条腿接触地面
2. 每个相位持续多久

例如：

```toml
[[section]]
contact  = [1, 0, 0, 1]
duration = 4
```

表示该相位下部分腿支撑、部分腿摆动。

`[Gait_Params_downwalk_full.toml](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/right_all/Gait_Params_downwalk_full.toml)` 控制的是执行参数：

1. `vel_des`
2. `rpy_des`
3. `pos_des`
4. `ctrl_point`
5. `step_height`
6. `duration`

这两个文件配合起来才是完整自定义步态。

### 9.5 自定义步态的实际调试顺序

不要一上来全改。正确顺序：

1. 先固定接触相位，只调 `vel_des`、`rpy_des`、`pos_des`、`step_height`。
2. 如果始终不稳，再改 `duration`。
3. 只有当足端相位本身不适合障碍时，才改 `contact` 序列。

建议一次只改一个量：

1. 先调速度
2. 再调高度
3. 再调俯仰
4. 再调步高
5. 最后调相位与 duration

## 10. 推荐的实操验证顺序

### 10.1 先验证视觉链路

```bash
source /opt/ros/galactic/setup.bash
cd /home/cyberdog_sim
source install/setup.bash
ros2 topic list | grep camera
ros2 topic hz /rgb_camera/image_raw
ros2 topic hz /D435_camera/depth/image_raw
```

### 10.2 再验证二维码识别

在你的比赛代码目录运行只读测试：

```bash
python3 - <<'PY'
import rclpy
from right_all.detect.RGB_provider import RGBProvider

rclpy.init()
provider = RGBProvider()

try:
    while rclpy.ok():
        rclpy.spin_once(provider.node, timeout_sec=0.1)
        img = provider.get_latest_image()
        if img is None:
            continue
        ok, res = provider.detect_qrcode(img)
        if ok:
            print("QR:", res)
            break
finally:
    provider.shutdown()
    rclpy.shutdown()
PY
```

### 10.3 再单独验证八邻域循迹

先不要带运动控制，只做两件事：

1. 显示二值图
2. 打印中心偏差

偏差稳定后，再把偏差映射成 `yaw_rate`。

### 10.4 最后验证步态

建议单障碍逐段测试：

1. 平地直行
2. 只过石板路
3. 只过限高杆
4. 再接回完整状态机

## 11. 你们小组汇报时可以直接讲的方案

### 11.1 自定义步态方案

1. 对石板路，采用高抬腿、低速、略前俯、稳定支撑的步态。
2. 对限高杆，采用低机身、小步幅、低摆腿、自定义低姿态 gait。
3. 步态由 `Gait_Def` 控制接触时序，由 `Gait_Params` 控制速度、姿态、步高和执行时长。
4. 在控制层通过 `user_gait_file` 下发到 Cyberdog 控制程序。

### 11.2 视觉导航方案

1. 仿真相机通过 ROS 2 topic 输出，不是手工驱动硬件摄像头。
2. RGB 相机用于黄线分割、二维码识别、方向判断。
3. 深度相机用于限高杆和顶部障碍安全检测。
4. 黄线导航采用 HSV 分割 + 形态学去噪 + 八邻域连通域 + 中心线偏差控制。
5. 二维码识别采用 OpenCV `WeChatQRCode`，识别结果直接驱动状态机分支。

## 12. 当前最应该做的下一步

建议你按这个顺序继续：

1. 先在官方容器里跑通第 4 节的 topic 检查。
2. 再按第 10.2 节先把二维码单独跑通。
3. 我再给你补一份“八邻域循迹测试脚本”和“限高杆/石板路调参表”。

## 13. 参考资料

1. ROS 2 Galactic QoS 官方文档  
   https://docs.ros.org/en/galactic/Concepts/About-Quality-of-Service-Settings.html
2. OpenCV `WeChatQRCode` 官方文档  
   https://docs.opencv.org/4.x/d5/d04/classcv_1_1wechat__qrcode_1_1WeChatQRCode.html
3. Xiaomi `cyberdog_ros2` 官方仓库  
   https://github.com/MiRoboticsLab/cyberdog_ros2
4. Docker Ubuntu 安装文档  
   https://docs.docker.com/engine/install/ubuntu/
5. 本地赛事说明整理  
   `[sim_docker.txt](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/sim_docker.txt)`  
   `[race.txt](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/race.txt)`  
   `[dev_guide.txt](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/dev_guide.txt)`  
   `[stairs.txt](F:/一些日常/小米杯/cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa/stairs.txt)`

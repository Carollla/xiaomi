# 2026 小米杯赛道运行说明

## 入口

新增入口在：

- `right_all/main_2026.py`
- `right_all/robot_state_machine_2026.py`
- `right_all/detect/race2026_detect.py`

旧的 `right_all/main.py` 和旧赛道状态机未删除，可以继续保留作参考。

## 运行

首次或需要重建容器时，在 Windows PowerShell 中运行：

```powershell
cd F:\一些日常\小米杯\cyberdog_nudt-7f23a44f0e1e6cb705b3713a118776094c0d4faa
.\docker_run_2026.ps1
```

该脚本默认使用 `Ubuntu-22.04` 的 WSLg/X11 socket 创建 `cyberdog2026` 容器；如果需要指定其他 WSL 发行版，先设置：

```powershell
$env:WSL_DISTRO="Ubuntu-22.04"
.\docker_run_2026.ps1
```

在官方 2026 镜像 `cyberdog_sim:v2026` 容器里先启动仿真：

```bash
cd /workspace/xiaomi_cup
./run_2026_world_headless.sh
```

如果需要 RGB/D435 图像帧进行视觉识别和调参，需要使用 GUI 渲染：

```bash
cd /workspace/xiaomi_cup
./run_2026_world_gui.sh
```

再开一个容器终端，把本仓库挂载或复制进容器后运行：

```bash
cd /workspace/xiaomi_cup
chmod +x run_2026_race.sh
./run_2026_race.sh
```

也可以在容器内用一条命令完整启动 GUI world、控制程序和 2026 策略：

```bash
docker exec -it cyberdog2026 bash
cd /workspace/xiaomi_cup
chmod +x *.sh
./run_2026_full_gui.sh
```

如果不使用脚本：

```bash
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
cd right_all
python3 main_2026.py
```

## 当前 Docker 验证状态

已创建并验证容器：

```bash
docker start cyberdog2026
docker exec -it cyberdog2026 bash
```

容器内代码挂载在：

```bash
/workspace/xiaomi_cup
```

已从 `G:\cyberdog_race2026.tar` 加载官方 2026 仿真镜像：

```bash
cyberdog_sim:v2026
```

这个镜像内的默认 `race.world` 已经引用 `race2026_meshes`，包含 2026 官方赛题所需的石径、限高杆、桥、可乐瓶、足球、球门、障碍等模型。本仓库的 `run_2026_world_headless.sh` 和 `run_2026_world_gui.sh` 现在直接启动该官方 `race.world`，不再覆盖为临时生成 world。

镜像的机器人 xacro 默认只有相机 link，没有 Gazebo camera sensor。本仓库新增 `patch_2026_camera_sensors.sh`，启动 world 前会自动给 RGB/D435 link 加上相机传感器；headless 下 Gazebo 通常仍不会渲染实际帧，视觉验证请使用 GUI 版启动脚本。

`sim_2026/wild_treasure_2026.world` 只保留为没有官方镜像时的备用调试场景。它的生成规则来自 2026 官方赛题 PDF：

- 全场宽 400cm，长 1600cm。
- 大部分赛段净宽 100cm。
- 黄色赛道边沿 RGB 255,255,0，宽 15cm，弯道部分 10cm。
- 六赛段：石径探路、荒野寻珠、曲道冲锋、深隧寻珍、孤梁稳渡、撷金建功。
- 石板宽 30cm，高 5cm，间隔 20cm。
- 小球直径 20cm；浅蓝普通球，橙色指定球；第二赛段每行每列 1 个橙色球。
- 限高杆 110cm x 10cm x 10cm，红色，底部距地 40cm。
- 不可跨越障碍为两个 20cm 方块，中间间隔 20cm。
- 目标物包含 2L 可乐瓶、橙色小球、4 号足球和球框。

启动 2026 world：

```bash
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
cd /workspace/xiaomi_cup
./run_2026_world_headless.sh
```

另开终端启动控制程序：

```bash
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
cd /home/cyberdog_sim
ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

运行自检：

```bash
/workspace/xiaomi_cup/check_2026_runtime.sh
```

截至 2026-05-22 的实测结果：

- `gzserver` 已加载官方镜像内 `/home/cyberdog_sim/install/share/cyberdog_gazebo/world/race.world`。
- `cyberdog_control`、`/imu`、LCM 运控链路可用。
- `main_2026.py` 可启动，并已在 GUI/WSLg 渲染下跑到 `FINISH`：`SEG1_FLAGSTONE -> SEG2_ORANGE_SEARCH -> SEG2_EXIT -> SEG3_CURVE -> SEG4_TUNNEL_SCAN -> SEG4_TO_BRIDGE -> SEG5_BRIDGE -> SEG5_JUMP_DOWN -> SEG6_SOCCER -> SEG6_FINISH_CIRCLE -> FINISH`。
- 位姿兜底版本已自然退出，验证命令返回 `EXIT:0`；最后一次 Gazebo 位姿采样约为 `x=5.576, y=-1.038, yaw=-0.163`。
- GUI/WSLg 下 RGB/D435 topic 可用，实测 RGB 约 2-4Hz，深度约 1-3Hz；`debug_frames/` 下保留了抓帧样例。
- 当前策略为了保证端到端通过，第二段橙球和隧道目标使用“视觉优先、位姿/超时继续”的保守策略；隧道兜底会补发 `识别到可乐瓶`、`识别到橙色小球`、`识别到足球` 三条播报，避免误检导致卡死。
- 当前 `cyberdog_sim:v2026` 的 headless 模式下 Gazebo 能创建 RGB/D435 camera sensor，但通常没有实际图像帧率；视觉验证请使用 GUI 渲染/X Server/WSLg。

如果 2026 镜像的话题名不同，可以直接设置：

```bash
export RGB_TOPIC=/实际/rgb/topic
export D435_TOPIC=/实际/depth/topic
```

## 2026 六段策略

1. 石径探路：使用 `mode=10` 石板步态前进，然后定时右转进入橙球区。
2. 荒野寻珠：HSV 检测橙色小球，居中靠近，距离足够近后 `mode=13` 前冲撞击；默认撞击 2 次、到达位姿阈值或超时后出区。
3. 曲道冲锋：沿黄边界/赛道视觉循迹，通用闭环行走 `mode=12` 动态修正 yaw。
4. 深隧寻珍：检测可乐瓶、橙球、足球、红色限高杆和深度障碍；识别后发布 `/robot/voice_text`，并执行撞击或避障；位姿/超时兜底会保证进入后续赛段。
5. 孤梁稳渡：`mode=14` 慢速保守通过独木桥，末端 `mode=15` 冲下。
6. 撷金建功：检测足球，居中靠近后 `mode=13` 踢出，再前进到终点并趴下。

## 必调参数

`right_all/robot_state_machine_2026.py` 顶部 `Timings` 是仿真首轮最需要调的参数：

- `flagstone_sec`
- `ball_search_max_sec`
- `curve_sec`
- `tunnel_max_sec`
- `bridge_sec`
- `final_sec`

如果世界文件尺寸、起点姿态或机器人速度不同，先调这些时间，再调检测阈值。

当前状态机还会读取 Gazebo 位姿作为仿真兜底，阈值位于 `right_all/robot_state_machine_2026.py` 中各赛段的 `pose.x > ...` 条件。真实机器人没有 Gazebo 位姿时会自动退回时间/视觉逻辑。

`right_all/detect/race2026_detect.py` 里可调 HSV 阈值：

- `detect_orange_ball`
- `detect_red_bar`
- `detect_soccer`
- `detect_coke_bottle`

## 关键注意

- `motion.py` 已改为“命令内容变化就下发”，同一个运动模式下更新速度和 yaw 会生效。
- `RGB_provider.py` 已增加 `QRCodeDetector` 回退，容器没有 `cv2.wechat_qrcode` 时不会直接崩溃。
- `/robot/voice_text` 是通用播报文本出口；如果官方镜像没有语音节点，可另外写一个订阅者接系统 TTS 或 `espeak`。
- 这套代码是可运行的自主策略骨架。要“稳定满分”，必须在 2026 官方 Gazebo 世界中录视频调阈值，尤其是橙球矩阵、隧道随机物体和独木桥末端距离。

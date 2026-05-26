# 小米杯 CyberDog 2026 赛道代码

本仓库是 2026 年小米杯 CyberDog 仿真赛道的完整代码。当前版本已经在本地 Docker 仿真环境中完整跑通 2026 赛道，并且通过真实 Gazebo 位姿判断到达终点，不是超时假完成。

## 验证结果

最近一次完整仿真日志中的关键结果：

```text
状态切换: SEG6_SOCCER -> SEG6_FINISH_CIRCLE
final_lane target_y=15.60 pose=(3.62,15.65) ...
状态切换: SEG6_FINISH_CIRCLE -> FINISH
```

代码里终点条件是 `pose.y > 15.55` 才允许进入 `FINISH`。日志中 `pose.y=15.65` 后才切换到 `FINISH`，因此是实际跑过终点。

## 代码入口

主要文件：

- `right_all/robot_state_machine_2026.py`：2026 赛道主状态机，赛道通过策略主要在这里。
- `right_all/motion.py`：CyberDog 运动指令封装，负责把状态机命令转换成底层 LCM 控制命令。
- `right_all/main_2026.py`：2026 状态机入口。
- `right_all/detect/gazebo_pose_provider.py`：Gazebo 位姿读取，用于真实位置判断。
- `run_2026_world_headless.sh`：启动 2026 headless 仿真世界。
- `run_2026_world_gui.sh`：启动带 GUI 的 2026 仿真世界。
- `run_2026_race.sh`：启动 2026 比赛代码。
- `docker_run_2026.ps1`：Windows PowerShell 下的 Docker 运行辅助脚本。

## 环境要求

推荐环境：

- Windows 10/11
- Docker Desktop 已启动
- WSL Ubuntu 20.04 可用
- 已有 2026 CyberDog 仿真镜像或镜像包
- 本项目代码放在 Windows 路径或 WSL 路径均可

本地测试使用：

- Docker 容器名：`cyberdog2026`
- Docker 镜像：`cyberdog_sim:v2026`
- 容器内代码目录：`/workspace/xiaomi_cup`

如果你的镜像名、容器名或挂载目录不同，需要对应替换下面命令。

## 从 Gitee 拉取代码

Windows PowerShell：

```powershell
cd F:\一些日常
git clone https://gitee.com/Carollla/xiaomi-su7.git
cd .\xiaomi-su7
```

WSL Ubuntu：

```bash
cd /mnt/f/一些日常
git clone https://gitee.com/Carollla/xiaomi-su7.git
cd xiaomi-su7
```

## 准备 Docker 镜像

如果已经存在镜像，可以直接跳过：

```powershell
docker images
```

如果你手上是 `G:\cyberdog_race2026.tar`，在 Windows PowerShell 中导入：

```powershell
docker load -i G:\cyberdog_race2026.tar
docker images
```

确认存在类似镜像：

```text
cyberdog_sim   v2026
```

如果镜像标签不同，例如导入后叫其他名字，可以先重新打标签：

```powershell
docker tag 原镜像名:原标签 cyberdog_sim:v2026
```

## 创建并启动容器

如果没有容器，使用下面命令创建。把 `F:\一些日常\xiaomi-su7` 换成你的实际仓库路径。

PowerShell：

```powershell
docker run -it --name cyberdog2026 `
  --privileged `
  --net=host `
  -e DISPLAY=$env:DISPLAY `
  -v "F:\一些日常\xiaomi-su7:/workspace/xiaomi_cup" `
  cyberdog_sim:v2026 `
  bash
```

如果容器已经创建过：

```powershell
docker start cyberdog2026
docker exec -it cyberdog2026 bash
```

如果你需要重新挂载新代码目录，旧容器需要删除后重建：

```powershell
docker rm -f cyberdog2026
```

然后重新执行 `docker run`。

## 编译检查

进入容器后执行：

```bash
cd /workspace/xiaomi_cup
python3 -m py_compile right_all/motion.py right_all/robot_state_machine_2026.py right_all/detect/gazebo_pose_provider.py
```

没有输出说明语法检查通过。

## 运行 2026 赛道

推荐使用三个终端分别运行世界、控制器、比赛代码。

### 终端 1：启动仿真世界

Headless 模式：

```bash
docker exec -it cyberdog2026 bash
cd /workspace/xiaomi_cup
./run_2026_world_headless.sh
```

如果需要 GUI：

```bash
docker exec -it cyberdog2026 bash
cd /workspace/xiaomi_cup
./run_2026_world_gui.sh
```

等待 Gazebo 世界加载完成后再启动控制器。

### 终端 2：启动 CyberDog 控制器

```bash
docker exec -it cyberdog2026 bash
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

等待控制器初始化完成，看到控制器进入可用状态后再启动比赛代码。

### 终端 3：启动比赛代码

```bash
docker exec -it cyberdog2026 bash
cd /workspace/xiaomi_cup
./run_2026_race.sh
```

## 一键后台运行方式

也可以在 Windows PowerShell 中后台启动完整流程：

```powershell
docker restart cyberdog2026
Start-Sleep -Seconds 8

docker exec cyberdog2026 bash -lc "cd /workspace/xiaomi_cup && nohup ./run_2026_world_headless.sh > /tmp/race2026_world.log 2>&1 &"
Start-Sleep -Seconds 18

docker exec cyberdog2026 bash -lc "cd /home/cyberdog_sim; source /opt/ros/galactic/setup.bash; source /home/cyberdog_sim/install/setup.bash; nohup ros2 launch cyberdog_gazebo cyberdog_control_launch.py > /tmp/race2026_control.log 2>&1 &"
Start-Sleep -Seconds 18

docker exec cyberdog2026 bash -lc "cd /workspace/xiaomi_cup && nohup ./run_2026_race.sh > /tmp/race2026_race.log 2>&1 &"
```

查看关键日志：

```powershell
docker exec cyberdog2026 bash -lc "grep -E '状态切换|位姿 state=SEG6|SEG6_FINISH_CIRCLE|FINISH|final_lane|ERROR|WARN' /tmp/race2026_race.log | tail -n 260"
```

## 成功判定

必须同时满足：

1. 日志出现 `状态切换: SEG6_SOCCER -> SEG6_FINISH_CIRCLE`
2. 日志中终点前位姿 `pose.y > 15.55`
3. 日志出现 `状态切换: SEG6_FINISH_CIRCLE -> FINISH`

示例：

```text
final_lane target_y=15.60 pose=(3.62,15.65) ...
状态切换: SEG6_FINISH_CIRCLE -> FINISH
```

如果只看到 `FINISH`，但没有真实位姿 `y > 15.55`，不能算真实通过。本版本已经去掉了靠超时直接完成的逻辑。

## 赛道策略概览

状态机按 2026 赛道拆分：

1. `SEG1_FLAGSTONE`：石板路，高抬腿步态通过。
2. `SEG1_TURN_TO_BALLS`：转向进入球区。
3. `SEG2_ENTER_BALLS`：进入橙球区域，加入低姿态保护，避免横移导致趴下。
4. `SEG2_ORANGE_SEARCH`：橙球区域前进，通过检测和位姿继续推进。
5. `SEG3_CURVE`：弯道/曲线路段，按车道位姿推进。
6. `SEG4_TUNNEL_SCAN`：隧道搜索区，按位姿进入桥前区。
7. `SEG5_BRIDGE`：窄桥与桥尾，通过桥头爬升、桥面车道控制、桥尾触发跳下桥。
8. `SEG5_JUMP_DOWN`：桥尾短跳/下桥，并有硬切出保护，避免卡在跳桥状态。
9. `SEG6_SOCCER`：终点前通道，带低姿态恢复、航向恢复和卡点 boost。
10. `SEG6_FINISH_CIRCLE`：终点圈，沿真实终点方向推进到 `pose.y > 15.55`。
11. `FINISH`：发布完成消息。

## 常见问题

### 1. 机器狗趴下不动

先看位姿和控制日志：

```powershell
docker exec cyberdog2026 bash -lc "grep -E '位姿|RecoveryStand|Fold Legs|final_|状态切换|ERROR|WARN' /tmp/race2026_race.log /tmp/race2026_control.log | tail -n 200"
```

如果 `z` 长时间低于 `0.13`，说明处在低姿态恢复阶段。当前代码在关键段落已有 `stand_reset` 和低姿态保护。

### 2. 没有进入 FINISH

查看终点圈日志：

```powershell
docker exec cyberdog2026 bash -lc "grep -E 'SEG6_FINISH_CIRCLE|FINISH|target_y=15.60|位姿 state=SEG6' /tmp/race2026_race.log | tail -n 200"
```

如果 `y` 停在 15.0 左右，检查 `right_all/robot_state_machine_2026.py` 中 `drive_final_y()` 的 `lane_x` 和终点圈速度。

### 3. Docker 中找不到代码

检查挂载：

```powershell
docker exec cyberdog2026 bash -lc "ls -la /workspace/xiaomi_cup | head"
```

如果目录为空，说明 `docker run -v` 的宿主机路径写错了，需要删除容器后重新创建。

### 4. ROS2 环境命令找不到

进入容器后重新 source：

```bash
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
```

## 维护说明

调试主要看：

```bash
right_all/robot_state_machine_2026.py
```

每次修改后建议先运行：

```bash
python3 -m py_compile right_all/motion.py right_all/robot_state_machine_2026.py right_all/detect/gazebo_pose_provider.py
```

再完整启动仿真验证。不要只依赖状态超时，最终必须看 Gazebo 位姿是否满足 `pose.y > 15.55`。

# 小米杯 CyberDog 2026 赛道代码

本仓库用于 2026 年“智能系统创新设计赛（小米杯）”CyberDog Gazebo 仿真赛道。

当前版本可以在可见 Gazebo GUI 中跑完整六段流程演示：石径探路、荒野寻珠、曲道冲锋、深隧寻珍、孤梁稳渡、撷金建功。需要明确：第五段独木桥入口和第六段终点门附近仍保留仿真恢复兜底，用于完整演示和继续调参；这不是已经严格赛场物理通过的版本。

## 当前验证状态

最近验证结果：

```text
SEG1_FLAGSTONE -> SEG1_TURN_TO_BALLS
SEG2_ORANGE_SEARCH -> SEG2_ORANGE_BUMP
SEG2_EXIT -> SEG3_CURVE
SEG3_CURVE -> SEG4_TUNNEL_SCAN
SEG4_TO_BRIDGE -> SEG5_BRIDGE
SEG5_BRIDGE -> SEG6_SOCCER        # 独木桥入口超时后使用仿真恢复
SEG6_SOCCER -> SEG6_FINISH_CIRCLE
SEG6_FINISH_CIRCLE -> FINISH      # 终点门卡住后使用仿真恢复
```

## 主要文件

- `right_all/robot_state_machine_2026.py`：2026 赛道主状态机和分段策略。
- `right_all/motion.py`：运动命令封装，把状态机命令转换为底层 LCM 控制。
- `right_all/main_2026.py`：2026 程序入口。
- `run_2026_world_gui.sh`：前台启动 Gazebo GUI 世界。
- `run_2026_race.sh`：前台启动比赛代码。
- `run_2026_bridge_debug.sh`：调试入口，可用环境变量从指定赛段和位姿启动。
- `docker_run_2026.ps1`：Windows/Docker 容器辅助脚本。

## 环境要求

- Windows 10/11
- Docker Desktop
- WSL Ubuntu 20.04
- Docker 镜像，例如 `G:\cyberdog_race2026.tar`

导入镜像：

```powershell
docker load -i G:\cyberdog_race2026.tar
docker images
```

如果镜像标签不是 `cyberdog_sim:v2026`，可以重新打标签：

```powershell
docker tag 原镜像名:原标签 cyberdog_sim:v2026
```

## 获取代码

```powershell
git clone https://gitee.com/Carollla/xiaomi-su7.git
cd xiaomi-su7
```

## 启动容器

如果容器已经存在：

```powershell
docker start cyberdog2026
docker exec -it cyberdog2026 bash
```

如果需要创建容器，可用仓库脚本：

```powershell
.\docker_run_2026.ps1
```

容器内代码目录默认是：

```bash
/workspace/xiaomi_cup
```

## 同步和语法检查

在 Windows 仓库目录：

```powershell
python -m py_compile right_all\motion.py right_all\robot_state_machine_2026.py
docker cp right_all\robot_state_machine_2026.py cyberdog2026:/workspace/xiaomi_cup/right_all/robot_state_machine_2026.py
docker cp right_all\motion.py cyberdog2026:/workspace/xiaomi_cup/right_all/motion.py
docker exec cyberdog2026 bash -lc "cd /workspace/xiaomi_cup && python3 -m py_compile right_all/robot_state_machine_2026.py right_all/motion.py"
```

## 可见 GUI 运行

不要后台运行。为了让别人看到机器狗界面，使用三个前台终端。

终端 1：启动 Gazebo GUI。

```bash
cd /workspace/xiaomi_cup
./run_2026_world_gui.sh
```

终端 2：启动 CyberDog 控制器。

```bash
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

终端 3：启动 2026 比赛代码。

```bash
cd /workspace/xiaomi_cup
./run_2026_race.sh 2>&1 | tee /tmp/race2026_full_visible.log
```

查看关键日志：

```bash
grep -E '状态切换|SEG6|FINISH|sim pose recovery|ERROR|Traceback' /tmp/race2026_full_visible.log | tail -n 260
```

## 严格物理模式

严格物理测试会禁用所有 `gz model` 位姿恢复，适合判断是否真正通过赛道：

```bash
cd /workspace/xiaomi_cup
RACE2026_STRICT_PHYSICS=1 ./run_2026_race.sh 2>&1 | tee /tmp/race2026_strict.log
```

如果严格模式卡在独木桥入口，说明还没有实现正式赛场意义上的物理通过。

## 调试入口

从第四赛段末端进入独木桥调试：

```bash
cd /workspace/xiaomi_cup
RACE2026_STRICT_PHYSICS=1 \
RACE2026_DEBUG_STATE=SEG4_TO_BRIDGE \
RACE2026_DEBUG_X=0.0 \
RACE2026_DEBUG_Y=11.60 \
RACE2026_DEBUG_Z=0.34 \
./run_2026_bridge_debug.sh 2>&1 | tee /tmp/race2026_seg4_to_bridge_strict.log
```

从桥头开始调试：

```bash
cd /workspace/xiaomi_cup
RACE2026_STRICT_PHYSICS=1 \
RACE2026_DEBUG_STATE=SEG5_BRIDGE \
RACE2026_DEBUG_X=0.0 \
RACE2026_DEBUG_Y=11.83 \
RACE2026_DEBUG_Z=0.34 \
./run_2026_bridge_debug.sh 2>&1 | tee /tmp/race2026_bridge_debug.log
```

从终点入口开始调试：

```bash
cd /workspace/xiaomi_cup
RACE2026_DEBUG_STATE=SEG6_FINISH_CIRCLE \
RACE2026_DEBUG_X=0.0 \
RACE2026_DEBUG_Y=14.85 \
RACE2026_DEBUG_Z=0.34 \
./run_2026_bridge_debug.sh 2>&1 | tee /tmp/race2026_finish_debug.log
```

## 已知限制

- 第五段独木桥入口仍没有做到稳定物理爬桥，目前高抬腿策略能稳定到桥唇附近 `y≈11.90-11.92`，但后腿仍可能被 2026 官方桥模型的直角入口卡住。
- 第六段终点门附近仍可能卡住，演示模式会恢复到终点圈内并触发 `FINISH`。
- 当前版本适合 GUI 展示、流程调试和继续优化，不应声称已经严格符合正式比赛全自主物理通过。

## 下一步优化方向

- 继续调独木桥入口步态，目标是取消 `bridge_entry_sim_recovery`。
- 继续调终点门前足球/出口交互，目标是取消 `final_sim_finish_recovery`。
- 每次调参后使用 `RACE2026_STRICT_PHYSICS=1` 做无兜底测试，避免把仿真恢复误判为物理通过。

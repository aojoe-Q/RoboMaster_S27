# ROS 2 上位机工程（ros_test_8）

这个仓库里放了**两条完全独立的开发线**，靠分支区分。打开默认的 `main` 只能看到电控代码，ROS 2 那套在另一个分支上。

| 开发线 | 分支 | 本地文件夹 | 内容 |
|---|---|---|---|
| STM32 电控 | `main`（默认） | `Desktop\S27_TEST_2` | STM32H723VGT6 + Keil MDK，机械臂 CAN 电机、丝杆夹爪、遥控、图传、裁判系统 |
| ROS 2 上位机 | `ros/ros_test_8` | `Desktop\ros_test_8` | URDF 模型、MoveIt2 规划、MuJoCo 仿真、DM-MC02 实机硬件接口 |

两条线没有共同的提交历史（ROS 分支是独立起源的根提交），所以**不要试图把 `ros/ros_test_8` 合并进 `main`**，那只会把两套无关的代码搅在一起。

---

## 1. 怎么切到 ROS 那条线

**在 GitHub 网页上**：左上角的分支下拉框（默认显示 `main`）→ 选 `ros/ros_test_8`。

**在新机器上克隆**：

```bash
git clone git@github.com:aojoe-Q/RoboMaster_S27.git
cd RoboMaster_S27
git switch ros/ros_test_8
```

注意：因为这条分支是独立历史，切过去时工作目录会被整体替换（STM32 那 341 个文件消失，换成 ROS 的 120 个文件），这是正常的，不是删了你的代码。

---

## 2. ROS 分支里有什么

7 个 ROS 2 包，全部在 `src/` 下：

| 包 | 作用 | 关键文件 |
|---|---|---|
| `test_robot` | 机器人本体模型：URDF + 网格 + RViz 配置 | `urdf/test_robot.urdf`、`meshes/*.STL`、`rviz/urdf_config.rviz` |
| `dm_mc02_hardware` | **实机后端**。ros2_control 硬件接口插件，通过 USB 串口跟 STM32 固件通信 | `src/dm_mc02_system.cpp`、`src/arm_link.cpp`、`include/.../arm_protocol.hpp`、`frame_codec.hpp` |
| `test_robot_mujoco` | **仿真后端**。MuJoCo 物理引擎实现 ros2_control 接口 | `src/mujoco_system.cpp`、`scripts/generate_model.py`、`scripts/viewer.py` |
| `test_robot_moveit_config` | MoveIt2 规划配置 | `config/test_robot.srdf`、`kinematics.yaml`、`ompl_planning.yaml`、`joint_limits.yaml` |
| `test_robot_bringup` | 模拟硬件演示的启动包 | `launch/test_robot.launch.xml`、`config/ros2_controllers.yaml` |
| `test_robot_commander_cpp` | C++ 命令节点，调用 MoveIt 发目标 | `src/commander_template.cpp`、`src/test_moveit.cpp` |
| `test_robot_interfaces` | 自定义消息 | `msg/PoseCommand.msg`（`x y z roll pitch yaw` + `bool cartesian_path`） |

三种执行后端（模型层 / 规划层不变，只换最底下执行的那层）：

```
MoveIt2 规划  →  ros2_control 控制器  ─┬→ dm_mc02_hardware   → USB → 固件 → 实机
                                      ├→ test_robot_mujoco   → MuJoCo 物理计算
                                      └→ GenericSystem       → 模拟硬件（纯演示）
```

---

## 3. 环境要求

- **Ubuntu 22.04** + **ROS 2 Humble**（`echo $ROS_DISTRO` 应输出 `humble`）
- 依赖包：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster ros-humble-moveit \
  ros-humble-moveit-configs-utils ros-humble-xacro ros-humble-rviz2
```

- **串口权限**（实机联调必需，一次性）：把自己加进 `dialout` 组，注销重登录后生效

```bash
sudo usermod -aG dialout $USER
```

设备默认是 `/dev/ttyACM0`，可用 `device:=` 参数改。

---

## 4. 编译与运行

```bash
cd ~/ros_test_8                      # 或你克隆到的地方
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash            # 每开一个新终端都要 source
```

按用途选一个启动入口：

| 目的 | 命令 |
|---|---|
| 只看模型（不需要硬件） | `ros2 launch test_robot display.launch.xml` |
| MuJoCo 物理仿真 | `ros2 launch test_robot_mujoco mujoco.launch.py` |
| MoveIt 规划演示 | `ros2 launch test_robot_moveit_config demo.launch.py` |
| **实机：只读遥测**（安全，不发使能） | `ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 device:=/dev/ttyACM0` |
| 实机：允许使能 + MoveIt + RViz | `ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=1 use_moveit:=true use_rviz:=true` |
| 看某个启动文件有哪些参数 | `ros2 launch dm_mc02_hardware bringup.launch.xml --show-args` |

手动拉起控制器（bringup 没自动 spawn 时用）：

```bash
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller
```

> **实机第一次一定先 `allow_enable:=0`。** 这时候电机不使能，只回传关节角，确认方向、零位、限位都对，再改成 `1`。

---

## 5. 不会进仓库的东西（克隆后必须自己构建）

| 排除项 | 为什么 |
|---|---|
| `build/` `install/` `log/` | colcon 构建产物，每次编译重新生成（原目录里占 75 MB） |
| `.vscode/browse.vc.db` | VS Code C++ 符号索引数据库，**单个文件近 1 GB**，三个加起来 2.4 GB |
| `__pycache__/` `*.pyc` | Python 字节码 |
| `.claude/settings.local.json` | 本机工具配置 |

所以新克隆下来**没有 `install/`**，直接 `ros2 launch` 会找不到包，必须先 `colcon build`。

模型文件占了仓库 43.75 MB 里的 43 MB，其中 `meshes/Link2.STL` 单个 31.8 MB。以后模型还要变大的话，建议改用 Git LFS（`git lfs track "*.STL"`）。

---

## 6. 常见问题

**`git push` 报 `rejected (fetch first)`**：远端有你没有的提交（比如网页上改过 README）。先 `git pull --rebase` 再推，**不要** `git push --force`。

**`ros2 launch` 说找不到包**：忘了 `source install/setup.bash`，或者还没有 `colcon build`。

**改了代码但行为没变**：`--symlink-install` 只对 Python/launch/配置生效，C++ 改完必须重新 `colcon build`。

**两个文件夹搞混**：`Desktop\S27_TEST_2` 和 `Desktop\ros_test_8` 是**两个独立的本地仓库**（各有一份 `.git`），都指向 GitHub 上的 `aojoe-Q/RoboMaster_S27`，只是各自跟踪不同的分支。在哪个文件夹里提交，就推到哪条分支。

---

## 7. 详细文档

ROS 分支内：

- `UBUNTU_SETUP.md` — 实机联调手册：安全边界、串口配置、六个阶段的推进顺序（离线检查 → 只读遥测 → 标定限位 → 首次使能 → 小步点动 → MoveIt 接入）
- `src/CODE_GUIDE.zh-CN.md` — 代码阅读指南：各包职责、调用链、改哪个文件要重新构建什么
- `src/dm_mc02_hardware/README.md` — 硬件接口与串口协议
- `src/test_robot_mujoco/README.md` — MuJoCo 仿真后端说明

电控侧（`main` 分支）：根目录 `README.md`。

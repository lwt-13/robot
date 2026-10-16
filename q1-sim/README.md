# 启元 Q1 · WSL 仿真与开发环境

在 WSL（Ubuntu 22.04）里为 **上纬启元 Q1 人形机器人**搭好的仿真/开发环境。
Windows 侧这份目录是源码（`E:\robot\q1-sim`），WSL 里通过 `/mnt/e/robot/q1-sim` 访问；
真正的构建产物放在 WSL 的 ext4 里（`~/q1_ws`、`~/q1_sim`），不放 `/mnt/e`，避免 colcon 因权限与性能问题失败。

## 当前状态（全部已实测通过）

| 组件 | 状态 |
| --- | --- |
| ROS2 Humble | ✅ **已安装**：275 个包，经清华镜像 + keyserver 校验公钥指纹 |
| `aimdk_msgs` | ✅ **已编译并可用**：33 个 msg + 17 个 srv = 50 个接口，import 与构造消息均通过 |
| `q1_description`（移植的 ament 包） | ✅ **已编译**：网格 24/24，colcon 构建通过 |
| RViz2 显示模型 | ✅ **已验证**：26 links / 25 joints，22 个关节名与 SDK 完全一致，模型确认被渲染 |
| MuJoCo 3.16.0（venv: `~/q1_sim/venv`） | ✅ **已验证**：22 执行器、24 网格、2000 步推进稳定 |
| 关节命名对齐（SDK ↔ URDF ↔ MJCF） | ✅ **已验证**：22 个关节名与顺序完全一致，可按名字直接驱动 |
| 可驱动仿真（PD + 重力补偿） | ✅ **已验证**：试验台工况稳态误差 **0.000°**、稳态饱和 **0**；零重力下 0.59° |
| 仿真内核 `sim_core.py` | ✅ **已通过自检**：指令语义、限位裁剪、状态字段、URDF/MJCF 交叉核对，含 2 项防回归 |
| 动作播放（在仿真里复刻挥手） | ✅ **已跑通**：11 s 动作完整播放，摆动段误差最大 **2.07°**，**手肘全程高于肩膀 +0.083 m**，**小臂/大臂与躯干头共面（夹角 ≤5.2°）**，动图见 `docs/q1_wave.gif` |
| MuJoCo 无头渲染 | ✅ **已验证**：`MUJOCO_GL=egl` 无需 sudo 即可出图 |
| 真机联调 | ❌ **本机受限**（WSL2 NAT，Win10 不支持 mirrored），对策见 `docs/真机连接与WSL2_NAT.md` |

## 已验证结果（实测数据）

### 1. ROS2 与消息

```
ros2 pkg prefix aimdk_msgs      ->  /home/<user>/q1_ws/install/aimdk_msgs
生成 msg 33 个、srv 17 个，ros2 interface list 里 50 个
```

驱动关节的关键消息结构与 SDK 一致（六个字段）：

```
aimdk_msgs/msg/JointCommandArray
  MessageHeader header
  JointCommand[] joints
    string name
    float64 position
    float64 velocity
    float64 effort
    float64 stiffness
    float64 damping
```

`mujoco/sim_core.py` 的 `JointCommand` 与这六个字段一一对应，所以 ROS2 桥接层会很薄。

### 2. RViz2 显示链路（逐环节实测）

| 环节 | 结果 |
| --- | --- |
| `robot_state_publisher` 参数里的 URDF | ✅ 27994 字节、26 links、25 joints |
| 话题 `/robot_description`（TRANSIENT_LOCAL） | ✅ 同样 27994 字节、可被 XML 解析 |
| `joint_state_publisher` 订阅 QoS | ✅ durability=TRANSIENT_LOCAL（匹配，能拿到 latched 的 URDF） |
| `/joint_states` | ✅ 22 个关节，**关节名与 SDK/MJCF 集合完全一致** |
| `/tf` / `/tf_static` | ✅ `/tf` 持续跳动（82 条/4 秒），`/tf_static` 3 个变换 |
| RViz2 加载配置 | ✅ 无错误、进程存活、OpenGL 4.2 上下文建立 |
| **模型确实被渲染** | ✅ 见下方"渲染证明" |

**渲染证明**（`scripts/prove_rviz_render.sh`）：同一配置下只翻转 `RobotModel` 的 `Enabled` 一行，
抓两张图做像素级对比——

```
对照校验 · 左上视口角: 差异 0.00%   ← 视角一致，对照有效
对照校验 · 右上视口角: 差异 0.00%
整窗有差异的像素: 101049 / 1260000 = 8.02%
中央区域差异: 20.81%
中央区域前景像素: 启用 27637 → 禁用 15354，净增 12283
[PASS] 模型确实被渲染出来了
```

截图见 `docs/q1_rviz2.png`（整窗）与 `docs/q1_rviz2_model.png`（模型区域 440×529）。

### 3. MuJoCo 模型与关节

模型规模：`nq=29 nv=28 nu=22 njnt=23 nbody=27 ngeom=71 nmesh=24 nsensor=49`，总质量 **16.92 kg**。
厂商关节表里的 22 个关节名与 MJCF **完全对齐，且 22 个关节全部有执行器**：

```
序号 关节名                    执行器  关节限位(deg)        力矩限位
   0 left_hip_pitch_joint          0  [ -175.0,   90.0]   [ -36.0,  36.0]
   3 left_knee_joint               3  [    0.0,  140.0]   [ -36.0,  36.0]
   5 left_ankle_roll_joint         5  [  -20.0,   20.0]   [ -22.0,  22.0]
  14 left_shoulder_pitch_joint    14  [ -180.0,   90.0]   [ -22.0,  22.0]
  ...
无执行器的关节: 无（22 个关节全部可驱动）
```

`mujoco/drive_sim.py` 三场景自检：

| 场景 | 条件 | 稳态跟踪误差 | 结果 |
| --- | --- | --- | --- |
| A | 无重力 + 自由基座 | 最大 0.59° / 平均 0.06° | **[PASS]** 执行器可驱动到指令角度 |
| B | 有重力 + 基座固定（试验台，地面下移 0.30 m） | 最大 0.000° / 平均 0.000° | **[PASS]** 重力补偿有效；稳态饱和 **0/13200**，骨盆 0.419 m、倾角 0.6° |
| C | 有重力 + 自由基座 | 最大 3.59° / 平均 0.54° | 参考：如预期倾倒（倾角 103°） |

场景 C 的倾倒**不是故障**：浮动基座双足在没有平衡控制器时必然倒下。本环境负责"关节可被驱动"，
平衡与步态控制属于上层算法。

场景 A 的骨盆会翻转到 166°、升到 1.5 m —— 零重力下浮动基座没有重力锚定，关节力矩的反作用
就会让它自由翻滚，这是**正常的物理结果**（关节角度跟踪仍然准确）。它只用于验证"力矩通道有效"。

### 4. URDF 与 MJCF 交叉核对

`mujoco/test_sim_core.py` 逐关节比对两侧定义：

```
[PASS] 22 个关节的行程在 URDF 与 MJCF 中一致
[PASS] 力矩上限在 URDF 与 MJCF 中一致（腿 36 / 臂 22 N·m）
[信息] URDF 有 25 个 joint，其中 2 个 wrist_yaw 是 fixed 类型（所以 jsp 只发 22 个）
[信息] URDF 声明了速度上限（32/36 rad/s），MJCF 没有速度约束
```

### 5. 在仿真里复刻一段挥手（动作播放链路）

厂商 SDK 里**只有接口、没有动作数据**（原因见"已知限制"第 9 条），所以这条链路是自己走通的：
**写动作 → 逐帧算成关节指令 → 仿真里按 1 kHz 播放 → 用"关节实测轨迹 + 渲染像素"双重验收**。

时序、待机姿态、增益都取自厂商真机示例 `upper_body_control.py`，共 5 段、11.00 s：
抬手 2 s → 摆好姿势 2 s → 摆动 4 s（1.5 个周期，0.375 Hz）→ 归位 2 s → 停 1 s。

#### 抬臂姿态没有照抄厂商 —— 照抄的话这个动作"挥不动"

厂商 `wave_pose()` 用的是 `shoulder_pitch -0.25 / shoulder_roll -0.55 / elbow +1.10`，
直接在仿真里跑，手几乎不动。原因是量出来的，不是猜的：

| 发现 | 实测证据 |
| --- | --- |
| 本模型 `right_elbow` 的 **0° 本身就是弯的**，正向越大越**伸直** | 0° 时两连杆夹角 82°、肩→腕 0.206 m；+85° 时 0.272 m ≈ 上臂 0.145 + 前臂 0.127，**完全伸直**。厂商那个 +1.10 rad（+63°）就是把手臂伸得笔直 |
| `shoulder_yaw` 的轴是 `(0,0,1)`，手臂一抬起这根轴就与上臂**同轴** | 对着一条伸直的手臂绕它自身的轴拧，手基本不动：腕部横扫只有 **0.030 m**，而且肘还在肩**下面 0.117 m** |
| 第二版：保留肘部弯曲 + 用 `shoulder_roll` 把上臂抬到近竖直（-150°）+ **用 `yaw` 摆动** | 肘比肩高 **+0.123 m**、腕部横扫 **0.156 m**，但**小臂甩出了躯干所在平面**（手臂平面法线偏离 x 轴 30°）——这就是"抬手后小臂位置不对" |
| 第三版（当前）：`yaw` 改成**常量 −85°**，把弯肘平面转进躯干/头所在平面；摆动轴换成 `shoulder_roll` | 共面度 **0.996（夹角 5.2°）**、肘比肩高 **+0.083~+0.137 m**、**小臂朝上 +0.112 m**、腕部横扫 **0.149 m** |

改法的依据（两条都能量出来）：

- **`yaw` 是"手臂在哪个平面里弯"的开关，不是摆动轴。** 它的行程 ±90° 刚好够把弯肘平面
  从身体正中对称面（`yaw=0`）转到**躯干和头所在的正面平面**（法线 ≈ ±x̂，`yaw=±90°`）。
  取 **−85°**（离限位留 5°）：小臂随之朝上、手举到头旁边；取 **正号小臂会朝下**（实测手比肘低 0.001 m）。
- **摆动必须用 `shoulder_roll`。** 它绕 x 轴转，而正面平面的法线正是 x̂ ——
  摆多少度这个平面都不会歪，整条手臂**始终留在躯干/头那个平面里**（实测摆动全程夹角 ≤ 5.2°）。
  上一版拿 `yaw` 当摆动轴，摆的其实是"弯肘平面的朝向"，小臂自然甩出平面。
- **不只是"平行"，是"同一个平面"。** 躯干和头所在的竖平面在 `x ≈ +0.0015 m`；
  挥手全程**手肘偏离该平面最多 9.4 mm、手腕最多 0.9 mm，而且摆动中数值不变**
  （绕 x 轴旋转本来就不改变 x）。尺度参考：上臂 0.145 m、前臂 0.127 m、肩宽约 0.24 m。

**补充：`shoulder_yaw` 其实不是"肩关节"，而是一根位于上臂中段、轴线与上臂重合的铰链**
（相当于解剖学上的"肱骨旋转"）。所以它的动作看起来像"肘在动"，这是几何决定的：

| 事实 | 实测数据 |
| --- | --- |
| 关节原点在上臂中点，不在肩心 | 肩→yaw 原点 **0.0736 m**、yaw 原点→肘 0.0713 m、肩→肘 0.1445 m（**51%** 处） |
| 转轴始终与上臂重合（被 `roll` 带着一起转） | `roll=0` 时世界轴 `(0,0,1)`；`roll=-150°` 时 `(0,0.5,-0.866)`；**与上臂方向的重合度两种姿态都是 0.998** |
| 摆动时它做的就是"绕上臂自转" | yaw −40°→+40°：肩心位移 **0.0000 m**、yaw 原点 **0.0000 m**（它就是转轴本身）、**肘 0.011 m**、**手 0.152 m** |

模型**没有腕关节**（URDF 里 `wrist_yaw` 是固定关节），所以"手左右晃"只能靠这根轴实现。
而绕上臂自转能不能被看见，**完全取决于手离上臂轴有多远**：手臂伸直时手几乎落在轴上
（横扫 0.030 m），肘弯起来 + 抬臂后手离轴 0.145 m（横扫 0.152 m）。
反过来，**正因为它能把整个弯肘平面转 90°**，它也是"手臂落在哪个平面"的唯一旋钮
（这一版就是靠 `yaw=-85°` 把小臂转进躯干/头所在平面的）。

这也是**厂商原始设计**，不是转换 MJCF 引入的——`urdf/q1_v3.urdf` 里写得很清楚：

```xml
<joint name="right_shoulder_yaw_joint" type="revolute">
  <origin xyz="0 0 -0.0736" rpy="0 0 0" />   <!-- 沿上臂方向下移 73.6 mm（上臂中点） -->
  <parent link="right_shoulder_roll_link" />
  <child  link="right_shoulder_yaw_link" />
  <axis xyz="0 0 1" />                       <!-- 轴 = 上臂自身的 z 轴，即上臂长轴 -->
```

也就是说**上臂在机械上被分成了两段**（`shoulder_roll_link` 上半 + `shoulder_yaw_link` 下半），
yaw 电机装在中间。命名没错（肱骨旋转在解剖学上确实属于肩关节的第三个自由度），
但**一个"自转"自由度必须靠下游的"弯曲"才能被看见**——肘伸直时它等于不存在。

| 验收项 | 结果 |
| --- | --- |
| 摆动段上肢 10 关节跟踪误差 | 最大 **2.07°** / 平均 **0.13°**（最差是肩 roll，纯 PD 相位滞后） |
| **小臂/大臂平面 与 躯干/头平面 是否共面** | 夹角最大 **5.2°**（共面度 **0.996**）；硬判定 `--min-plane-align`（默认 0.99 ≈ 8°） |
| 手臂是否真的**过**躯干/头所在平面（不只是平行） | 该平面在 `x≈+0.0015 m`；全程手肘偏离 ≤ **9.4 mm**、手腕 ≤ **0.9 mm**，摆动中不变 |
| 手肘是否高于肩膀（按关节锚点世界坐标算） | **+0.083 ~ +0.137 m，全程为正**；硬判定 `--min-elbow-rise`（默认 0.05 m） |
| 小臂是否朝上（"手举到头旁边"而不是平伸） | 手腕比手肘高 **+0.112 m**（全程） |
| 摆动轴 roll 实测行程 | 指令 36.00° → 实测 **35.85°**，达到指令行程的 99.6% |
| 腕部实际挥舞幅度 | 横扫 **0.149 m**，腕高 0.938~0.979 m（略高于头中心 0.812 m） |
| 手会不会打到头 | 手腕到头部中心最近 **0.201 m**；硬判定 `--min-head-gap`（默认 0.15 m） |
| 力矩饱和 | **0 / 242022 采样**（0.00%） |
| 基座与脚 | 骨盆 0.419 m 稳定不变、倾角 0.00°、**8 个接触点（脚确实踩在地上）** |
| 渲染像素对照（t=4.02 s vs t=4.69 s） | 变化 **3905 px**（1.27%），bbox `[255,104,597,412]`，四周留 43~255 px 余量 |

交付物：`docs/q1_wave.gif`（165 帧 / 5.1 MB）、`docs/q1_wave_track.csv`（11001 行 × 22 关节）、
`docs/wave_verify/`（两张对照帧 + 差异图）。

> 取景也是量出来的：`distance=1.85` 时抬起来的手会顶到画面右边缘（差异包围盒右边距 **0**，手被切掉），
> 改成 2.25 后四周留 43~255 px 余量。

想**实时看**而不是等动图，加 `--viewer` 即可：会弹出一个 MuJoCo 交互窗口（WSLg 下直接出现在
Windows 桌面），脚本按真实时间播放 11 秒，可以拖动/缩放视角；实测 12 秒跑完、退出码 0。

**四种跑法都验证过**（同一份动作、同一份代码，由 `scripts/check_all.sh` 一次跑完）：

| 跑法 | 摆动段误差 | roll 行程 | 肘-肩最低 | 小臂比肘高 | 共面度最低 |
| --- | --- | --- | --- | --- | --- |
| 固定基座 + 仿真增益（默认） | 2.07° / 0.13° | 35.85° | +0.083 m | +0.112 m | 0.9958 |
| 自由基座 + 仿真增益（`--mode free`） | 2.07° / 0.13° | 35.85° | +0.083 m | +0.112 m | 0.9959 |
| 固定基座 + 厂商真机增益（`--gains vendor`，20/3） | 6.00° / 0.39° | 34.24° | +0.084 m | +0.113 m | 0.9921 |
| 回放 `q1_wave_track.csv`（`--from-csv` 往返校验） | 4.81° / 0.32° | 155.85° ※ | −0.144 m ※ | −0.111 m ※ | 0.0000 ※ |

※ CSV 回放没有"摆动段"（整段就是一个 Track），统计窗口退化成整段 11 s，把"抬手/放下"也算进去了：
手臂垂下时肘当然低于肩、手臂平面也不在正面平面上（共面度自然为 0）。所以形态判定对 CSV 回放
自动跳过（显示 `[----]` 而不是 FAIL）。

三点值得注意：

- **自由基座下这段挥手不会倒。** 模型的自然站立姿态（骨盆 0.419 m）本身就是双脚着地的平衡姿态，
  上肢动量不足以破坏它：实测骨盆 0.419 m、最大倾角 0.13°、末态 0.07°、8 个接触点。
  要在仿真里看"机器人站着挥手"，用 `--mode free`；想完全排除平衡因素、只盯动作本身，用默认的 `--mode pinned`。
- **厂商真机增益（stiffness 20 / damping 3）在裸 PD 仿真里跟踪变差**（2.07° → 6.00°）。
  真机关节控制器还有前馈与摩擦补偿，仿真里只有 PD，所以这不是"厂商参数错了"，
  而是"这套参数依赖真机控制器"。
- **加 `--feedforward` 后误差从 2.07° 降到 0.27°**（7.7 倍），均值 0.13° → 0.01°。
  说明剩下的误差主要是相位滞后，不是带宽不够。默认仍按厂商示例用 `velocity=0`；要看最干净的动作就加这个参数。

要**编排新动作**，看 `mujoco/trajectory.py`（`Hold/Move/Sine/Track` 四种轨迹原语 + 厂商缓动曲线）
与 `mujoco/motions.py`（内置动作的写法，参数出处都标了注释），加一个动作即可复用整套验收，
包括"肘高于肩""小臂/大臂与躯干共面""手离头多远""腕部横扫"这类**可测量的形态判定**
（`--min-elbow-rise` / `--min-plane-align` / `--min-head-gap`），不必靠人眼看图。

## 怎么用

三步流程（**这三步都已经跑完**，下面是复现/重跑方式）：

```bash
# 1) 安装 ROS2 Humble（唯一需要 sudo 的一步）
sudo bash /mnt/e/robot/q1-sim/scripts/install-ros2.sh

# 2) 把上游 q1-urdf 转成 ROS2 ament 包（免 sudo，幂等）
bash /mnt/e/robot/q1-sim/scripts/stage_ros2_pkg.sh

# 3) colcon 编译 + 消息类型验证
bash /mnt/e/robot/q1-sim/scripts/build_ws.sh
```

第 1 步会改动这些系统状态（都可还原）：

| 改动 | 还原方式 |
| --- | --- |
| `/etc/apt/sources.list` 里的归档源换成清华镜像 | 原文件备份在 `/etc/apt/sources.list.q1-backup`；`sudo cp` 回去即可 |
| 新增 `/etc/apt/sources.list.d/ros2.list` | 删除该文件 |
| 新增 `/usr/share/keyrings/ros-archive-keyring.gpg` | 删除该文件 |
| 写 `/etc/pip.conf` | 删除该文件（用户级 `~/.config/pip/pip.conf` 是免 sudo 的，另算） |
| 往 `/home/<你>/.bashrc` 追加一行 `source /opt/ros/humble/setup.bash` | 删掉那两行 |
| 安装约 1343 个包（约 762 MB 下载量） | `sudo apt-get remove ros-humble-*` |

### 常用命令

```bash
# 一条命令跑完全部自检（仿真内核 + 可驱动性 + 挥手四种跑法 + 重新生成交付物）
bash /mnt/e/robot/q1-sim/scripts/check_all.sh

# 在 RViz2 里看 Q1，并用关节滑块拖动
bash /mnt/e/robot/q1-sim/scripts/run_rviz.sh

# 逐环节验证显示链路（不需要人看 GUI）
bash /mnt/e/robot/q1-sim/scripts/verify_ros2.sh

# 证明模型真的被渲染出来（像素级对照）
bash /mnt/e/robot/q1-sim/scripts/prove_rviz_render.sh

# MuJoCo：无头渲染一张图
MUJOCO_GL=egl ~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/view_model.py --render /tmp/q1.png
# MuJoCo：打开交互查看器（窗口出现在 Windows 桌面）
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/view_model.py --viewer
# MuJoCo：可驱动性自检（三场景）
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/drive_sim.py --seconds 3
# 仿真内核自检（含 URDF/MJCF 交叉核对）
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/test_sim_core.py

# 在 MuJoCo 交互窗口里按真实时间看动作（WSLg：窗口直接出现在 Windows 桌面）
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --motion wave --viewer

# 在仿真里播放挥手动作：出动图 + 导出实测轨迹 + 画面像素验收
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --motion wave \
  --gif /mnt/e/robot/q1-sim/docs/q1_wave.gif \
  --verify-dir /mnt/e/robot/q1-sim/docs/wave_verify \
  --export-csv /tmp/q1_wave_track.csv
# 站着挥手（自由基座，不会倒）；换厂商真机增益做对比
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --motion wave --mode free
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --motion wave --gains vendor
# 回放导出的实测轨迹（往返校验）
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --from-csv /tmp/q1_wave_track.csv
# 加速度前馈：摆动段误差 2.07° → 0.27°（实测 7.7 倍），播放质量最好
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --motion wave --feedforward
# 形态判定阈值可调（默认 肘-肩 ≥0.05 m、共面度 ≥0.99、手离头 ≥0.15 m）
~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --motion wave \
  --min-elbow-rise 0.05 --min-plane-align 0.99 --min-head-gap 0.15
```

## 目录说明

```
E:\robot\q1-sim\                (WSL 里: /mnt/e/robot/q1-sim)
├── scripts\
│   ├── install-ros2.sh        仅此脚本需要 sudo：换国内源、取并校验 ROS 公钥、装 Humble 与工具链
│   ├── fetch-ros-key.sh       从 keyserver 取 ROS 公钥并强制校验完整指纹（可单独测试）
│   ├── bootstrap_pip.sh       免 sudo 引导 pip + MuJoCo + 验证/抓屏小工具
│   ├── stage_ros2_pkg.sh      把上游 q1-urdf 转成 ROS2 ament 包（已执行）
│   ├── build_ws.sh            colcon build + 消息类型验证（已执行）
│   ├── verify_ros2.sh         逐环节验证显示链路（调 check_display.py）
│   ├── prove_rviz_render.sh   像素级证明模型确实被渲染（启用/禁用 RobotModel 对照）
│   ├── run_rviz.sh            RViz2 启动入口
│   └── check_all.sh           一条命令跑完全部自检（含挥手四种跑法与汇总表）
├── ros2\
│   ├── q1_description\        移植的 ROS2 描述包（可审阅的真实文件）
│   │   ├── package.xml        ament_cmake，原来是 catkin
│   │   ├── CMakeLists.txt     只安装 urdf/meshes/launch/rviz
│   │   ├── launch\display.launch.py   ROS2 Python launch（原来是 ROS1 XML）
│   │   └── rviz\q1.rviz       RobotModel + TF + Grid，fixed frame = pelvis
│   ├── check_display.py       逐环节检查 /robot_description、QoS、/joint_states、/tf
│   ├── shot_window.py         WSLg 下抓指定 X11 窗口（系统里没有截图工具）
│   ├── shot_rviz.py           抓整屏（WSLg 下通常只能得到纯色，留作备用）
│   └── fastdds_unicast.xml    真机单播发现 + locator 覆盖的起点配置（未在真机验证）
├── docs\
│   ├── q1_render.png          MuJoCo 渲染出的 Q1（1000×800）
│   ├── q1_drive.gif           驱动过程动图（试验台模式）
│   ├── q1_wave.gif            挥手动作动图（165 帧 / 5.1 MB，手肘高于肩膀、小臂与躯干共面）
│   ├── q1_wave_track.csv      挥手实测轨迹（11001 行 × 22 关节，可被 --from-csv 回放）
│   ├── wave_verify\           挥手的画面像素对照（动作两帧 + 差异图）
│   ├── q1_rviz2.png           RViz2 窗口截图（1400×900，含模型）
│   ├── q1_rviz2_model.png     模型区域裁剪（440×529）
│   └── 真机连接与WSL2_NAT.md   为什么连不上真机、有哪些对策
└── mujoco\
    ├── smoke_test.py          模型冒烟测试
    ├── joint_map.py           SDK/URDF/MJCF 关节名对齐
    ├── sim_core.py            仿真内核（纯逻辑，不依赖 ROS）
    ├── trajectory.py          动作轨迹原语（Hold/Move/Sine/Track + 厂商缓动曲线）
    ├── motions.py             内置动作（挥手；时序/姿态出处写在注释里，抬臂角按实测标定）
    ├── play_motion.py         动作播放器（仿真实测 + 出图 + 导出/回放 CSV）
    ├── test_sim_core.py       仿真内核自检 + URDF/MJCF 交叉核对
    ├── drive_sim.py           可驱动性自检（PD + 重力补偿，三场景）
    └── view_model.py          渲染 PNG / 交互查看器
```

WSL 内部布局：

```
~/q1_sim/q1_urdf/     上游模型（URDF/网格/MJCF）
~/q1_sim/venv/        MuJoCo 与验证/抓屏工具的 venv
~/q1_ws/src/          aimdk_msgs（SDK 消息）+ q1_description（Q1 模型）
~/q1_ws/install/      colcon 产物
```

## 对上游 q1-urdf 做了哪些修改

上游 `primebot/q1-urdf` 是 **ROS1 时代的包**，直接用是跑不起来的，本环境做了最小必要修正：

| 问题 | 处理 |
| --- | --- |
| `package.xml` 用 `catkin`、依赖 `roslaunch`/`rviz`（ROS1） | 改为 `ament_cmake`，依赖 `rviz2`/`robot_state_publisher` |
| `launch/*.launch` 是 ROS1 XML 格式 | 重写为 ROS2 Python launch |
| launch 引用 `urdf/Q1-V2.0-URDF-0601.urdf` —— **该文件不存在**（实际是 `q1_v3.urdf`） | 指向真实文件，并支持 `urdf:=` 覆盖 |
| launch 引用 `urdf.rviz` —— **该文件不存在** | 新写了 `rviz/q1.rviz` |
| launch 用了不存在的 `base_link`/`base_footprint`（URDF 根链接是 `pelvis`） | 去掉；RViz fixed frame 设为 `pelvis` |
| 网格路径是相对路径 `../meshes/` | 改为 `package://q1_description/meshes/` |
| URDF 内含 `<mujoco>` 扩展标签（urdfdom 不认识） | 生成 ROS2 版时移除 |

**模型本体、网格、惯量、关节限位均未改动。**

## 已知限制（重要）

1. **Gazebo 不可用（不是配置问题）**
   `q1-urdf` 的 URDF 里**没有任何 `<transmission>` / `<gazebo>` / `ros2_control` / plugin**，
   所以在 Gazebo 里只能当静态模型看，关节不会动。厂商唯一带驱动的模型是 MuJoCo 的
   `xml/q1_v3_mc.xml`（22 个 motor），因此本环境以 MuJoCo 作为运动仿真后端。

2. **WSL2 里连真机有网络限制（本机无解，只能绕）**
   本机是 **Windows 10 (19045)**，WSL2 只能走 NAT（WSL 地址 `172.21.204.166`，机器人在 `10.1.1.100/101`）。
   DDS 会把 WSL 的 `172.21.x` 地址通告出去，机器人按该地址回连必然失败；
   `networkingMode=mirrored` 需要 Windows 11 22H2+，本机用不了。
   **对策与取舍见 `docs/真机连接与WSL2_NAT.md`**（首选：把程序部署到机器人板子上跑）。
   **纯仿真不受影响**。

3. **模型与版本命名不一致**
   仓库叫 V3.0、URDF 文件名是 `q1_v3.urdf`、但包名与关节表文件名是 `Q1-V2.0-URDF-0601`；
   文档里的 SDK 是 v0.1.2.7-R，而厂商下载页已到 v0.3.0.3。联调真机前建议核对版本。

4. **模型没有手部/足部网格**
   24 个网格覆盖头/躯干/骨盆/大臂/大小腿/踝，没有手掌与脚掌细节。

5. **URDF 的 root link 带惯量会触发 KDL 警告**
   `robot_state_publisher` 启动时会警告
   `The root link pelvis has an inertia specified in the URDF, but KDL does not support a root link with an inertia`。
   **不影响 RViz2 显示**；若要用依赖 KDL 的工具（如某些 IK），标准做法是加一个无惯量的
   `base_link` 作根链接（顺带也能对上上游 launch 里那个不存在的 `base_link`）。

6. **镜像源（三处都要换，否则在国内几乎装不动）**

   | 用途 | 源 | 说明 |
   | --- | --- | --- |
   | ROS2 apt 源 | `mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu` | `packages.ros.org` 在本机 **HTTP 000 / TLS 校验失败**，换证书后仍不通 |
   | Ubuntu 归档源 | `mirrors.tuna.tsinghua.edu.cn/ubuntu` | 实测 archive.ubuntu.com 约 **2 MB/s**，清华 **18.7 MB/s**（快约 9 倍） |
   | pip | `pypi.tuna.tsinghua.edu.cn` | 写 `~/.config/pip/pip.conf` 与 `/etc/pip.conf` |

   `ros-humble-desktop` 的依赖闭包实测是 **1343 个包、约 762 MB**，所以换 Ubuntu 归档源是必需的。

   **ROS 公钥**是最容易失败的一步：`raw.githubusercontent.com` 本机不可达（HTTP 000），
   各镜像站的 `/ros/ros.key` 都是 404。`fetch-ros-key.sh` 改为从 `keyserver.ubuntu.com` 获取，
   并**强制校验完整指纹** `C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`（Open Robotics）；
   失败时再试 GitHub 代理，仍然校验同一指纹——因此经第三方传输也无法被替换成别的密钥。

   换源开关：`ROS_MIRROR=... UBUNTU_MIRROR=... sudo -E bash install-ros2.sh`；
   不想动 Ubuntu 源就加 `SKIP_UBUNTU_MIRROR=1`；只求跑通、接受跳过包签名校验则用
   `ALLOW_TRUSTED=1 sudo -E bash install-ros2.sh`。

7. **曾经有两个"看起来像物理现象"的实现缺陷（都已修掉，留作教训）**
   它们一度让本环境的结论出错，所以特别记下来：

   - **固定基座不能靠"每步把 `qpos`/`qvel` 强行归位"实现**。那样等于持续向腿注入能量，
     实测出现**2 个积分步周期**的极限环：`qvel` 在 ±3.2 rad/s 之间逐拍翻转、力矩全程贴着
     ±36 N·m 上限，而关节角度看起来几乎没动（误差 <2.5°）。正确做法是 MuJoCo 的
     **equality/weld 约束**（见 `sim_core.build_pinned_model_xml`）。
   - **踝关节的阻尼不能照抄髋/膝**。脚掌惯量只有 ~0.006 kg·m²，`kd=12` 时 `kd·dt/I ≈ 2`
     正好压在显式积分的稳定边界上，脚一离地就自激（力矩反复打满 ±22 N·m）。
     现在踝关节单列一组 `(220, 3)`；临界阻尼估算 `kd ≈ 2√(kp·I) ≈ 2.3`，
     与厂商真机腿增益里的 `damping=2` 正好对得上。

   修正后，试验台工况的稳态饱和从 12% 降到 **0**，稳态跟踪误差从 2.45° 降到 **0.000°**。
   **在此之前本文档写的"MJCF 力矩上限偏保守、保持该姿态需要超限力矩"是错误结论，已删除**——
   力矩上限没有问题，是上面两个 bug 伪造出了饱和。`test_sim_core.py` 里加了两项防回归测试
   分别盯这两件事（"无指令时没有关节贴着力矩上限"、"悬吊+屈膝下没有关节自激振荡"）。

   顺带一个真实约束：**基座焊在站立高度、脚又踩在地上时，屈膝这类改变腿长的动作是不可达的**
   （脚会被地板顶住）。测腿部动作要用 `floor_drop` 把地面下移给脚留空间，
   这不是控制器调不好，是姿态本身做不到。

8. **默认姿态是自检值，不是标定值**
   `drive_sim.py` / `sim_core.py` / `motions.py` 里的姿态与增益只用于验证仿真可驱动，
   **不能当作真机关节角度下发**。

9. **SDK 里只有动作接口，没有动作数据**
   厂商的预置动作（`McPresetMotion.msg`：`Q_DANCE_A/B/C` = 3046/3047/3048，
   以及 `Q_WAVE_HAND` 3004、`Q_STRETCH` 3010、`Q_MAKE_HEART` 3007 等）
   实现都在**运控板**上，SDK 压缩包里没有对应轨迹，仿真里**无法直接调 `Q_DANCE_A`**。
   而且 `McControlArea.msg` 只有 `LEFT_HAND/RIGHT_HAND/HEAD/WAIST` 四个位标志
   （没有腿、没有全身），所以这批预置动作应当**只作用于上肢**（推断，未在真机验证）。

   要在仿真里看跳舞，三条路：

   1. **自己编排**——本次挥手走的就是这条，见 `mujoco/motions.py` + `trajectory.py`；
   2. **录真机再回放**——让真机跳，同时订阅 `/aima/hal/joint/state`（`JointStateArray`，
      含 position/velocity/effort/温度/电压）把关节轨迹录下来，再用
      `play_motion.py --from-csv` 在仿真里回放（CSV 格式已经就是这条路的产物）；
   3. **找厂商要数据**——全身动作接口 `SetMcMotion.srv`（"播放全身动作接口"）需要
      `model`（onnx 绝对路径）与 `resource`（轨迹参考文件绝对路径），这两个文件**不在 SDK 里**。

   另外注意：真机上自定义上肢动作走话题 `/aima/mc/custom/joint/command`（1000 Hz，
   刚度 20/阻尼 3），而此时 `McInputSource.msg` 会在 `rc/vr/app_proxy/interaction/pnc`
   等多个输入源之间按 `priority` 仲裁、还有 `timeout`（默认 1000 ms），
   自定义动作**可能被更高优先级的输入抢走**——真机演示时的隐藏坑。

10. **别直接用厂商 `wave_pose()` 的抬臂角度，也别假设"肘 0° 是伸直"**
    本模型 `right_elbow` 的 **0° 本身是弯的**（两连杆夹角 82°），正向越大越**伸直**；
    而 `shoulder_yaw` 的轴 `(0,0,1)` 装在上臂中点，**始终与上臂同轴**——它是「肱骨自转」，不是「肩左右摆」。两条凑在一起，
    厂商那组 `elbow +1.10` 就变成"绕一条伸直的胳膊拧螺丝"——手几乎不动。
    抬臂姿态要按 `shoulder_roll` 抬 + **保留肘部弯曲**，让肘高于肩、前臂偏离转轴。
    实测过程与数据见 §5。**肘关节零位语义这类事必须实测，不能照字面理解。**

    还有一条更容易踩的：**`shoulder_yaw` 是「手臂落在哪个平面」的开关，不是摆动轴。**
    它 ±90° 的行程刚好够把弯肘平面从身体正中对称面（`yaw=0`）转到躯干/头所在的正面平面
    （`yaw=±90°`，本次取 −85°，实测共面度 0.996；取正号小臂会朝下）。
    **要让手臂留在某个竖直平面里挥手，摆动轴必须选「该平面法线方向」那根轴**
    （正面平面的法线是 x，所以用 `shoulder_roll`）；拿 `yaw` 当摆动轴，
    摆的其实是弯肘平面的朝向，小臂必然甩出平面。

## 故障排查

- **`ros2 topic echo /robot_description` 只显示 154 字节** → **这是正常的，别被它骗了**。
  `ros2 topic echo` 会截断长字符串，而 URDF 有 27994 字节。
  判断 URDF 是否发布请用 `bash scripts/verify_ros2.sh`（内部走 `check_display.py`）。
- **`joint_state_publisher` 打印 `Waiting for robot_description ...`** → 正常。
  它在启动瞬间总会先打印这句，随后静默拿到 latched 的 URDF。
  刚启动时 `/joint_states` 可能短暂是空的，等几秒即可（验证脚本最多等 25 秒）。
- **`colcon build` 报找不到 rosidl 依赖** → `install-ros2.sh` 没装全，重跑一次（幂等）。
- **RViz2 里模型是白色/看不见** → 先跑 `verify_ros2.sh` 看是哪一环节断了；
  再确认 fixed frame 是 `pelvis`、RobotModel 的 Description Topic 是 `/robot_description`。
- **`ros2 launch` 报 URDF 解析失败** → 用 `bash verify_ros2.sh` 看日志，必要时
  `ros2 launch q1_description display.launch.py urdf:=$HOME/q1_ws/src/q1_description/urdf/q1_v3.urdf`。
- **MuJoCo 渲染报无法创建渲染器** → 无头渲染需要 `libegl1`/`libgl1-mesa-dri`/`libosmesa6`
  （在 `install-ros2.sh` 第 5 步里装），并设 `MUJOCO_GL=egl`。
- **仿真里关节一直在抖、力矩反复打满上限** → 先怀疑增益，别急着下"物理上就需要这么大"的结论。
  踝/腕这类末端关节惯量极小（~0.006 kg·m²），阻尼稍大就会在 1 ms 步长下自激，
  典型现象是 `qvel` **逐拍正负翻转**（而不是平滑振荡）。跑 `test_sim_core.py` 的第 8 项可直接判定。
- **仿真出图是竖着的** → `mujoco.Renderer` 的参数顺序是 `(model, height, width)`，
  传成 `(width, height)` 就会得到竖图。
- **交互窗口退出时报段错误 / 只出数据看不到窗口** → 两个坑：① 无头出图需要 `MUJOCO_GL=egl`，
  而交互窗口必须走 **GLFW**，两者同时强制会在退出时 GL 上下文冲突（`play_motion.py` 已按命令行
  自动切换）；② WSLg 下 GLFW 的进程退出清理本身会段错误，`play_motion.py` 在 `--viewer` 模式下
  输出完结果后直接 `os._exit()` 跳过析构（结果文件都已落盘，不影响任何数据）。
- **动作播放了，但手/关节"看起来没在动"** → 先怀疑**转轴选错了**，而不是幅值太小。
  典型例子：肩 `yaw` 的轴与抬起后的上臂同轴时，加大角度也只是"拧螺丝"。
  用 `play_motion.py` 打印的 **腕部横扫** 和 **肘-肩高度差** 判断，别只看关节角度。
  见 §5 与"已知限制"第 10 条。
- **手抬起来了，但小臂/手臂"偏出了身体那个平面"** → 这是**弯肘平面的朝向**没调，
  不是抬臂角度的问题。用 `shoulder_yaw`（常量）把弯肘平面转进躯干/头所在平面，
  **摆动轴要换成该平面的法线方向那根轴**（正面平面 → `shoulder_roll`）。
  `play_motion.py` 会打印"小臂/大臂平面 与 躯干/头平面 的最大夹角"并做硬判定
  （`--min-plane-align`，默认 0.99 ≈ 8°）。见 §5。
- **出图里手被画框切掉了** → 用差异包围盒的**边距**判断（`play_motion.py` 会打印 bbox），
  边距为 0 就是切到了；调 `make_camera` 里的 `distance`。
- **pip 卡住** → 默认 PyPI 很慢，已写入 `~/.config/pip/pip.conf` 指向清华镜像。
- **安装卡在取公钥** → 先单独测：`bash scripts/fetch-ros-key.sh /tmp/k.gpg`。
  它成功的前提是 keyserver 可达；全部来源都失败时脚本会**中止而不是使用未校验的源**。
- **想确认 apt 与你机器上的源是否真的匹配** → 用取到的公钥验签镜像的 `InRelease`：
  `curl -sSo /tmp/InRelease https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/dists/jammy/InRelease && gpgv --keyring /usr/share/keyrings/ros-archive-keyring.gpg /tmp/InRelease`
  出现 `Good signature from "Open Robotics"` 即说明 apt 一定能通过签名校验。
- **换源后想还原 Ubuntu 源** → `sudo cp /etc/apt/sources.list.q1-backup /etc/apt/sources.list && sudo apt-get update`。
- **WSLg 下想给 RViz2 截图** → 系统里没有任何截图工具（装它们要 sudo），
  可用 venv 里的 mss/python-xlib：`~/q1_sim/venv/bin/python ros2/shot_window.py`。
  注意抓整屏（root window）只会得到纯色，必须抓具体窗口。

## 安全提示

上游文档明确警示：**底层关节伺服是高物理风险操作**，不当指令可能损毁硬件；
官方《友好用户使用声明》在 SDK 压缩包内（仓库里没有），启用低层运控开发前务必阅读。
本环境默认只在仿真里跑，不下发真机指令。
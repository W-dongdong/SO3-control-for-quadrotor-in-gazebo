# SO3 Control for Quadrotor in Gazebo

PX4 Offboard 模式下的四旋翼控制器。姿态环与速度环全部自行实现，
PX4 仅承担混控器角色（`thrust_and_torque` 插口）。

## 控制结构

```
   手柄 ────►┌──────────────────┐
             │ 速度 PID  (x,y,z)│
             │ in : v_sp        │
             │ out: a_des       │
             └────────┬─────────┘
                 ▲    │
        v (PX4)  │    ▼
                 │   F_des/m = a_des + g          g = [0, 0, −9.81]
                 │    │
                 │    ├───────────────────────┐
                 │    ▼                       ▼
                 │  ┌──────────────┐     ‖F_des‖ / (m·g)
                 │  │   姿态反解     │         │
   yaw_des ──────┼─►│ z_b^d ← F_des │         ▼
                 │  │ x_c^d ← yaw   │     [开方线性化 √]
                 │  └──────┬───────┘          │
                 │         │ R_d              ▼
                 │         ▼            归一化推力 ──────────┐
                 │  ┌──────────────┐                        │
                 │  │  SO(3) 姿态环 │──► τ_x, τ_y ───────────┤
                 │  │ τ = Kp·e−Kd·Ω │                       │
                 │  └──────┬───────┘                        │
                 │         │ τ_z                            ▼
                 │         ▼                             ┌─────┐
                 └────  [偏航通道切换]  ────────────────► │ PX4 │──► 电机
                                                         └─────┘
```

- **外环 PID**：输入期望速度，输出期望加速度 `a_des`，共四个通道（x / y / z / 偏航角速度）
- **内环 SO(3)**：输入期望姿态 `R_d`，输出机体力矩 `τ`

姿态反解由期望推力方向与期望偏航角构造 `R_d`，沿用 Mellinger & Kumar 几何控制的写法。
控制器本身不含轨迹优化环节，期望速度直接由手柄给出。

### 偏航通道

| | 有偏航命令 | 无偏航命令 |
|---|---|---|
| `R_d` 的偏航角 | 当前航向 | 锁存的航向 |
| `τ_z` 来源 | 偏航 PID 覆盖内环输出 | SO(3) 自身的偏航分量 |

## 文件

| 文件 | 内容 |
|---|---|
| `px4_control/offboard_control.py` | 主节点：订阅、外环 PID、推力与姿态反解、话题发布 |
| `px4_control/control_method.py` | 控制律：SO(3) 姿态环、PID |

## 依赖

- ROS 2 Humble
- [`px4_msgs`](https://github.com/PX4/px4_msgs) **v1.17.0**（必须与 PX4 固件的 tag 一致）
- PX4-Autopilot v1.17.0 + Gazebo（`make px4_sitl gz_x500`）

## 编译与运行

```bash
# 依赖层（px4_msgs 编译一次即可）
colcon build --packages-select px4_msgs
source install/setup.bash

# 本包
colcon build --packages-select px4_control --symlink-install
source install/setup.bash

ros2 run px4_control offboard_control
```

运行前需先启动 PX4 SITL 与 uXRCE-DDS Agent（`MicroXRCEAgent udp4 -p 8888`），
并确认 `ros2 topic list | grep fmu` 能看到 `/fmu/out/*` 话题。

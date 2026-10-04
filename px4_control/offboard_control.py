
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,     # 发：心跳 + 要用哪种控制方式的声明
    VehicleThrustSetpoint,   # 发：推力
    VehicleTorqueSetpoint,   # 发：力矩
    VehicleStatus,           # 收：PX4 的状态
    VehicleOdometry,         # 收：位置
    VehicleLocalPosition     # 收：世界坐标系下的x, y, z加速度
)

from sensor_msgs.msg import Joy # 手柄消息

from px4_control.control_method import SO3_control, PID
import numpy as np

# ---- 发布话题：我们 -> PX4 ----
T_OFFBOARD_MODE = '/fmu/in/offboard_control_mode'    # 心跳
T_THRUST        = '/fmu/in/vehicle_thrust_setpoint'  # 推力
T_TORQUE        = '/fmu/in/vehicle_torque_setpoint'  # 力矩

# ---- 订阅话题：PX4 -> 我们 ----
T_STATUS        = '/fmu/out/vehicle_status_v1'       # 体温计  ← 只有它带 _v1
T_ODOMETRY      = '/fmu/out/vehicle_odometry'
T_LOCAL_POS     = '/fmu/out/vehicle_local_position_v1'

# 手柄话题
T_JOY = '/joy'

def qos_out() -> QoSProfile:
    """订阅 /fmu/out/*（读 PX4 状态）用"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def qos_in() -> QoSProfile:
    """发布 /fmu/in/*（给 PX4 下指令）用"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

def vector_clamp(vector, max_length):
    max_length = abs(max_length)
    length = np.linalg.norm(vector)
    if length > max_length:
        vector = vector/(length + 1e-8) * max_length
    return vector


# ---------------------------------------------------------------------------
# 推力 -> 电机控制信号 的线性化
#
# 【为什么要做这一步】
# 我们算出来的是"想要多少推力"，而 PX4 要的是"归一化的电机控制信号"。
# 这两者【不是】线性关系 —— 物理链是：
#
#       c (归一化控制信号) -> ω (转速) -> 推力
#                         线性          平方
#
# 所以"想要多少推力"和"该给多少控制信号"之间差一个【开方】。
# 只在悬停点附近两者才近似相等，偏离越多误差越大
# （实测：想要 0.5 倍重量时线性算法会差 -29%，想要 1.5 倍时差 +31%）。
#
# 【参数是从哪来的】
# 仿真机型文件 ROMFS/.../airframes/4001_gz_x500 里：
#     param set-default SIM_GZ_EC_MIN1 150      <- 电调怠速
#     param set-default SIM_GZ_EC_MAX1 1000     <- 电调满量程
# 而 gz 模型（x500/model.sdf）里：
#     maxRotVelocity 1000, motorConstant 8.54858e-06
# 合起来就是：ω = 150 + 850·c，推力 = k·ω²
#
# HOVER_C 是【实测】出来的悬停点（见 notes：不是理论算的 0.769，
# 实测在 0.72~0.73 之间）。
#
# ⚠️ A_ESC 是【硬件属性】。换机型 / 上真机必须重新标定这两个数，
#    但公式的形状不变（真机的推力曲线可能不是纯平方，那时改用
#    PX4 的 THR_MDL_FAC 参数去标定更合适）。
# ---------------------------------------------------------------------------
ESC_MIN = 150.0
ESC_MAX = 1000.0
HOVER_C = 0.7288                                   # 实测的悬停控制信号
A_ESC = ESC_MIN / (ESC_MAX - ESC_MIN)               # = 0.17647，怠速偏置


def thrust_to_motor(thrust_ratio):
    """把「期望推力 ÷ 悬停推力」换算成归一化的电机控制信号。

    推导（三步）：
        ① ω = ESC_MIN + (ESC_MAX - ESC_MIN)·c          c 是控制信号
        ② 推力 ∝ ω²                                    螺旋桨的物理
        ③ 设 ratio = 推力/悬停推力
           → ω = ω_悬停 · √ratio                       （平方所以开方）
           → c + A = (HOVER_C + A) · √ratio            （把怠速偏置提出来）
           → c = (HOVER_C + A)·√ratio − A              （再减回去）

    验算（ratio=1 时它必须退化成 HOVER_C，否则就是错的）：
        ratio = 1.0  ->  c = 0.7288  （悬停）✓
        ratio = 0.5  ->  c = 0.4637  -> 实际推力正好 0.500 倍 ✓
        ratio = 1.5  ->  c = 0.9323  -> 实际推力正好 1.500 倍 ✓

    参数：
        thrust_ratio : 期望推力是悬停推力的多少倍。1.0 = 悬停。
    返回：
        归一化的电机控制信号。注意它【可能小于 0 或大于 1】，
        调用方要用 np.clip(clip 到 [0,1])。
    """
    # 姿态接近翻过来时，投影可能算出负的推力。推力不能为负 -> 当 0 处理。
    # （不这么写的话 sqrt 会得到 nan，发出去行为不可控）
    ratio = max(float(thrust_ratio), 0.0)
    return (HOVER_C + A_ESC) * np.sqrt(ratio) - A_ESC

AXIS_LX = 0     # 左摇杆：左右
AXIS_LY = 1     # 左摇杆：上下
AXIS_RX = 2     # 右摇杆：左右
AXIS_RY = 5     # 右摇杆：上下
# [3] / [4] 是扳机（静止 +1.00）

def joy_to_sticks(joy_msg):
    """从 sensor_msgs/Joy 里取出两个摇杆，返回 [左x, 左y, 右x, 右y]。

    每个分量都在 -1.0 ~ 1.0 之间。

    ⚠️ 如果 axes 数量不够（手柄没接上、驱动没上报、话题是别的设备），
       缺的部分会保持 0 而不是报错 —— 让上层可以拿"全 0 = 摇杆居中"
       当作安全兜底，而不是抛出异常把控制循环打断。
    """
    axes = joy_msg.axes
    out = np.zeros(4)
    for i, idx in enumerate((AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY)):
        if idx < len(axes):
            out[i] = axes[idx]
    return out


class OffboardControl(Node):

    def __init__(self):
        super().__init__("offboard_control")    #注册这个节点叫“offboard_control"

        self.get_logger().info(f'Launching, waiting for PX4…')
        
        self.pub_mode = self.create_publisher(
            OffboardControlMode, T_OFFBOARD_MODE, qos_in())
        self.pub_thrust = self.create_publisher(
            VehicleThrustSetpoint, T_THRUST, qos_in())
        self.pub_torque = self.create_publisher(
            VehicleTorqueSetpoint, T_TORQUE, qos_in())

        self.create_subscription(
            VehicleStatus, T_STATUS, self.on_state, qos_out())
        self.create_subscription(
            VehicleOdometry, T_ODOMETRY, self.on_odom, qos_out())
        self.create_subscription(
            VehicleLocalPosition, T_LOCAL_POS, self.on_lpos, qos_out())
        self.create_subscription(
            Joy, T_JOY, self.on_joy, qos_in())

        self.status = None
        self.hover_thrust = 0.7288    # 悬停推力 0.7288
        self.max_torque = 10
        self.odom = None              # 用来存odometry消息
        self.lpos = None
        self.joy = None               # 手柄消息
        self.yaw_last = 0             # 没给过 yaw 命令时的默认期望航向
        self.height_last = 0          # 没给过高度命令时的默认期望高度
        self.create_timer(0.01, self.loop)
        self.so3_controller = SO3_control(0.53, 0.10)
        self.so3_controller.SetTargetAttitude(np.array([[1, 0, 0],
                                                        [0, 1, 0],
                                                        [0, 0, 1]]))

        # ------------ PID 设置 ------------ #
        self.height_controller = PID(Kp=7, Ki=0.05, Kd=5, integral_limit=1, output_limit=4)
        self.vx_controller = PID(Kp=2, Ki=0.0, Kd=0.8, integral_limit=0.5, output_limit=10)
        self.vy_controller = PID(Kp=2, Ki=0.0, Kd=0.8, integral_limit=0.5, output_limit=10)
        self.yaw_controller = PID(Kp=20, Ki=0.0, Kd=0.8, integral_limit=0, output_limit=10)


    def on_state(self, msg):
        if self.status is None:
            self.get_logger().info('Recv from PX4')
        self.status = msg

    def stamp_us(self):
        """
        当前时间，换成 PX4 要的格式：墙钟微秒。
            get_clock()    → Node 给的时钟（和 create_publisher 一样，是父类提供的）
            .now()         → 一个 Time 对象
            .nanoseconds   → 把这个时间取成【纳秒】整数
            / 1000         → 纳秒 → 微秒(PX4 用微秒)
            int()          → 字段要整数，除法结果是浮点

        必须是【墙钟】（从 1970 年算起），不是 PX4 的开机计时 ——
        PX4 那边靠 XRCE 的时间同步把墙钟换算成它自己的时钟。
        填错的后果: offboard 的"心跳超时检查"用的就是这个字段。
        """
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def publish_offboard_mode(self):
        msg = OffboardControlMode()     # 来源于最前面的from px4_msgs.msg import OffboardControlMode
        msg.timestamp = self.stamp_us()
        """
        True和False表明我们打算给什么, 剩下的由PX4完成
        假设我们只想要给位置, 那么position就填True, PX4底层会自己控制无人机到那里
        七个里面只能有一个是True
        """
        msg.thrust_and_torque = True
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = False
        self.pub_mode.publish(msg)

    def publish_thrust_torque(self, thrust_norm, torque=(0.0, 0.0, 0.0)):
        t = VehicleThrustSetpoint()
        t.timestamp = self.stamp_us()
        """
        物理上，推力永远沿机体的 -z 方向（机体 z 朝下、推力朝上）。那消息为什么设计成三维？为了通用性：
        多旋翼：只有 z 分量有意义, x/y 你填什么 PX4 都不用(混控器只看 z)
        其他机型(矢量推力之类): x/y 可能有用
        """
        t.xyz = [0.0, 0.0, -float(thrust_norm)]
        self.pub_thrust.publish(t)

        m = VehicleTorqueSetpoint()
        m.timestamp = self.stamp_us()
        m.xyz = [float(torque[0]), float(torque[1]), float(torque[2])]
        self.pub_torque.publish(m)

    def on_odom(self, msg):
        if self.odom is None:
            self.get_logger().info('Recv odometry msg from PX4...')
        self.odom = msg

    def on_lpos(self, msg):
        if self.lpos is None:
            self.get_logger().info('Recv lpos msg from PX4...')
        self.lpos = msg

    def on_joy(self, msg):
        if self.joy is None:
            self.get_logger().info('Recv msg from joystick...')
        self.joy = msg

    def loop(self):
        self.publish_offboard_mode()
        torque = (0.0, 0.0, 0.0)
        thrust = self.hover_thrust
        if self.joy is not None:
            sticks = joy_to_sticks(self.joy)
            vx = sticks[0]*5
            vy = sticks[1]*5
            self.vx_controller.set_target(vx)
            self.vy_controller.set_target(vy)

        if self.odom and self.lpos and self.joy is not None:
            # ---- 当前姿态 ----
            R = self.so3_controller.quat_to_R(self.odom.q)
            W = np.array(self.odom.angular_velocity)
            self.so3_controller.SetCurrentStatus(R, W)

            # ---- 高度控制 ----
            # self.height_controller.set_measure(-self.odom.position[2])
            self.height_controller.set_measure(-self.lpos.vz)
            a_up_des = joy_to_sticks(self.joy)[3]
            self.height_controller.set_target(a_up_des)
            a_up = self.height_controller.control_calc()

            # ---- 合成推力 ----
            a_x, a_y = 0.0, 0.0
            self.vx_controller.set_measure(self.lpos.vx)
            self.vy_controller.set_measure(self.lpos.vy)
            a_x = self.vx_controller.control_calc()
            a_y = self.vy_controller.control_calc()
            a_xy = np.array([a_x, a_y])
            a_xy = vector_clamp(a_xy, 5)
            a_x = a_xy[0]; a_y = a_xy[1]
            # px4的z+方向朝下
            a_des = np.array([a_x, a_y, -a_up])
            z_b = np.array([R[0][2], R[1][2], R[2][2]])
            """
            F/m = a_des + g; F/m = u_a
            这里F是推力给无人机的力, 实际上推力F_t = -F, 推力方向与F相反
            F_t = -(a_des + g)
            """
            g = np.array([0, 0, -9.80665])  # px4里的z轴朝下，所以g值为负时加速度才会朝上
            u_a = -(a_des + g)

            # ---- 控制姿态 ----
            # 计算机体的期望姿态
            # 有 yaw 命令时把当前航向记进 yaw_last，没命令时就不更新 ——
            # 于是松手后 yaw_last 冻在松手那一刻的航向，由 SO(3) 自己把飞机拧回去
            yaw_cmd = joy_to_sticks(self.joy)[2]
            yaw_cmd_flag = 0
            if abs(yaw_cmd) > 0.1:
                self.yaw_last = np.arctan2(R[1, 0], R[0, 0])
                yaw_cmd_flag = 1
            yaw_des = self.yaw_last

            zb_des = u_a / np.linalg.norm(u_a)
            xc_des = np.array([np.cos(yaw_des), np.sin(yaw_des), 0])
            yb_des = np.cross(zb_des, xc_des)
            yb_des = yb_des / np.linalg.norm(yb_des)
            xb_des = np.cross(yb_des, zb_des)

            R_d = np.array([[xb_des[0], yb_des[0], zb_des[0]],
                            [xb_des[1], yb_des[1], zb_des[1]],
                            [xb_des[2], yb_des[2], zb_des[2]]])

            self.so3_controller.SetTargetAttitude(R_d)
            torque = self.so3_controller.AttitudeControl()

            # 只有【有 yaw 命令】时才用角速度环覆盖 SO(3) 的偏航力矩；
            # 没有命令就保留 SO(3) 算出来的偏航力矩，让它去锁航向
            if yaw_cmd_flag != 0:
                self.yaw_controller.set_measure(W[2])
                self.yaw_controller.set_target(yaw_cmd*0.5)
                torque[2] = self.yaw_controller.control_calc()

            torque = vector_clamp(torque, self.max_torque)

            # F/m * z_b，期望推力加速度投影到机体坐标轴机体推力加速度的需要的大小(from minimum snap)
            f_rel = np.dot(z_b, u_a)/9.80665    # 为什么除以重力g？
            thrust = thrust_to_motor(f_rel)

            self.get_logger().info(f'vx:{self.lpos.vx:.2f} vy:{self.lpos.vy:.2f} H:{-self.odom.position[2]:.2f}')
        self.publish_thrust_torque(thrust, torque)      # 这里推力发的是电机转速的归一化，与推力的关系不是线性的


def main(args=None):
    rclpy.init(args = args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
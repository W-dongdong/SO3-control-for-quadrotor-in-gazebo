"""SO(3) 姿态环：把实际姿态 R 拉到期望姿态 R_d，输出力矩 M。

【PX4 给的（我们只读，不用自己算）】
    /fmu/out/vehicle_odometry
        .q                 姿态四元数 [w,x,y,z]，机体 FRD -> NED（EKF 算好的）
        .angular_velocity  机体系角速度 [rad/s]（EKF 滤波过）
    /fmu/out/vehicle_local_position_v1
        .vx/.vy/.vz        NED 速度
        .ax/.ay/.az        NED 加速度（重力已减掉）
    ── 所以不需要写姿态估计器（不用 Mahony），也不需要自己把 IMU 转到世界系
    ── 混控（力矩/推力 -> 四个电机）也是 PX4 做

【我们自己实现的（全部内容）】
    1. 四元数 -> 旋转矩阵 R
    2. 期望姿态 R_d          （第一版：硬编码 = 单位阵）
    3. 姿态误差 e = ½(RᵀR_d − R_dᵀR)^∨
       ⚠️ 本文件用【期望 − 实际】的约定，和论文的 ½(R_dᵀR − RᵀR_d) 正好差一个负号
          物理含义也不同：本文件的 e 指向"该往哪边推"，论文的 e_R 指向"现在偏在哪边"
    4. 角速度误差 e_Ω = Ω                （第一版 Ω_d = 0，即 e_Ω = Ω）
    5. 力矩 M = +Kp·e − Kd·e_Ω
       ⚠️ Kp 项是【正号】—— 因为上面 e 的定义和论文相反，所以这里要跟着翻
       ⚠️ Kd 项永远是【负号】—— 阻尼必须反抗角速度，和约定无关
       （第一版省略陀螺项 Ω×JΩ）
    6. 归一化 + clamp + 发布到 /fmu/in/vehicle_torque_setpoint
"""

import numpy as np

class SO3_control:

    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.R_d = None
        self.R  = None
        self.W = None

    def SetTargetAttitude(self, Rdes):
        self.R_d = Rdes

    def SetCurrentStatus(self, R, W):
        self.R = R
        self.W = W

    def quat_to_R(self, q):
    # 四元数 [w,x,y,z] -> 旋转矩阵。
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
            [  2*(x*y + w*z),   1 - 2*(x*x + z*z),   2*(y*z - w*x)],
            [  2*(x*z - w*y),     2*(y*z + w*x),   1 - 2*(x*x + y*y)],
        ])

    def AttitudeControl(self):
        if self.R_d is None or self.R is None or self.W is None:
            return np.zeros(3)
        R_e = 0.5*(self.R.T@self.R_d - self.R_d.T@self.R)       # 这里是: 期望 - 实际，和论文相反
        R_e_Vee = np.array([R_e[2, 1], R_e[0, 2], R_e[1, 0]])
        torque = self.Kp*R_e_Vee - self.Kd*self.W
        return torque

class PID:
    def __init__(self, Kp, Ki, Kd, integral_limit, output_limit):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.P_term = 0
        self.I_term = 0
        self.D_term = 0
        self.target = 0
        self.measure = 0
        self.last_measure = 0
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)

    def set_measure(self, measure):
        self.measure = measure

    def set_target(self, target):
        self.target = target

    def control_calc(self):
        error = self.target - self.measure
        self.P_term = self.Kp * error
        self.I_term += self.Ki * error
        self.D_term = self.Kd * (self.last_measure - self.measure)

        if self.I_term > self.integral_limit:
            self.I_term = self.integral_limit
        if self.I_term < -self.integral_limit:
            self.I_term = -self.integral_limit

        output = self.P_term + self.I_term + self.D_term

        if output > self.output_limit:
            output = self.output_limit
        if output < -self.output_limit:
            output = -self.output_limit 

        self.last_measure = self.measure

        return output

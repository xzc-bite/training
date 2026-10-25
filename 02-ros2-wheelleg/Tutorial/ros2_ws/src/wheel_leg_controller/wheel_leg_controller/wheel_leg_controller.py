# 导包
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Joy

from rm_msgs.msg import WheelLegCmd

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller
from .roll_leg_control import RollLegController
from .ground_detection import detect_leg_ground_status
from .util.climb_traj import VelocityRampFilter

# /joint_states 的关节名，取数据时按名字索引，不依赖消息里的顺序
JOINT_NAMES = ['right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel']


class PIDController:
    """带限幅的 PID，腿长环和偏航环共用。

    微分项是步差分、没除 dt，因为调用频率固定在 500 Hz，够用；
    换到别的频率下用的话要注意这一点。
    """

    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        self.feedforward = feedforward
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, current, target):
        error = target - current
        # 积分限幅防饱和，输出统一 clip
        self.integral += error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        derivative = error - self.prev_error
        self.prev_error = error
        output = (self.kp * error + self.ki * self.integral
                  + self.kd * derivative + self.feedforward)
        return np.clip(output, -self.max_out, self.max_out)


class WheelLegControllerNode(Node):
    """轮腿平衡控制器节点。

    职责边界：只做"节点该做的事"——参数、话题、周期调度、传感器
    预处理、力矩限幅下发。具体的 LQR / VMC 数学在队里的文件里。
    """

    def __init__(self):
        super().__init__('wheel_leg_controller')

        # ---- 参数 ----
        self.declare_parameter('dt', 0.002)               # 必须等于仿真 dt，否则时序对不上
        self.declare_parameter('wheel_radius', 0.06)      # 轮半径，位置环 x_b 的换算系数
        self.declare_parameter('base_leg_length', 0.1527) # 目标腿长：落在 LQR 的 K 矩阵可靠区间 0.09~0.17 内
        self.declare_parameter('min_leg_length', 0.10)    # 腿长下限，防止五连杆折过头
        self.declare_parameter('max_leg_length', 0.17)    # 腿长上限，和 K 矩阵可靠区间对齐
        self.declare_parameter('wheel_torque_limit', 1.5) # 轮力矩限幅 (N·m)
        self.declare_parameter('leg_torque_limit', 1.8)   # 髋关节力矩限幅 (N·m)
        self.declare_parameter('enable_ground_protection', False)  # 离地保护，仿真场景默认用不上
        self.declare_parameter('enable_joy', True)        # 订阅 /joy；没手柄也不影响，收不到消息自动回退
        self.declare_parameter('joy_timeout', 0.5)        # 手柄多久没消息就回退到 /cmd_vel (s)
        self.declare_parameter('accel_rate', 0.8)         # 目标速度爬坡的加速度上限，防阶跃打翻车
        self.declare_parameter('decel_rate', 1.4)         # 减速度上限，松手能停得快一点

        # 下面三个是符号适配。仿真模型的轮子安装方向和队里原工程相反，
        # 照搬原版符号的话，俯仰环、位置环、速度环会全变成正反馈（必摔）。
        # 默认值是按本模型实测标定的；想复现原版行为或适配别的模型可以改。
        self.declare_parameter('pitch_sign', 1.0)         # 本模型前倒时 asin(euler) 为正，取正号
        self.declare_parameter('gyro_pitch_sign', 1.0)    # 俯仰角速度 = gyro.y，取正号
        self.declare_parameter('wheel_torque_sign', -1.0) # 轮力矩映射翻转：左轮 +wT、右轮 -wT
        self.declare_parameter('gravity_feedforward', -4.0)  # VMC 前馈力，实测稳定工作点

        self.dt = float(self.get_parameter('dt').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        base_len = float(self.get_parameter('base_leg_length').value)
        min_len = float(self.get_parameter('min_leg_length').value)
        max_len = float(self.get_parameter('max_leg_length').value)
        self.wheel_torque_limit = float(self.get_parameter('wheel_torque_limit').value)
        self.leg_torque_limit = float(self.get_parameter('leg_torque_limit').value)
        self.ground_protection = bool(self.get_parameter('enable_ground_protection').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        accel = float(self.get_parameter('accel_rate').value)
        decel = float(self.get_parameter('decel_rate').value)
        self.wheel_sign = float(self.get_parameter('wheel_torque_sign').value)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.gyro_pitch_sign = float(self.get_parameter('gyro_pitch_sign').value)
        self.gravity_ff = float(self.get_parameter('gravity_feedforward').value)

        # ---- 话题：订阅状态和指令，发布力矩 ----
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states',
                                                  self.joint_cb, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.actuator_pub = self.create_publisher(WheelLegCmd, '/actuator_cmd', 10)
        self.enable_joy = bool(self.get_parameter('enable_joy').value)
        if self.enable_joy:
            # 手柄硬件由 joy_node 读，这里只收标准 /joy 话题
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # ---- 控制律实例 ----
        self.declare_parameter('yaw_kp', 0.5)    # 偏航 P 增益，原版值
        self.declare_parameter('yaw_kd', 0.0)    # 原版 1.0 是按步差分没除 dt，量纲不对，置 0
        # 腿长 PID 积分默认关：目标腿长已标定在自然稳定点，前馈能抵消
        # 腿的位形漂移，不需要积分；开了反而会把 L0 推离目标（实测推高约 5 mm）
        self.declare_parameter('l0_ki', 0.0)
        self.declare_parameter('l0_max_i', 1.0)
        self.five_links_right = five_links_param(dt=self.dt)
        self.five_links_left = five_links_param(dt=self.dt)
        self.lqr_right = lqr_controller(dt=self.dt)
        self.lqr_left = lqr_controller(dt=self.dt)

        yaw_kp = float(self.get_parameter('yaw_kp').value)
        yaw_kd = float(self.get_parameter('yaw_kd').value)
        l0_ki = float(self.get_parameter('l0_ki').value)
        l0_max_i = float(self.get_parameter('l0_max_i').value)
        # 每条腿一套：偏航 PID + 腿长 PID。偏航输出最后同向加到左右两个轮子上
        self.left_yaw_pid = PIDController(yaw_kp, 0.0, yaw_kd, 2.0, 0.0, feedforward=0.0)
        self.right_yaw_pid = PIDController(yaw_kp, 0.0, yaw_kd, 2.0, 0.0, feedforward=0.0)
        self.left_L0_pid = PIDController(700.0, l0_ki, 5.0, 150.0, l0_max_i, feedforward=0.0)
        self.right_L0_pid = PIDController(700.0, l0_ki, 5.0, 150.0, l0_max_i, feedforward=0.0)

        # 横滚平衡：按 roll 把左右腿目标长度往两侧差分分配，抑制侧翻
        self.roll_balancer = RollLegController(kp=1.0, ki=0.0, kd=0.0,
                                               max_out=0.14,
                                               min_leg=min_len, max_leg=max_len)
        # 目标速度爬坡滤波，防 /cmd_vel 阶跃
        self.v_filter = VelocityRampFilter(dt=self.dt, accel_rate=accel, decel_rate=decel)

        # ---- 运行状态：回调只缓存数据，解算统一在定时器里做 ----
        self.imu_data = None
        self.joint_data = None
        self.cmd_vel_data = Twist()
        self.joy_data = None
        self.joy_last_stamp = None
        self.joy_prev_buttons = []
        self.target_length = base_len      # RB/LB 可调，限位 [min_len, max_len]
        self.current_v = 0.0               # 爬坡后的目标速度，同时是 LQR 的 v_set
        self.robot_is_airborne = False     # 上一周期整车是否判定离地
        self.step_count = 0

        # 500 Hz 主回路，和仿真"一条指令一个 dt"的契约对齐
        self.timer = self.create_timer(self.dt, self.control_timer_cb)

        self.get_logger().info(
            f'轮腿控制器已启动 (dt={self.dt}s, 基础腿长={base_len}m, '
            f'离地保护={"开" if self.ground_protection else "关"})')
        self.get_logger().info('等待 /imu 与 /joint_states 数据……（仿真节点启动后自动开始）')

    def imu_cb(self, msg: Imu):
        # 只存最新一帧，解算用定时器里的数据，不在回调里做运算
        self.imu_data = msg

    def joint_cb(self, msg: JointState):
        self.joint_data = msg

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vel_data = msg

    def joy_cb(self, msg: Joy):
        # 缓存手柄帧并记时间戳（用于超时回退判断）；
        # RB/LB 边沿触发改目标腿长，按一次 ±5 mm，够着就停（限幅）
        self.joy_data = msg
        self.joy_last_stamp = self.get_clock().now()

        min_len = float(self.get_parameter('min_leg_length').value)
        max_len = float(self.get_parameter('max_leg_length').value)
        buttons = list(msg.buttons)
        prev = self.joy_prev_buttons
        if len(buttons) > 5 and len(prev) > 5:
            if buttons[5] and not prev[5]:
                self.target_length = min(self.target_length + 0.005, max_len)
                self.get_logger().info(f'手柄 RB：目标腿长 → {self.target_length:.3f} m')
            if buttons[4] and not prev[4]:
                self.target_length = max(self.target_length - 0.005, min_len)
                self.get_logger().info(f'手柄 LB：目标腿长 → {self.target_length:.3f} m')
        self.joy_prev_buttons = buttons

    def control_timer_cb(self):
        """主回路：传感器 → 姿态/关节 → 目标输入 → 双腿解算 → 限幅下发。"""
        if self.imu_data is None or self.joint_data is None:
            return  # 状态还没齐，等仿真节点发数据

        # 1. 四元数转欧拉角，算法和 robot_info_update.py 保持一致
        q = self.imu_data.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        # pitch 取正号：本模型机体前倒（top 向 +x）时 asin 值就是正的，
        # LQR 的 K 矩阵要求前倒时俯仰环和位置环都往 +x 方向修，取正才自洽
        pitch = self.pitch_sign * math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        g = self.imu_data.angular_velocity
        gyro_pitch = self.gyro_pitch_sign * g.y   # 俯仰角速度：绕机体 +y 轴
        # 偏航率取 g.z（绕机体 +z）。原版取 g.x，那其实是横滚率，
        # 偏航环会完全失效，实测原地持续打转
        gyro_yaw = g.z
        gyro_roll = g.x       # 横滚率，目前没用，留着备用

        # 2. 关节状态：按名字取，消息里顺序变了也不怕
        js = self.joint_data
        pos = dict(zip(js.name, js.position))
        vel = dict(zip(js.name, js.velocity))
        right1, right2 = pos['right1'], pos['right2']
        left1, left2 = pos['left1'], pos['left2']
        rightwheel_pos = pos['rightwheel']
        leftwheel_pos = pos['leftwheel']
        rightwheel_vel = vel['rightwheel']
        leftwheel_vel = vel['leftwheel']

        # 左右轮安装方向相反，取平均得到共模位置/速度，LQR 的位置/速度环用
        common_motor_speed = (leftwheel_vel - rightwheel_vel) / 2.0
        common_motor_pos = (leftwheel_pos - rightwheel_pos) / 2.0

        # 3. 目标输入：手柄优先，超过 joy_timeout 没消息就回退 /cmd_vel
        target_v_raw = self.cmd_vel_data.linear.x
        target_w = self.cmd_vel_data.angular.z
        if self.enable_joy and self.joy_data is not None and self.joy_last_stamp is not None:
            if (self.get_clock().now() - self.joy_last_stamp).nanoseconds / 1e9 <= self.joy_timeout:
                axes = self.joy_data.axes
                if len(axes) > 3:
                    deadzone = 0.1
                    if abs(axes[1]) > deadzone:
                        target_v_raw = -axes[1] * 0.6   # 左摇杆前后 → 速度，最大 0.6 m/s
                    else:
                        target_v_raw = 0.0
                    if abs(axes[3]) > deadzone:
                        target_w = -axes[3] * 2.0       # 右摇杆左右 → 偏航角速度
                    else:
                        target_w = 0.0

        # 目标速度爬坡限幅，防止指令阶跃把车打翻
        self.current_v = self.v_filter.update(current_v=self.current_v,
                                              target_v=target_v_raw)

        # 4. 横滚平衡：按 roll 大小差分分配左右腿目标长度
        target_length_left, target_length_right = self.roll_balancer.compute_leg_lengths(
            current_roll=-roll, target_roll=0.0, base_length=self.target_length)

        # 5. LQR 的位置/速度环基准
        # 站桩时（当前速度和目标速度都为 0）必须冻结 x_b_set，
        # 否则 x_b - x_b_set 恒等于 0，位置环形同虚设，车会匀速滑走
        if not (self.current_v == 0.0 and target_v_raw == 0):
            self.lqr_left.x_b_set = common_motor_pos * self.wheel_radius
            self.lqr_right.x_b_set = common_motor_pos * self.wheel_radius
        self.lqr_left.v_set = self.current_v
        self.lqr_right.v_set = self.current_v

        # 6. 左右腿各解算一遍（两腿镜像安装，共模量符号相反，在 control_leg 里处理）
        ctrl_left_0, ctrl_left_1, left_wheel_T, left_flight = self.control_leg(
            leg_flag=1,
            five_links=self.five_links_left, lqr=self.lqr_left,
            yaw_pid=self.left_yaw_pid, L0_pid=self.left_L0_pid,
            motor1_angle=left1, motor2_angle=left2,
            pitch=pitch, gyro_pitch=gyro_pitch, gyro_yaw=gyro_yaw, target_w=target_w,
            target_length=target_length_left,
            common_motor_pos=common_motor_pos, common_motor_speed=common_motor_speed)

        ctrl_right_0, ctrl_right_1, right_wheel_T, right_flight = self.control_leg(
            leg_flag=0,
            five_links=self.five_links_right, lqr=self.lqr_right,
            yaw_pid=self.right_yaw_pid, L0_pid=self.right_L0_pid,
            motor1_angle=right1, motor2_angle=right2,
            pitch=pitch, gyro_pitch=gyro_pitch, gyro_yaw=gyro_yaw, target_w=target_w,
            target_length=target_length_right,
            common_motor_pos=common_motor_pos, common_motor_speed=common_motor_speed)

        # 7. 离地保护汇总（默认关闭）。开启后任一腿判定离地，
        # 就清零位置/速度期望，防止离地期间误累积
        if self.ground_protection:
            self.robot_is_airborne = bool(left_flight or right_flight)
            if self.robot_is_airborne:
                self.lqr_left.x_b_set = common_motor_pos * self.wheel_radius
                self.lqr_right.x_b_set = common_motor_pos * self.wheel_radius
                self.current_v = 0.0
        else:
            self.robot_is_airborne = False

        # 8. 6 路力矩统一下发，分别限幅
        cmd = WheelLegCmd()
        cmd.right_phi1 = float(np.clip(ctrl_right_0, -self.leg_torque_limit, self.leg_torque_limit))
        cmd.right_phi4 = float(np.clip(ctrl_right_1, -self.leg_torque_limit, self.leg_torque_limit))
        cmd.left_phi1 = float(np.clip(ctrl_left_0, -self.leg_torque_limit, self.leg_torque_limit))
        cmd.left_phi4 = float(np.clip(ctrl_left_1, -self.leg_torque_limit, self.leg_torque_limit))
        cmd.right_wheel = float(np.clip(right_wheel_T, -self.wheel_torque_limit, self.wheel_torque_limit))
        cmd.left_wheel = float(np.clip(left_wheel_T, -self.wheel_torque_limit, self.wheel_torque_limit))
        self.actuator_pub.publish(cmd)

        self.step_count += 1
        if self.step_count % 500 == 0:  # 每秒一次状态日志，方便看机器人状态
            self.get_logger().info(
                f'pitch={math.degrees(pitch):+.2f}° L0(L/R)={self.five_links_left.L_0:.4f}/'
                f'{self.five_links_right.L_0:.4f} m v={self.current_v:+.3f} m/s '
                f'wheel_T(L/R)={left_wheel_T:+.3f}/{right_wheel_T:+.3f} N·m')

    def control_leg(self, leg_flag, five_links, lqr, yaw_pid, L0_pid,
                    motor1_angle, motor2_angle, pitch, gyro_pitch, gyro_yaw,
                    target_w, target_length, common_motor_pos, common_motor_speed):
        """单腿解算：运动学 + LQR + VMC。

        返回 (髋力矩1, 髋力矩2, 轮力矩, 离地标志)。
        leg_flag: 0=右腿, 1=左腿。左右腿逻辑相同，只有两处不一样：
        共模量取负、最终符号取反（因为两腿镜像安装）。
        """
        # 右腿的共模位置/速度取负——两腿镜像安装，取负后左右解算出的
        # x_b / 速度方向才一致（与原版 control_right_leg 的处理一致）
        if leg_flag == 0:
            motor_pos = -common_motor_pos
            motor_speed = -common_motor_speed
        else:
            motor_pos = common_motor_pos
            motor_speed = common_motor_speed
        # 正运动学 + LQR 状态更新 + 按当前腿长查 K 矩阵
        five_links.forward_kinematics_cal(leg_flag, motor1_angle, motor2_angle,
                                          pitch, gyro_pitch)
        lqr.states_update(leg_flag, five_links, pitch, gyro_pitch,
                          motor_pos, motor_speed, self.wheel_radius)
        lqr.calc_k_matrix_from_poly(five_links.L_0)

        # 腿长 PID：误差 >3 cm 时冻结积分（开机从 0.209 收缩那一下防止积分冲爆），
        # 进入邻域后才允许积分消除前馈不准带来的稳态误差。
        # F_0 = 前馈 + PID，符号不用取反（实验验证 PID 输出直接叠加就能收敛）
        if abs(target_length - five_links.L_0) > 0.03:
            L0_pid.integral = 0.0
        F_0_ctrl = L0_pid.compute(five_links.L_0, target_length)
        cos_theta = math.cos(five_links.theta)
        F_0 = (self.gravity_ff / cos_theta if cos_theta != 0 else 0) + F_0_ctrl
        F_0 = np.clip(F_0, -100.0, 100.0)

        # 虚拟髋力矩 T_p：LQR 偶数项加权，管俯仰角和腿的角度
        T_p = (lqr.K[1] * (lqr.theta - 0.0)
               + lqr.K[3] * (lqr.d_theta - 0.0)
               + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
               + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
               + lqr.K[9] * (lqr.phi - 0.0)
               + lqr.K[11] * (lqr.d_phi - 0.0))

        if self.ground_protection and (detect_leg_ground_status(five_links, F_0, T_p) == 1
                                       or self.robot_is_airborne):
            # 判定离地：轮矩清零、T_p 只留姿态项、位置/速度期望交给主回路清
            wheel_T = 0.0
            T_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
            flight_flag = 1
        else:
            # 轮力矩：LQR 奇数项加权，管位置和速度；再加偏航 PID 的差动修正
            yaw_ctrl = yaw_pid.compute(gyro_yaw, target_w)
            wheel_T = (lqr.K[0] * (lqr.theta - 0.0)
                       + lqr.K[2] * (lqr.d_theta - 0.0)
                       + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                       + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                       + lqr.K[8] * (lqr.phi - 0.0)
                       + lqr.K[10] * (lqr.d_phi - 0.0))
            # 轮力矩符号：本模型左轮正力矩 → 车体 +x、右轮正力矩 → 车体 -x，
            # 与原版相反，默认翻转映射（wheel_torque_sign=-1）。
            # 偏航项同加到两个轮子上：两轮镜像安装，同号力矩产生偏航、
            # 异号产生平移，所以偏航修正要"同加"，-yaw_ctrl 才是负反馈
            if self.wheel_sign > 0:      # 原版符号
                wheel_T_out = -wheel_T if leg_flag == 1 else wheel_T
                yaw_term = -yaw_ctrl
            else:                        # 修正符号（默认）
                wheel_T_out = wheel_T if leg_flag == 1 else -wheel_T
                yaw_term = -yaw_ctrl
            wheel_T = float(np.clip(wheel_T_out + yaw_term,
                                    -self.wheel_torque_limit, self.wheel_torque_limit))
            flight_flag = 0

        # VMC 雅可比映射：虚拟力 F_0 + 虚拟力矩 T_p → 两个髋关节力矩
        torque_set_0, torque_set_1 = five_links.VMC_torque_cal(F_0, T_p)

        # 左腿结果取负（与原版一致）
        if leg_flag == 1:
            return -torque_set_0, -torque_set_1, wheel_T, flight_flag
        return torque_set_0, torque_set_1, wheel_T, flight_flag


def main(args=None):
    # 标准 rclpy 启动流程，Ctrl-C 时干净退出
    rclpy.init(args=args)
    node = WheelLegControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

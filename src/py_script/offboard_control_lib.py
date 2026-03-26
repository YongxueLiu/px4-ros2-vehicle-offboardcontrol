'''
================================================================================
作者/Author: 刘永学/Liu Yongxue
邮箱/Email: 805110687@qq.com
QQ群：1080856708

版权声明/Copyright Notice:
© All rights reserved. 保留所有权利。

使用许可/Usage License:
仅供个人使用，禁止商业用途。
For personal use only. Commercial use is prohibited.
================================================================================
'''


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OffboardControl + Vehicle 封装示例（支持调试输出与节流打印）
OffboardControl + Vehicle example (with debug print and throttled logging)

功能说明：
- OffboardControl: 提供底层飞控命令接口（ROS2 Publisher/Subscriber）
- Vehicle: 封装生命周期与高层调用（隐藏ROS2特性，方便上层调用）

Functionality:
- OffboardControl: Low-level PX4 offboard control interface via ROS2 publishers/subscribers.
- Vehicle: High-level wrapper managing node lifecycle and providing a simple Python API.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    GotoSetpoint,
)
import time
import math
import threading
from rclpy.executors import MultiThreadedExecutor
from px4_msgs.msg import VehicleAttitudeSetpoint
import numpy as np  # 用于 NaN

# ---------------- 常量定义 | Constants ----------------
DISTANCE_TOLERANCE = 0.5  # 位置误差容忍度 (米) | Position error tolerance (meters)
YAW_TOLERANCE = 0.1       # 航向角误差容忍度 (弧度) | Yaw error tolerance (radians)
namespace = ''  # 可根据实例设置命名空间，例如 "/px4_1" | Namespace can be set per instance, e.g., "/px4_1"


class OffboardControl(Node):
    """
    ROS2 节点：实现 PX4 离板模式控制（基于位置）
    ROS2 Node for PX4 offboard control (position-based)
    """

    def __init__(self):
        super().__init__('offboard_control_center')
        self.get_logger().info("🚀 [INIT] Initializing OffboardControl node...")

        # 配置 QoS：最佳努力传输 + 瞬时本地持久化（适用于 PX4 快速状态流）
        # 核心原则：Subscriber 的 QoS 要求不能比 Publisher 更“严格”。
        # reliability=RELIABLE 会显著增加 CPU、内存和网络开销。
        # Configure QoS: Best-effort reliability + transient local durability (suitable for PX4 fast streams)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------- 发布者 | Publishers ----------------
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{namespace}/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{namespace}/fmu/in/trajectory_setpoint', qos_profile)
        self.goto_setpoint_publisher = self.create_publisher(
            GotoSetpoint, f'{namespace}/fmu/in/goto_setpoint', qos_profile)
        
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(VehicleAttitudeSetpoint
          , f'{namespace}/fmu/in/vehicle_attitude_setpoint_v1', qos_profile)

        self.get_logger().info(f"[PUB] Created publishers under namespace: '{namespace or 'default'}'")

        # ---------------- 订阅者 | Subscribers ----------------
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{namespace}/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        self.get_logger().info("[SUB] Subscribed to vehicle_local_position_v1 and vehicle_status")

        # ---------------- 状态变量 | State Variables ----------------
        self.offboard_setpoint_counter = 0          # 心跳计数器 | Heartbeat counter
        self.takeoff_height = 2.0                   # 默认起飞高度 | Default takeoff altitude
        self.home_position = [0.0, 0.0, 0.0]        # Home 点（ENU 坐标系）| Home position in ENU
        self.vehicle_local_position_enu = VehicleLocalPosition()  # 存储转换后的 ENU 位置 | Store converted ENU position
        self.vehicle_local_position_received = False  # 是否收到有效位置 | Whether valid position received
        self.vehicle_status = VehicleStatus()         # 当前飞行器状态 | Current vehicle status
        self.control_mode = 'position'

        # 标志位与线程锁 | Flags & Thread Locks
        self.is_takeoff_complete = False
        self.target_reached = False
        self.lock = threading.Lock()

        # 节流日志时间记录表 | Throttled log timestamp dict
        self._last_log = {}
        self.target = (0.0, 0.0, 0.0, 0.0)  # (x, y, z, yaw) in ENU

        self.get_logger().info("✅ [INIT] OffboardControl initialized successfully!")


    # ---------------- 工具函数：节流日志 | Utility: Throttled Logging ----------------
    def throttle_log(self, interval_sec: float, msg: str, level: str = "info", tag: str = "default"):
        """
        节流打印函数：避免高频日志刷屏（如位置反馈、心跳）
        Throttled logging: prevent console flooding from high-frequency logs (e.g., position, heartbeat)
        
        参数 | Args:
        - interval_sec: 最小打印间隔（秒）| Minimum interval between logs (seconds)
        - msg: 日志内容 | Log message
        - level: 日志级别 ("info", "warn", "error") | Log level
        - tag: 日志标签，用于区分不同来源 | Log tag for source differentiation
        """
        now = time.time()
        if tag not in self._last_log or (now - self._last_log[tag]) > interval_sec:
            if level == "info":
                self.get_logger().info(msg)
            elif level == "warn":
                self.get_logger().warning(msg)
            elif level == "error":
                self.get_logger().error(msg)
            self._last_log[tag] = now


    # ---------------- 心跳机制 | Heartbeat Mechanism ----------------
    def heartbeat_thread_start(self):
        """启动心跳线程：以固定频率发布控制模式与轨迹点，维持 Offboard 模式"""
        """Start heartbeat thread: publish control mode & setpoints at fixed rate to maintain offboard mode"""
        self.stop_heartbeat = threading.Event()
        # 位置环与速度环运行频率为 50Hz,位置姿态控制超过没有意义。对于小型飞控，机载通信频率过高，会报cpu load too high错误，导致无法解锁
        self.heartbeat_hz = 20
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        self.get_logger().info(f"🔁 [HEARTBEAT] Started heartbeat thread at {self.heartbeat_hz} Hz")


    def heartbeat_loop(self):
        """心跳主循环：每 1/20 秒发送一次控制信号"""
        """Heartbeat main loop: send control signal every 1/20 second"""
        rate = 1.0 / float(self.heartbeat_hz)  #TODO 使用ros2时间相关API，如rate,实现更精确控制
        self.get_logger().debug("[HEARTBEAT] Entering heartbeat loop...")
        while not self.stop_heartbeat.is_set() and rclpy.ok():
            try:
                self.publish_offboard_control_heartbeat_signal(self.control_mode)
                self.publish_current_setpoint()
                self.offboard_setpoint_counter += 1
                # 节流打印心跳计数（每5秒一次）
                self.throttle_log(5.0, f"[HEARTBEAT] Published setpoint #{self.offboard_setpoint_counter}", tag="heartbeat")
            except Exception as e:
                self.get_logger().error(f"[HEARTBEAT] Exception in loop: {e}")
            time.sleep(rate)
        self.get_logger().info("⏹️ [HEARTBEAT] Heartbeat thread exiting")


    def publish_current_setpoint(self):
        """发布当前目标点（线程安全），根据控制模式自适应"""
        """Publish current target setpoint (thread-safe), adapt based on control mode"""
        with self.lock:
            mode = self.control_mode
            target = self.target
        if mode == 'position':
            x, y, z, yaw = target
            self.publish_trajectory_setpoint(position=[x, y, z], yaw=yaw)
        elif mode == 'velocity':
            vx, vy, vz, yawspeed = target
            self.publish_trajectory_setpoint(velocity=[vx, vy, vz], yawspeed=yawspeed)
        elif mode == 'attitude':
            roll, pitch, yaw, thrust = target
            q_d = self.euler_to_quaternion(roll, pitch, yaw)
            thrust_body = [0.0, 0.0, -thrust]  # 多旋翼假设，负 z 向上
            self.publish_attitude_setpoint(q_d, thrust_body, yaw_sp_move_rate=None)  # 或设置 yaw_sp_move_rate 如果需要
            self.publish_attitude_setpoint(roll, pitch, yaw, thrust)
        else:
            self.get_logger().warning(f"[SETPOINT] Unsupported control mode: {mode}")


    def set_control_mode(self, mode: str):
        """设置控制模式：'position', 'velocity', 'attitude'"""
        if mode in ['position', 'velocity', 'attitude']:
            with self.lock:
                self.control_mode = mode
            self.get_logger().info(f"🔄 [MODE] Switched to {mode} control mode")
        else:
            self.get_logger().error(f"❌ [MODE] Invalid control mode: {mode}")


    def update_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """更新位置目标点（线程安全）"""
        with self.lock:
            if self.control_mode != 'position':
                self.set_control_mode('position')
            old_target = self.target
            self.target = (float(x), float(y), float(z), float(yaw))
        self.get_logger().debug(f"🎯 [POSITION] Updated from {old_target} → {self.target} (ENU)")


    def update_velocity_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float):
        """更新速度目标点（线程安全）"""
        with self.lock:
            if self.control_mode != 'velocity':
                self.set_control_mode('velocity')
            old_target = self.target
            self.target = (float(vx), float(vy), float(vz), float(yawspeed))
        self.get_logger().debug(f"🎯 [VELOCITY] Updated from {old_target} → {self.target} (ENU)")


    def update_attitude_setpoint(self, roll: float, pitch: float, yaw: float, thrust: float):
        """更新姿态目标点（线程安全），使用欧拉角和推力"""
        with self.lock:
            if self.control_mode != 'attitude':
                self.set_control_mode('attitude')
            old_target = self.target
            self.target = (float(roll), float(pitch), float(yaw), float(thrust))
        self.get_logger().debug(f"🎯 [ATTITUDE] Updated from {old_target} → {self.target}")

    
    def request_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Send a vehicle command request."""
        '''non-blocking'''
        request = VehicleCommandSrv.Request()
        msg = VehicleCommand()
        # Ensure the parameters are floats
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        request.request = msg
        self.service_done = False
        self.service_result = None
        future = self.vehicle_command_client.call_async(request)
        #print(f'future:{future}')
        #不等待服务返回\服务返回后，执行 response_callback(future).适合：不需要额外参数传回调
        future.add_done_callback(partial(self.response_callback))
        self.get_logger().info('Command sent (non-blocking)')

    def response_callback(self, future):
            """Handle the response from the vehicle command service."""
            try:
                response = future.result()
                #print(f'response:{response}')
                reply = response.reply
                self.service_result = response.reply.result
                if self.service_result == reply.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.get_logger().info('Command accepted')
                elif self.service_result == reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                    self.get_logger().warning('Command temporarily rejected')
                elif self.service_result == reply.VEHICLE_CMD_RESULT_DENIED:
                    self.get_logger().warning('Command denied')
                elif self.service_result == reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
                    self.get_logger().warning('Command unsupported')
                elif self.service_result == reply.VEHICLE_CMD_RESULT_FAILED:
                    self.get_logger().warning('Command failed')
                elif self.service_result == reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
                    self.get_logger().warning('Command in progress')
                elif self.service_result == reply.VEHICLE_CMD_RESULT_CANCELLED:
                    self.get_logger().warning('Command cancelled')
                else:
                    self.get_logger().warning('Command reply unknown')
                self.service_done = True

            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

    async def request_vehicle_command_blocking(self, command, param1=0.0, param2=0.0, timeout_sec=5.0):
        """
        Send vehicle command and BLOCK (await) until response is received or timeout.

        This function:
        - Sends a command via call_async()
        - Awaits the service response using asyncio.wait_for()
        - Processes the result immediately (no callback needed)
        """
        # -------------------------
        # 1. 构造服务请求
        # -------------------------
        request = VehicleCommandSrv.Request()
        msg = VehicleCommand()

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        request.request = msg

        self.get_logger().info(f"Sending command {command} (blocking for response)...")

        # -------------------------
        # 2. 异步发送并等待响应
        # -------------------------
        future = self.vehicle_command_client.call_async(request)

        try:
            # 👇 阻塞等待（await）
            response = await asyncio.wait_for(future, timeout=timeout_sec)

        except asyncio.TimeoutError:
            self.get_logger().error(f"Command {command} timed out after {timeout_sec} seconds")
            return None

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

        # -------------------------
        # 3. 服务响应处理
        # -------------------------
        reply = response.reply
        result = reply.result
        self.service_result = result

        if result == reply.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info("Command accepted")
        elif result == reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            self.get_logger().warning("Command temporarily rejected")
        elif result == reply.VEHICLE_CMD_RESULT_DENIED:
            self.get_logger().warning("Command denied")
        elif result == reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
            self.get_logger().warning("Command unsupported")
        elif result == reply.VEHICLE_CMD_RESULT_FAILED:
            self.get_logger().warning("Command failed")
        elif result == reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
            self.get_logger().warning("Command in progress")
        elif result == reply.VEHICLE_CMD_RESULT_CANCELLED:
            self.get_logger().warning("Command cancelled")
        else:
            self.get_logger().warning("Command reply unknown")

        return result


    # ---------------- 坐标系转换 | Coordinate Conversion ----------------
    def ned_to_enu(self, x_ned, y_ned, z_ned):
        """将 NED 坐标转为 ENU 坐标系"""
        """Convert NED coordinates to ENU"""
        return y_ned, x_ned, -z_ned

    def enu_to_ned(self, x_enu, y_enu, z_enu):
        """将 ENU 坐标转为 NED 坐标系"""
        """Convert ENU coordinates to NED"""
        return y_enu, x_enu, -z_enu

    def normalize_yaw(self, yaw_diff: float) -> float:
        """将航向角差归一化到 [-π, π] 并返回绝对值"""
        """Normalize yaw difference to [-π, π] and return absolute value"""
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        return abs(yaw_diff)
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> list[float]:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [float(w), float(x), float(y), float(z)]  # 正则化可选


    # ---------------- ROS2 回调函数 | ROS2 Callbacks ----------------
    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """
        位置回调：接收 NED 坐标，转换为 ENU 并存储
        Position callback: receive NED, convert to ENU, and store
        """
        try:
            x_enu, y_enu, z_enu = self.ned_to_enu(msg.x, msg.y, msg.z)
            heading_enu = -msg.heading + math.radians(90)  # NED heading → ENU heading
            with self.lock:
                self.vehicle_local_position_enu.x = x_enu
                self.vehicle_local_position_enu.y = y_enu
                self.vehicle_local_position_enu.z = z_enu
                self.vehicle_local_position_enu.heading = heading_enu
                self.vehicle_local_position_received = True
            self.throttle_log(
                2.0,
                f"[POSITION] ENU=({x_enu:.2f}, {y_enu:.2f}, {z_enu:.2f}), "
                f"heading={math.degrees(heading_enu):.1f}°",
                tag="position"
            )
        except Exception as e:
            self.get_logger().error(f"[POSITION] Callback error: {e}")


    def vehicle_status_callback(self, msg: VehicleStatus):
        """状态回调：更新飞行器导航与解锁状态"""
        """Status callback: update navigation and arming state"""
        with self.lock:
            old_nav = getattr(self.vehicle_status, 'nav_state', 'N/A')
            old_arm = getattr(self.vehicle_status, 'arming_state', 'N/A')
            self.vehicle_status = msg
        self.throttle_log(
            5.0,
            f"[STATUS] nav_state={msg.nav_state} (was {old_nav}), "
            f"arming_state={msg.arming_state} (was {old_arm})",
            tag="status"
        )


    # ---------------- 飞行器命令 | Vehicle Commands ----------------
    def arm(self):
        """发送解锁命令，并记录 Home 点"""
        """Send arm command and record home position"""
        self.get_logger().info("🔓 Sending ARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("✅ Arm command sent")

        if not self.vehicle_local_position_received:
            self.get_logger().warning("⚠️ Waiting for first local position message...")

        # 等待有效位置
        while not self.vehicle_local_position_received and rclpy.ok():
            time.sleep(0.5)

        with self.lock:
            self.home_position = [
                self.vehicle_local_position_enu.x,
                self.vehicle_local_position_enu.y,
                self.vehicle_local_position_enu.z,
            ]
        self.get_logger().info(f"🏠 Home position recorded: {self.home_position} (ENU)")


    def disarm(self):
        """发送上锁命令"""
        """Send disarm command"""
        self.get_logger().info("🔒 Sending DISARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("✅ Disarm command sent")


    def arm_srv(self):
        self.get_logger().info('Requesting arm')
        self.request_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        while not self.service_done:
            time.sleep(0.05)
            self.get_logger().info('waiting for arm service to be done')
        self.get_logger().info('arm service has done')
        if self.service_result == 0:
                self.get_logger().info('Vehicle is armed')
                self.state = 'armed'
        # record takeoff position and RTL position
        with self.lock:
            self.home_position = [
                self.vehicle_local_position_enu.x,
                self.vehicle_local_position_enu.y,
                self.vehicle_local_position_enu.z,
            ]
        self.get_logger().info(f"🏠 Home position recorded: {self.home_position} (ENU)")


    def disarm_srv(self):
        self.get_logger().info('Requesting disarm')
        self.request_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)




    def engage_offboard_mode(self, prewarm_count=10, prewarm_timeout=5.0):
        """
        进入 Offboard 模式前，需先预热（发送至少若干个控制点）
        Must pre-warm by sending several setpoints before engaging offboard mode
        """
        self.get_logger().info(f"🔄 Engaging OFFBOARD mode (prewarm: {prewarm_count} msgs or {prewarm_timeout}s)")

        start = time.time()
        while self.offboard_setpoint_counter < prewarm_count and (time.time() - start) < prewarm_timeout and rclpy.ok():
            time.sleep(0.05)

        if self.offboard_setpoint_counter < prewarm_count:
            self.get_logger().warning(
                f"⚠️ Prewarm insufficient: only {self.offboard_setpoint_counter}/{prewarm_count} setpoints sent"
            )
        else:
            self.get_logger().info(f"✅ Prewarm complete: {self.offboard_setpoint_counter} setpoints sent")

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("✅ Switched to OFFBOARD mode!")


    def engage_offboard_mode_srv(self, prewarm_count=10, prewarm_timeout=5.0):
        """
        进入 Offboard 模式前，需先预热（发送至少若干个控制点）
        Must pre-warm by sending several setpoints before engaging offboard mode
        """
        self.get_logger().info(f"🔄 Engaging OFFBOARD mode (prewarm: {prewarm_count} msgs or {prewarm_timeout}s)")

        start = time.time()
        while self.offboard_setpoint_counter < prewarm_count and (time.time() - start) < prewarm_timeout and rclpy.ok():
            time.sleep(0.05)

        if self.offboard_setpoint_counter < prewarm_count:
            self.get_logger().warning(
                f"⚠️ Prewarm insufficient: only {self.offboard_setpoint_counter}/{prewarm_count} setpoints sent"
            )
        else:
            self.get_logger().info(f"✅ Prewarm complete: {self.offboard_setpoint_counter} setpoints sent")
        self.get_logger().info('Requesting switch to Offboard mode')
        self.request_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)



    def hover(self, duration: float, timeout: float = None) -> bool:
        """
        阻塞式悬停：保持当前位置和航向，持续指定时长
        Blocking hover: maintain current position and heading for the specified duration
        
        参数 | Args:
        - duration: 悬停时长（秒）| Hover duration (seconds)
        - timeout: 可选超时（秒），默认等于 duration + 10 | Optional timeout (seconds), defaults to duration + 10
        
        返回 | Returns:
        - bool: 是否成功完成悬停 | True if hover completed successfully
        """
        if duration <= 0:
            self.get_logger().error("❌ Hover duration must be positive!")
            return False

        # 默认超时为 duration + 缓冲
        if timeout is None:
            timeout = duration + 10.0

        # 获取当前 ENU 位置和航向
        with self.lock:
            if not self.vehicle_local_position_received:
                self.get_logger().warning("⚠️ No valid position received; cannot hover.")
                return False
            cx = self.vehicle_local_position_enu.x
            cy = self.vehicle_local_position_enu.y
            cz = self.vehicle_local_position_enu.z
            ch = self.vehicle_local_position_enu.heading

        # 更新 setpoint 为当前位置（position 模式）
        self.update_position_setpoint(cx, cy, cz, ch)
        self.get_logger().info(f"🛸 Starting hover at ENU ({cx:.2f}, {cy:.2f}, {cz:.2f}), yaw={math.degrees(ch):.1f}° for {duration:.1f}s")

        start = time.time()
        last_log = start
        while rclpy.ok() and (time.time() - start) < timeout:
            elapsed = time.time() - start
            if elapsed >= duration:
                self.get_logger().info("✅ Hover duration completed!")
                return True

            # 节流日志剩余时间
            if time.time() - last_log >= 1.0:
                self.throttle_log(
                    1.0,
                    f"[HOVER] Elapsed: {elapsed:.1f}/{duration:.1f}s",
                    tag="hover"
                )
                last_log = time.time()

            time.sleep(0.1)

        self.get_logger().warning("⚠️ Hover timed out!")
        return False
    
    def land(self, latitude: float = np.nan, longitude: float = np.nan, altitude: float = 0.0, yaw: float = np.nan, abort_alt: float = 0.0, land_mode: int = 0, timeout: float = 60.0) -> bool:
        """
        阻塞式降落：发送 MAV_CMD_NAV_LAND 命令，并在指定位置降落
        Blocking land: send MAV_CMD_NAV_LAND command to land at specified location
        如果在 Offboard 模式下，commander 会覆盖外部设定点，确保切换到内部控制
        
        参数 | Args:
        - latitude: 纬度（NaN 使用当前位置）| Latitude (NaN for current position)
        - longitude: 经度（NaN 使用当前位置）| Longitude (NaN for current position)
        - altitude: 降落高度（地面高度）| Landing altitude (ground level)
        - yaw: 期望偏航角（NaN 使用系统默认）| Desired yaw angle (NaN for system default)
        - abort_alt: 中止高度（0=默认）| Abort altitude (0=undefined/system default)
        - land_mode: 降落模式（0=正常，参考 PRECISION_LAND_MODE）| Land mode (e.g., PRECISION_LAND_MODE)
        - timeout: 超时时间（秒）| Timeout (seconds)
        
        返回 | Returns:
        - bool: 是否成功降落 | True if landed successfully
        """
        self.get_logger().info(f"🛬 Sending LAND command at lat={latitude}, lon={longitude}, alt={altitude:.2f}m, yaw={yaw if not np.isnan(yaw) else 'default'}")

        # 发送 VehicleCommand: MAV_CMD_NAV_LAND (21)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            param1=float(abort_alt),
            param2=float(land_mode),
            param3=0.0,  # Empty
            param4=float(yaw),
            param5=float(latitude),
            param6=float(longitude),
            param7=float(altitude)
        )

        # 阻塞等待降落完成：检查高度 ≈ 0 或状态为 landed
        start = time.time()
        last_log = start
        while rclpy.ok() and time.time() - start < timeout:
            with self.lock:
                cz = self.vehicle_local_position_enu.z
                nav_state = self.vehicle_status.nav_state
            remaining_time = timeout - (time.time() - start)

            # 节流日志
            if time.time() - last_log >= 1.0:
                self.throttle_log(
                    1.0,
                    f"[LAND] Altitude: {cz:.2f}m, nav_state={nav_state}, remaining time: {remaining_time:.1f}s",
                    tag="land"
                )
                last_log = time.time()

            # 检查是否降落：高度 < 0.1m 或 nav_state 表示 landed (VehicleStatus.NAVIGATION_STATE_AUTO_LAND 或 arming_state disarmed)
            if cz < 0.1 or nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND or self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info("✅ Landing complete!")
                return True

            time.sleep(0.1)

        self.get_logger().warning("⚠️ Land timed out!")
        return False


    # ---------------- 发布辅助函数 | Publish Helpers ----------------
    def publish_offboard_control_heartbeat_signal(self, control_mode='position'):
        """发布 Offboard 控制模式信号（维持模式激活）"""
        """Publish offboard control mode signal (to keep mode active)"""
        msg = OffboardControlMode()
        msg.position = (control_mode == 'position')
        msg.velocity = (control_mode == 'velocity')
        msg.acceleration = (control_mode == 'acceleration')  # 如果支持
        msg.attitude = (control_mode == 'attitude')
        msg.body_rate = (control_mode == 'body_rate')  # 如果支持
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)


    def publish_trajectory_setpoint(
        self,
        position: list[float] = None,
        velocity: list[float] = None,
        acceleration: list[float] = None,
        jerk: list[float] = None,
        yaw: float = None,
        yawspeed: float = None
    ):
            """发布轨迹点（ENU 输入 → NED 发布），自适应位置、速度、加速度等控制"""
            """Publish trajectory setpoint (ENU input → NED published), adaptive to position, velocity, acceleration, etc."""
            msg = TrajectorySetpoint()
            nan3 = [np.nan] * 3  # 默认 NaN 数组，表示不控制

            # 处理位置
            if position is not None:
                x_enu, y_enu, z_enu = position
                x_ned, y_ned, z_ned = self.enu_to_ned(x_enu, y_enu, z_enu)
                msg.position = [float(x_ned), float(y_ned), float(z_ned)]
            else:
                msg.position = nan3

            # 处理速度
            if velocity is not None:
                vx_enu, vy_enu, vz_enu = velocity
                vx_ned, vy_ned, vz_ned = self.enu_to_ned(vx_enu, vy_enu, vz_enu)  # 速度转换类似位置（方向变换）
                msg.velocity = [float(vx_ned), float(vy_ned), float(vz_ned)]
            else:
                msg.velocity = nan3

            # 处理加速度
            if acceleration is not None:
                ax_enu, ay_enu, az_enu = acceleration
                ax_ned, ay_ned, az_ned = self.enu_to_ned(ax_enu, ay_enu, az_enu)
                msg.acceleration = [float(ax_ned), float(ay_ned), float(az_ned)]
            else:
                msg.acceleration = nan3

            # 处理 jerk（仅用于日志，不影响控制）
            if jerk is not None:
                jx_enu, jy_enu, jz_enu = jerk
                jx_ned, jy_ned, jz_ned = self.enu_to_ned(jx_enu, jy_enu, jz_enu)
                msg.jerk = [float(jx_ned), float(jy_ned), float(jz_ned)]
            else:
                msg.jerk = nan3

            # 处理 yaw
            if yaw is not None:
                msg.yaw = float(-yaw + math.radians(90))  # ENU yaw → NED yaw
            else:
                msg.yaw = np.nan

            # 处理 yawspeed
            if yawspeed is not None:
                msg.yawspeed = -float(yawspeed)  # yawspeed 不需要转换（角速度标量）
            else:
                msg.yawspeed = np.nan

            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(msg)

            # 调试日志：仅记录非 NaN 值
            log_str = "[PUB] TrajectorySetpoint NED: "
            if position is not None:
                log_str += f"pos=({msg.position[0]:.2f},{msg.position[1]:.2f},{msg.position[2]:.2f}), "
            if velocity is not None:
                log_str += f"vel=({msg.velocity[0]:.2f},{msg.velocity[1]:.2f},{msg.velocity[2]:.2f}), "
            if acceleration is not None:
                log_str += f"acc=({msg.acceleration[0]:.2f},{msg.acceleration[1]:.2f},{msg.acceleration[2]:.2f}), "
            if jerk is not None:
                log_str += f"jerk=({msg.jerk[0]:.2f},{msg.jerk[1]:.2f},{msg.jerk[2]:.2f}), "
            if yaw is not None:
                log_str += f"yaw={msg.yaw:.2f}, "
            if yawspeed is not None:
                log_str += f"yawspeed={msg.yawspeed:.2f}"
            self.get_logger().debug(log_str)

    def publish_attitude_setpoint(
        self,
        q_d: list[float],
        thrust_body: list[float],
        yaw_sp_move_rate: float = None
    ):
            """发布姿态设定点（基于四元数和推力）"""
            """Publish attitude setpoint (based on quaternion and thrust)"""
            msg = VehicleAttitudeSetpoint()
            msg.q_d = [float(q) for q in q_d]  # 期望四元数
            msg.thrust_body = [float(t) for t in thrust_body]  # 体坐标系下归一化推力 [-1,1]
            
            # 处理 yaw_sp_move_rate
            if yaw_sp_move_rate is not None:
                msg.yaw_sp_move_rate = float(yaw_sp_move_rate)
            else:
                msg.yaw_sp_move_rate = np.nan  # 默认不控制

            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_attitude_setpoint_publisher.publish(msg)  

            log_str = f"[PUB] VehicleAttitudeSetpoint: q_d={msg.q_d}, thrust_body={msg.thrust_body}"
            if yaw_sp_move_rate is not None:
                log_str += f", yaw_sp_move_rate={msg.yaw_sp_move_rate:.2f}"
            self.get_logger().debug(log_str)

    def publish_vehicle_command(self, command, **params):
            msg = VehicleCommand()
            msg.command = command
            msg.param1 = params.get("param1", 0.0)
            msg.param2 = params.get("param2", 0.0)
            msg.param3 = params.get("param3", 0.0)
            msg.param4 = params.get("param4", 0.0)
            msg.param5 = params.get("param5", 0.0)
            msg.param6 = params.get("param6", 0.0)
            msg.param7 = params.get("param7", 0.0)
            # 假设 namespace="/px4_1"
            try:
                sys_id = int(namespace.strip('/px4_')) + 1 
            except:
                sys_id = 1
            msg.target_system = sys_id
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            try:
                self.vehicle_command_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"publish_vehicle_command error: {e}")


    # ---------------- 高层飞行控制 | High-Level Motion Control ----------------
    def takeoff(self, takeoff_height=2.0, timeout=20.0) -> bool:
        """阻塞式起飞：从 Home 点垂直上升至指定高度"""
        '''使用 setpoint 来模拟起飞路径。'''
        """Blocking takeoff: ascend vertically from home to specified altitude"""
        if takeoff_height <= 0:
            self.get_logger().error("❌ Takeoff height must be positive!")
            return False

        with self.lock:
            home_z = self.home_position[2]
        target_alt = home_z + takeoff_height
        current_heading = self.vehicle_local_position_enu.heading if self.vehicle_local_position_received else 0.0
        self.update_position_setpoint(self.home_position[0], self.home_position[1], target_alt, current_heading)

        self.get_logger().info(f"🛫 Starting takeoff to {target_alt:.2f} m (from {home_z:.2f} m)")

        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            with self.lock:
                current_z = self.vehicle_local_position_enu.z
            remaining = target_alt - current_z
            self.throttle_log(
                1.0,
                f"[TAKEOFF] Altitude: {current_z:.2f}/{target_alt:.2f} m, Δ={remaining:.2f} m",
                tag="takeoff"
            )
            if remaining <= 0.1:
                self.get_logger().info("✅ Takeoff complete!")
                return True
            time.sleep(0.1)

        self.get_logger().warning("⚠️ Takeoff timed out!")
        return False
    
    def simulated_land(self, descent_rate: float = -0.5, ground_tolerance: float = 0.1, timeout: float = 60.0) -> bool:
        """
        阻塞式模拟降落：使用 setpoint 逐步降低高度，实现平稳降落（需在 Offboard 模式下）
        Blocking simulated land: use setpoints to gradually decrease altitude for smooth landing (requires Offboard mode)
        
        参数 | Args:
        - descent_rate: 下降速度（m/s），负值表示向下 | Descent rate (m/s), negative for downward
        - ground_tolerance: 地面容忍高度（m） | Ground tolerance altitude (m)
        - timeout: 超时时间（秒） | Timeout (seconds)
        
        返回 | Returns:
        - bool: 是否成功降落 | True if landed successfully
        """
        if descent_rate >= 0:
            self.get_logger().error("❌ Descent rate must be negative for landing!")
            return False

        # 获取当前 ENU 位置和航向（假设在 position/velocity 模式）
        with self.lock:
            if not self.vehicle_local_position_received:
                self.get_logger().warning("⚠️ No valid position received; cannot land.")
                return False
            cx = self.vehicle_local_position_enu.x
            cy = self.vehicle_local_position_enu.y
            cz = self.vehicle_local_position_enu.z
            ch = self.vehicle_local_position_enu.heading

        # 切换到 velocity 模式以控制下降速度（更平稳）
        self.set_control_mode('velocity')
        self.get_logger().info(f"🛬 Starting simulated land from altitude {cz:.2f}m with descent rate {descent_rate:.2f} m/s")

        start = time.time()
        last_log = start
        target_z = 0.0  # 目标地面高度
        while rclpy.ok() and time.time() - start < timeout:
            with self.lock:
                current_z = self.vehicle_local_position_enu.z
                nav_state = self.vehicle_status.nav_state

            # 计算剩余距离并更新 velocity setpoint（水平速度 0，垂直 descent_rate，yawspeed 0）
            remaining_dist = current_z - target_z
            vz = max(descent_rate, -remaining_dist * 2.0)  # 接近地面时减速（简单线性减速）
            self.update_velocity_setpoint(0.0, 0.0, vz, 0.0)  # vx=0, vy=0, vz=下降, yawspeed=0

            # 节流日志
            if time.time() - last_log >= 1.0:
                self.throttle_log(
                    1.0,
                    f"[SIM_LAND] Altitude: {current_z:.2f}m, vz={vz:.2f} m/s, remaining: {remaining_dist:.2f}m",
                    tag="sim_land"
                )
                last_log = time.time()

            # 检查是否到达地面
            if current_z <= ground_tolerance:
                # 停止下降，发送零速度
                self.update_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                self.get_logger().info("✅ Simulated landing complete! Altitude near ground.")
                return True

            time.sleep(0.05)  # 高频循环以确保平稳

        self.get_logger().warning("⚠️ Simulated land timed out!")
        # 超时后恢复悬停
        self.update_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
        return False


    def fly_to_trajectory_setpoint(self, x, y, z, yaw, timeout=60.0) -> bool:
        """阻塞式飞往目标点（ENU 坐标）"""
        """Blocking flight to target point (in ENU coordinates)"""
        self.update_position_setpoint(x, y, z, yaw)
        self.get_logger().info(f"✈️ Flying to ENU target: ({x:.2f}, {y:.2f}, {z:.2f}), yaw={math.degrees(yaw):.1f}°")

        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            with self.lock:
                cx = self.vehicle_local_position_enu.x
                cy = self.vehicle_local_position_enu.y
                cz = self.vehicle_local_position_enu.z
                ch = self.vehicle_local_position_enu.heading
            dist = math.sqrt((cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2)
            yaw_diff = self.normalize_yaw(ch - yaw)
            self.throttle_log(
                1.0,
                f"[FLYTO] Remaining distance: {dist:.2f} m, yaw diff: {yaw_diff:.2f} rad ({math.degrees(yaw_diff):.1f}°)",
                tag="flyto"
            )
            if dist < DISTANCE_TOLERANCE and yaw_diff < YAW_TOLERANCE:
                self.get_logger().info("✅ Target reached!")
                return True
            time.sleep(0.1)

        self.get_logger().warning("⚠️ fly_to_trajectory_setpoint timed out!")
        return False


# ---------------- 高层封装类：Vehicle | High-Level Wrapper: Vehicle ----------------
class Vehicle:
    """
    高层封装类：隐藏 ROS2 生命周期管理，提供简洁的 Python 控制接口
    High-level wrapper: hides ROS2 lifecycle details, provides clean Python API
    """

    def __init__(self):
        print("🌍 Initializing ROS2...")
        if not rclpy.ok():  # 关键修复：检查是否已初始化
           rclpy.init()
        self.drone = OffboardControl()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.drone)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.drone.get_logger().info("🌀 Vehicle node spinning in background thread")
        self.drone.heartbeat_thread_start()
        self.drone.engage_offboard_mode()

    def close(self):
        """关闭所有线程与 ROS2 资源"""
        """Shut down all threads and ROS2 resources"""
        self.drone.get_logger().info("🛑 Shutting down Vehicle...")
        self.drone.stop_heartbeat.set()
        if self.drone.heartbeat_thread.is_alive():
            self.drone.heartbeat_thread.join(timeout=3.0)
        rclpy.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=3.0)
        self.drone.destroy_node()
        self.executor.shutdown()
        print("✅ Vehicle shutdown complete!")


    def __enter__(self):
        return self


    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

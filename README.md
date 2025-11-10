offboard_control_lib 使用说明（详细版）
Detailed Usage Guide for offboard_control_lib

1. 概述 / Overview

offboard_control_lib 是一个基于 ROS 2（rclpy）开发的无人机离线控制库，封装了 PX4 飞控系统与 ROS 2 接口之间的通信逻辑。该库提供简洁、高层的 API，用于实现无人机的起飞、飞行控制、悬停、降落及安全退出等操作。

offboard_control_lib is a ROS 2 (rclpy)-based library for unmanned aerial vehicle (UAV) offboard control. It encapsulates the communication logic between the PX4 flight stack and ROS 2 interfaces, providing a clean, high-level API for essential operations such as arming, takeoff, trajectory control, hovering, landing, and safe shutdown.

2. 安装依赖 / Installation Dependencies

确保已安装以下组件：
Ensure the following components are installed:
ROS 2（推荐 Humble 或 Iron）
ROS 2 (Humble or Iron recommended)
PX4 SITL（软件在环仿真）或真实飞控硬件
PX4 SITL (Software-in-the-Loop simulation) or real flight controller hardware
px4_msgs（需与 PX4 版本匹配）
px4_msgs (must match your PX4 version)
rclpy
rclpy
本库 offboard_control_lib（需正确放置于 Python 路径中或通过 setup.py 安装）
This library offboard_control_lib (must be in your Python path or installed via setup.py)

3. 核心类：Vehicle / Core Class: Vehicle

Vehicle 类继承自 rclpy.node.Node，自动初始化 ROS 2 节点，并内部管理以下子模块：

The Vehicle class inherits from rclpy.node.Node, automatically initializing a ROS 2 node and managing the following internal components:
drone：实际控制接口对象，提供飞行指令方法
drone: The actual control interface object that provides flight command methods
订阅器：接收飞控状态（如位置、姿态、是否就绪等）
Subscribers: Receive flight controller status (e.g., position, attitude, readiness)
发布器：发送轨迹设定点（TrajectorySetpoint）、飞控模式命令（OffboardControlMode）等
Publishers: Send trajectory setpoints (TrajectorySetpoint) and mode commands (OffboardControlMode)
定时器：周期性发送控制指令以维持 Offboard 模式
Timers: Periodically publish control commands to maintain Offboard mode
初始化 / Initialization

python
vehicle = Vehicle()

此调用会：
This call will:
创建名为 offboard_control_node 的 ROS 2 节点
Create a ROS 2 node named offboard_control_node
初始化所有必要的发布者/订阅者
Initialize all required publishers and subscribers
启动后台定时器（通常为 20 Hz）以维持 Offboard 控制链路
Start a background timer (typically at 20 Hz) to sustain the Offboard control link
⚠️ 注意：Vehicle 实例必须在 rclpy.init() 之后创建（通常由 main() 函数隐式处理）。
⚠️ Note: The Vehicle instance must be created after rclpy.init() (usually handled implicitly in main()).

4. 主要 API 方法（通过 vehicle.drone 调用） / Key API Methods (via vehicle.drone)
4.1 arm() → bool
功能 / Function: 解锁电机（Arming）
Unlock motors (Arming)
返回值 / Returns: 成功返回 True，否则 False
Returns True on success, otherwise False
前提条件 / Prerequisites: 飞控处于就绪状态（如 GPS 锁定、IMU 校准完成等）
Flight controller must be ready (e.g., GPS fix, IMU calibrated)
4.2 takeoff(target_altitude: float) → bool
功能 / Function: 执行自动起飞至指定高度（单位：米）
Perform automatic takeoff to a specified altitude (in meters)
参数 / Parameters:
target_altitude: 目标高度（相对于起飞点），建议 ≥ 1.5 米以确保安全
Target altitude relative to takeoff point; ≥1.5 m recommended for safety
行为 / Behavior:
自动切换至 Offboard 模式
Automatically switches to Offboard mode
垂直上升至目标高度并悬停
Ascends vertically and hovers at target altitude
返回值 / Returns: 成功到达目标高度并稳定后返回 True
Returns True once the target altitude is reached and stabilized
4.3 fly_to_trajectory_setpoint(x: float, y: float, z: float, yaw: float) → None
功能 / Function: 飞往指定的 NED 坐标系下的位置和偏航角
Fly to a specified position and yaw in the NED frame
参数 / Parameters:
x: 北向位置（m）
North position (m)
y: 东向位置（m）
East position (m)
z: 高度（m）— 注意：本库已封装，传入正值表示高度（如 z=2.0 表示 2 米高）
Altitude (m) — Note: This library abstracts the NED convention; pass positive values for altitude (e.g., z=2.0 means 2 meters above ground)
yaw: 偏航角（弧度），0 表示机头朝北
Yaw angle (radians); 0 means facing north
说明 / Notes:
此方法为非阻塞调用，仅发送一次设定点
This is a non-blocking call that sends a single setpoint
若需持续控制，应在循环中定期调用，或使用更高层封装（如路径跟踪）
For sustained control, call periodically in a loop or use higher-level wrappers (e.g., waypoint tracking)
4.4 disarm() → bool
功能 / Function: 锁定电机（Disarming）
Lock motors (Disarming)
返回值 / Returns: 成功返回 True
Returns True on success
4.5 close()
功能 / Function: 释放资源，停止定时器，关闭节点
Release resources, stop timers, and shut down the node
使用场景 / Usage: 程序退出前必须调用（建议放在 finally 块中）
Must be called before program exit (recommended in a finally block)

5. 使用示例（完整 main 文件） / Usage Example (Complete main File)

python
#!/usr/bin/env python3

from offboard_control_lib import Vehicle

def main():

# 创建 Vehicle 实例（它是 rclpy.node.Node 的子类）
# Create a Vehicle instance (subclass of rclpy.node.Node)
vehicle = Vehicle()

try:
# 解锁无人机
# Arm the drone
if not vehicle.drone.arm():
print("Arming failed!")
return

# 起飞至 2.0 米高度
# Take off to 2.0 meters
if vehicle.drone.takeoff(2.0):
print("Takeoff successful. Flying to target position...")
# 飞往 (5.0, 0.0, 2.0)，偏航角为 0 弧度（朝北）
# Fly to (5.0, 0.0, 2.0) with yaw = 0 rad (facing north)
vehicle.drone.fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0)
# 可在此处添加 sleep 或等待逻辑以保持飞行
# Add sleep or waiting logic here if needed to maintain flight
else:
print("Takeoff failed!")

# 安全降落并锁定电机
# Safely land and disarm
vehicle.drone.disarm()

except KeyboardInterrupt:
print("\nKeyboardInterrupt received. Disarming...")
vehicle.drone.disarm()
finally:
# 清理 ROS 2 资源
# Clean up ROS 2 resources
vehicle.close()


if __name__ == '__main__':
main()

6. 注意事项 / Important Notes

1. 坐标系 / Coordinate System:
本库对用户隐藏了 PX4 的 NED（北-东-下）坐标系细节。用户传入的 z 为正数表示高度（如 2.0 表示 2 米高），内部自动转换为 -2.0 发送给 PX4。
This library abstracts away PX4’s NED (North-East-Down) coordinate system. Users provide positive z values for altitude (e.g., 2.0 = 2 meters high); the library internally converts this to -2.0 for PX4.

2. Offboard 模式要求 / Offboard Mode Requirements:
必须以 ≥ 2 Hz 的频率发送控制指令，否则飞控会自动退出 Offboard 模式
Control commands must be sent at ≥2 Hz, or PX4 will automatically exit Offboard mode
本库通过内部定时器自动维持此频率
This library maintains this rate automatically via an internal timer


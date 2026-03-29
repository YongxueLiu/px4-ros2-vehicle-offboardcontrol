#!/usr/bin/env python3
import time
import math
from offboard_control_lib import Vehicle  # 假设库已安装到 PYTHONPATH 或 ROS2 package 中

def main():
    # 创建 Vehicle：会在内部初始化 rclpy、节点并在后台 spin
    vehicle = Vehicle()

    try:
        # 1. 解锁（非阻塞的发布解锁命令 + 等待位置）
        vehicle.drone.arm()

        # 2. 起飞到 2.0 m（阻塞，直到到达或超时）
        if vehicle.drone.takeoff(2.0):
            # 3. 四边形航线（ENU 坐标系）
            side = 5.0
            z = 2.0
            # 4 个点，顺时针
            waypoints = [
                (side, 0.0, z, 0.0),
                (side, side, z, math.pi/2),
                (0.0, side, z, math.pi),
                (0.0, 0.0, z, -math.pi/2),
            ]

            for (x, y, z, yaw) in waypoints:
                success = vehicle.drone.fly_to_trajectory_setpoint(x, y, z, yaw, timeout=60.0)
                if not success:
                    vehicle.drone.get_logger().warning("Waypt not reached, aborting mission.")
                    break

        # 5. 降落并上锁
        vehicle.drone.land()
        time.sleep(1.0)
        vehicle.drone.disarm()

    except KeyboardInterrupt:
        vehicle.drone.get_logger().info("User interrupt, attempting safe shutdown...")

    finally:
        # 确保资源释放
        vehicle.close()

if __name__ == "__main__":
    main()


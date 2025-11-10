#!/usr/bin/env python3

import rclpy
from offboard_control_lib import Vehicle

def main():
   

    # 创建 Vehicle 实例（现在它是 rclpy.node.Node 的子类）
    vehicle = Vehicle()

    try:
        vehicle.drone.arm()
        if vehicle.drone.takeoff(2.0):
            vehicle.drone.fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0)  # x, y, z, yaw
        vehicle.drone.disarm()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        vehicle.close()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

from offboard_control_lib import Vehicle

if __name__ == '__main__':
    with Vehicle() as vehicle:
        vehicle.drone.arm()
        if vehicle.drone.takeoff(2.0):
            vehicle.drone.fly_to_trajectory_setpoint(0.0, 0.0, 5.0, 0.0)  # Example position (x, y, z, yaw)
            vehicle.drone.hover(2)
        vehicle.drone.simulated_land()
        vehicle.drone.disarm()
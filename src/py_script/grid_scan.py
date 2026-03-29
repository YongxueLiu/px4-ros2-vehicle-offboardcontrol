#!/usr/bin/env python3
import math
import time
import threading
from typing import List, Tuple
from offboard_control_lib import Vehicle


class PhotoManager:
    def __init__(self, vehicle: Vehicle,
                 time_interval: float = 2.0,
                 distance_interval: float = None):
        self.vehicle = vehicle
        self.time_interval = time_interval
        self.distance_interval = distance_interval
        self._running = False
        self._thread = None
        self._last_position = None
        self._accumulated_distance = 0.0

    def takephoto(self, x: float, y: float, z: float):
        timestamp = time.strftime("%H:%M:%S")
        print(f"📸 [PHOTO] {timestamp} | ({x:.2f}, {y:.2f}, {z:.2f})")

    def _get_current_position(self):
        pos = self.vehicle.drone.vehicle_local_position_enu
        if pos is None:
            return None
        return pos.x, pos.y, pos.z

    def _distance(self, p1, p2):
        return math.sqrt(
            (p1[0] - p2[0]) ** 2 +
            (p1[1] - p2[1]) ** 2 +
            (p1[2] - p2[2]) ** 2
        )

    def _distance_worker(self):
        while self._running:
            pos = self._get_current_position()
            if pos is None:
                time.sleep(0.1)
                continue

            if self._last_position is None:
                self._last_position = pos
                self.takephoto(*pos)
                continue

            d = self._distance(pos, self._last_position)
            self._accumulated_distance += d
            self._last_position = pos

            if self._accumulated_distance >= self.distance_interval:
                self.takephoto(*pos)
                self._accumulated_distance = 0.0

            time.sleep(0.1)

    def start(self):
        if self._running:
            return
        self._running = True
        print("📷 启动等距离连续拍照线程")
        self._thread = threading.Thread(target=self._distance_worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        print("📷 拍照线程已停止")


class GridScanner:
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle

    # ================== 关键修复1：东西方向扫描 ==================
    def generate_ew_waypoints(self, origin_x, origin_y, z,
                              width, height, spacing):
        """东西方向往返扫描（主方向 X）"""
        cols = max(1, int(math.ceil(width / spacing)))
        rows = max(1, int(math.ceil(height / spacing)))

        wps = []
        for r in range(rows):
            y = origin_y + r * spacing
            if r % 2 == 0:
                xs = [origin_x + i * spacing for i in range(cols)]
            else:
                xs = [origin_x + (cols - 1 - i) * spacing for i in range(cols)]
            for x in xs:
                wps.append((x, y, z))
        return wps

    # ================== 关键修复2：真正的南北扫描 ==================
    def generate_ns_waypoints(self, origin_x, origin_y, z,
                              width, height, spacing):
        """南北方向往返扫描（主方向 Y）"""
        cols = max(1, int(math.ceil(width / spacing)))
        rows = max(1, int(math.ceil(height / spacing)))

        wps = []
        for c in range(cols):
            x = origin_x + c * spacing
            if c % 2 == 0:
                ys = [origin_y + i * spacing for i in range(rows)]
            else:
                ys = [origin_y + (rows - 1 - i) * spacing for i in range(rows)]
            for y in ys:
                wps.append((x, y, z))
        return wps

    def fly_waypoints_blocking(self, waypoints):
        for i, (x, y, z) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                nx, ny, _ = waypoints[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0

            self.vehicle.drone.get_logger().info(
                f"🚁 航点 {i+1}/{len(waypoints)} → ({x:.2f}, {y:.2f}, {z:.2f})"
            )

            ok = self.vehicle.drone.fly_to_trajectory_setpoint(
                x, y, z, yaw, timeout=120.0
            )
            if not ok:
                self.vehicle.drone.get_logger().warning("航点未到达，任务中止")
                return False
        return True

    def execute_full_scan(self, origin_x, origin_y, z,
                          width, height, spacing):
        # 记录真实起飞点（工程级）
        pos = self.vehicle.drone.vehicle_local_position_enu
        home = (pos.x, pos.y, z) if pos else (0.0, 0.0, z)

        # 1）东西扫描
        self.vehicle.drone.get_logger().info("🧭 开始东西方向扫描")
        ew = self.generate_ew_waypoints(origin_x, origin_y, z,
                                        width, height, spacing)
        if not self.fly_waypoints_blocking(ew):
            return False

        # 2）平滑过渡到南北起点（关键修复）
        ns_start = (origin_x, origin_y, z)
        self.vehicle.drone.get_logger().info("🔄 过渡至南北扫描起点")
        self.vehicle.drone.fly_to_trajectory_setpoint(
            ns_start[0], ns_start[1], ns_start[2], 0.0, timeout=60.0
        )

        # 3）南北扫描（真正90°方向）
        self.vehicle.drone.get_logger().info("🧭 开始南北方向扫描")
        ns = self.generate_ns_waypoints(origin_x, origin_y, z,
                                        width, height, spacing)
        if not self.fly_waypoints_blocking(ns):
            return False

        # 4）返航（真实Home点）
        self.vehicle.drone.get_logger().info("🏠 返回起飞点")
        self.vehicle.drone.fly_to_trajectory_setpoint(
            home[0], home[1], home[2], 0.0, timeout=120.0
        )

        return True


def main():
    vehicle = Vehicle()

    try:
        # 起飞
        vehicle.drone.arm()
        if not vehicle.drone.takeoff(3.0):
            vehicle.drone.get_logger().error("❌ 起飞失败")
            return

        # 启动拍照线程（飞行时并行）
        photo = PhotoManager(vehicle, distance_interval=2.0)
        photo.start()

        # 扫描任务（主线程阻塞飞行）
        scanner = GridScanner(vehicle)
        success = scanner.execute_full_scan(
            origin_x=0.0,
            origin_y=0.0,
            z=3.0,
            width=20.0,
            height=10.0,
            spacing=2.0
        )

        # 停止拍照
        photo.stop()

        if success:
            vehicle.drone.get_logger().info("✅ 东西+南北扫描全部完成")

        vehicle.drone.simulated_land()
        vehicle.drone.disarm()

    finally:
        vehicle.close()


if __name__ == "__main__":
    main()

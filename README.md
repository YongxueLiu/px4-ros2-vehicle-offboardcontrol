
# 🛠️ offboard_control_lib 使用说明（详细版）  
# 🛠️ Detailed Usage Guide for offboard_control_lib  

---

## 📑 目录 / Table of Contents  

- [1. 概述 / Overview](#1-概述--overview)
- [2. 安装依赖 / Installation Dependencies](#2-安装依赖--installation-dependencies)
- [3. 核心类：Vehicle / Core Class: Vehicle](#3-核心类vehicle--core-class-vehicle)
- [4. 主要 API 方法（通过 vehicledrone 调用） / Key API Methods (via vehicledrone)](#4-主要-api-方法通过-vehicledrone-调用--key-api-methods-via-vehicledrone)
  - [4.1 arm() → bool](#41-arm--bool)
  - [4.2 takeoff(target_altitude: float) → bool](#42-takeofftarget_altitude-float--bool)
  - [4.3 fly_to_trajectory_setpoint(x, y, z, yaw) → None](#43-fly_to_trajectory_setpointx-y-z-yaw--none)
  - [4.4 disarm() → bool](#44-disarm--bool)
  - [4.5 close()](#45-close)
- [5. 使用示例（完整 main 文件） / Usage Example (Complete main File)](#5-使用示例完整-main-文件--usage-example-complete-main-file)
- [6. 注意事项 / Important Notes](#6-注意事项--important-notes)
  - [6.1 坐标系 / Coordinate System](#61-坐标系--coordinate-system)
  - [6.2 Offboard 模式要求 / Offboard Mode Requirements](#62-offboard-模式要求--offboard-mode-requirements)
  - [提示 / Tip](#提示--tip)

---

## 1. 概述 / Overview  

**offboard_control_lib** 是一个基于 **ROS 2（rclpy）** 开发的无人机离线控制库，封装了 PX4 飞控系统与 ROS 2 接口之间的通信逻辑。  
该库提供简洁、高层的 API，用于实现无人机的起飞、飞行控制、悬停、降落及安全退出等操作。  

**offboard_control_lib** is a **ROS 2 (rclpy)**-based library for unmanned aerial vehicle (UAV) offboard control.  
It encapsulates the communication logic between the PX4 flight stack and ROS 2 interfaces, providing a clean, high-level API for essential operations such as arming, takeoff, trajectory control, hovering, landing, and safe shutdown.

---

## 2. 安装依赖 / Installation Dependencies  

确保已安装以下组件：  
Ensure the following components are installed:

- **ROS 2**（推荐 *Humble* 或 *Iron*）  
  **ROS 2** (*Humble* or *Iron* recommended)  
- **PX4 SITL**（软件在环仿真）或真实飞控硬件  
  **PX4 SITL** (*Software-in-the-Loop simulation*) or real flight controller hardware  
- **px4_msgs**（需与 PX4 版本匹配）  
  **px4_msgs** (must match your PX4 version)  
- **rclpy**  
- **offboard_control_lib**（需正确放置于 Python 路径中或通过 `setup.py` 安装）  
  **offboard_control_lib** (must be in your Python path or installed via `setup.py`)  

---

## 3. 核心类：Vehicle / Core Class: `Vehicle`  

`Vehicle` 类继承自 `rclpy.node.Node`，自动初始化 ROS 2 节点，并内部管理以下子模块：  

The `Vehicle` class inherits from `rclpy.node.Node`, automatically initializing a ROS 2 node and managing the following internal components:

- **drone**：实际控制接口对象，提供飞行指令方法  
  **drone:** The actual control interface object that provides flight command methods  
- **订阅器**：接收飞控状态（如位置、姿态、是否就绪等）  
  **Subscribers:** Receive flight controller status (e.g., position, attitude, readiness)  
- **发布器**：发送轨迹设定点（TrajectorySetpoint）、飞控模式命令（OffboardControlMode）等  
  **Publishers:** Send trajectory setpoints (`TrajectorySetpoint`) and mode commands (`OffboardControlMode`)  
- **定时器**：周期性发送控制指令以维持 Offboard 模式  
  **Timers:** Periodically publish control commands to maintain Offboard mode  

### 初始化 / Initialization  

```python
vehicle = Vehicle()
````

此调用会：
This call will:

* 创建名为 `offboard_control_node` 的 ROS 2 节点
* 初始化所有必要的发布者/订阅者
* 启动后台定时器（通常为 20 Hz）以维持 Offboard 控制链路

> ⚠️ **注意 / Note**：`Vehicle` 实例必须在 `rclpy.init()` 之后创建（通常由 `main()` 函数隐式处理）。

---

## 4. 主要 API 方法（通过 `vehicle.drone` 调用） / Key API Methods (via `vehicle.drone`)

### 4.1 `arm() → bool`

**功能 / Function:** 解锁电机（Arming）
**返回值 / Returns:** 成功返回 `True`，否则 `False`
**前提条件 / Prerequisites:** 飞控处于就绪状态（如 GPS 锁定、IMU 校准完成等）

---

### 4.2 `takeoff(target_altitude: float) → bool`

**功能 / Function:** 执行自动起飞至指定高度（单位：米）
**参数 / Parameters:**

* `target_altitude`: 目标高度（相对于起飞点），建议 ≥ 1.5 米
  **行为 / Behavior:** 自动切换至 Offboard 模式，垂直上升至目标高度并悬停
  **返回值 / Returns:** 成功到达目标高度并稳定后返回 `True`

---

### 4.3 `fly_to_trajectory_setpoint(x: float, y: float, z: float, yaw: float) → None`

**功能 / Function:** 飞往指定的 NED 坐标系下的位置和偏航角
**参数 / Parameters:**

* `x`: 北向位置（m）
* `y`: 东向位置（m）
* `z`: 高度（m）— 输入正值表示高度（例如 `2.0` 表示 2 米）
* `yaw`: 偏航角（弧度），0 表示机头朝北

**说明 / Notes:**

* 非阻塞调用，仅发送一次设定点
* 若需持续控制，应在循环中定期调用

---

### 4.4 `disarm() → bool`

**功能 / Function:** 锁定电机（Disarming）
**返回值 / Returns:** 成功返回 `True`

---

### 4.5 `close()`

**功能 / Function:** 释放资源，停止定时器，关闭节点
**使用场景 / Usage:** 程序退出前必须调用（建议放在 `finally` 块中）

---

## 5. 使用示例（完整 main 文件） / Usage Example (Complete main File)

```python
#!/usr/bin/env python3

import rclpy
from offboard_control_lib import Vehicle

def main():
  

    # 创建 Vehicle 实例
    vehicle = Vehicle()

    try:
        # 解锁无人机
        if not vehicle.drone.arm():
            print("Arming failed!")
            return

        # 起飞至 2.0 米高度
        if vehicle.drone.takeoff(2.0):
            print("Takeoff successful. Flying to target position...")
            vehicle.drone.fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0)
        else:
            print("Takeoff failed!")

        # 安全降落并锁定电机
        vehicle.drone.disarm()

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Disarming...")
        vehicle.drone.disarm()

    finally:
        vehicle.close()
        

if __name__ == '__main__':
    main()
```

> 🔔 **注意 / Note:**
> 原始代码中缺少 `rclpy.init()` 和 `rclpy.shutdown()`，已在示例中补充。

---

## 6. 注意事项 / Important Notes

### 6.1 坐标系 / Coordinate System

本库对用户隐藏了 PX4 的 **NED（北-东-下）** 坐标系细节。
用户传入的 `z` 为正数表示高度（如 `2.0` 表示 2 米高），内部自动转换为 `-2.0` 发送给 PX4。

This library abstracts PX4’s **NED (North-East-Down)** coordinate system.
Users provide positive `z` values (e.g., `2.0 = 2 meters high`); internally converted to `-2.0`.

---

### 6.2 Offboard 模式要求 / Offboard Mode Requirements

* 必须以 ≥ 2 Hz 的频率发送控制指令，否则 PX4 会自动退出 Offboard 模式
* 本库通过内部定时器自动维持此频率

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



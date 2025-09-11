#!/usr/bin/env python3
import rclpy
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time
import matplotlib.pyplot as plt


def se3_to_vec(V_hat):
    w = np.array([V_hat[2, 1], V_hat[0, 2], V_hat[1, 0]])
    v = V_hat[0:3, 3]
    return np.concatenate((w, v))


def main(args=None):
    bot = InterbotixManipulatorXS(robot_model='vx300s')
    robot_startup()

    prev_time = time.time()
    ee_prev = None
    v_log = []
    t_log = []

    try:
        while True:
            now = time.time()
            dt = now - prev_time
            prev_time = now

            ee_pose = bot.arm.get_ee_pose()

            if ee_pose is not None:
                T = np.array(ee_pose)

                if ee_prev is not None and dt > 1e-6:
                    T_dot = (T - ee_prev) / dt
                    V_hat = T_dot @ np.linalg.inv(T)
                    twist = se3_to_vec(V_hat)
                    v_log.append(twist)
                    t_log.append(now)
                    print("twist:", twist)

                ee_prev = T

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("检测到 Ctrl+C,退出程序...")

        if len(v_log) == 0:
            print("⚠️ 没有采集到速度数据，检查 get_ee_pose() 返回值！")
            return

        v_log = np.array(v_log)
        t_log = np.array(t_log) - t_log[0]

        fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        axs[0].plot(t_log, v_log[:, 0], label="wx (rad/s)")
        axs[0].plot(t_log, v_log[:, 1], label="wy (rad/s)")
        axs[0].plot(t_log, v_log[:, 2], label="wz (rad/s)")
        axs[0].set_ylabel("Angular Vel.")
        axs[0].legend()
        axs[0].grid(True)

        axs[1].plot(t_log, v_log[:, 3], label="vx (m/s)")
        axs[1].plot(t_log, v_log[:, 4], label="vy (m/s)")
        axs[1].plot(t_log, v_log[:, 5], label="vz (m/s)")
        axs[1].set_xlabel("Time [s]")
        axs[1].set_ylabel("Linear Vel.")
        axs[1].legend()
        axs[1].grid(True)

        plt.suptitle("Ground Truth End-Effector Velocity")
        plt.show(block=True)

    finally:
        robot_shutdown()

        


if __name__ == '__main__':
    main()

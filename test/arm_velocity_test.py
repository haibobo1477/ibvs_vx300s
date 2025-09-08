#!/usr/bin/env python3
import rclpy
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time


def main():
    # 初始化机械臂
    bot = InterbotixManipulatorXS(robot_model='vx300s')
    robot_startup()

    # 切换 arm 组到 velocity 模式
    bot.core.robot_set_operating_modes('group', 'arm', 'velocity')

    # 设置一个合理的速度 (单位 rad/s)
    velocity = [0.5, 0, 0, 0, 0.2, 0]   # 只让 waist 关节转

    # 发送速度命令
    bot.core.robot_write_commands('arm', velocity)

    # 让它转 3 秒钟
    time.sleep(3)

    # 停止运动
    bot.core.robot_write_commands('arm', [-0.2, 0, 0, 0, 0, 0])

    # 关闭机器人
    robot_shutdown()


if __name__ == '__main__':
    main()

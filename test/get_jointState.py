import rclpy
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
import time

def main(args=None):
    # 初始化
    bot = InterbotixManipulatorXS(robot_model='vx300s')
    robot_startup()   # 关键：启动 ROS2 通信并连接机械臂

    try:
        while True:
            q = bot.arm.get_joint_positions()
            dq = bot.arm.get_joint_velocities()
            ee = bot.arm.get_ee_pose()
            # print(q)
            print(q)
            time.sleep(0.1)   # 每 0.1s 打印一次
    except KeyboardInterrupt:
        pass
    finally:
        robot_shutdown()

if __name__ == '__main__':
    main()

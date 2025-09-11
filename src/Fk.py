import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import time

   

def main(args=None):
        # initialize Interbotix Manipulator
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
    )
    
    
    # bot.arm.set_joint_positions([0, 0, 0, 0, 0, 0.0])
    bot.arm.set_joint_positions([0.03810342,  0.58741801,  0.36868257,  0.10310063, -0.82013755,  0.00235547])
    time.sleep(1)
    # bot.arm.set_joint_positions([ -0.147578, 0.53741834, 0.15469197, 0.1145797, -0.6364175, -0.00203412])
    # bot.arm.set_joint_positions([0.1, 0.24, -0.61, -1.22, 1.05, 1.28])
    # time.sleep(1)
    # bot.arm.set_joint_positions([1.10, 0.20, -0.55, -1.11, 1.14, 1.20])
    # time.sleep(1)
    # bot.arm.set_joint_positions([1.15, 0.23, -0.60, -1.21, 1.04, 1.27])
    

if __name__ == '__main__':
    main()
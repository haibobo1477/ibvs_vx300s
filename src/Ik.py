import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import time

   

def main(args=None):
        # initialize Interbotix Manipulator
    bot = InterbotixManipulatorXS(
        robot_model='vx300s'
    )
    
    
    
    bot.arm.set_ee_pose_components(x=0.4, y=-0.01, z=0.18)
    time.sleep(1)
    bot.arm.set_ee_pose_components(x=0.6, y=0.01, z=0.18)
    # time.sleep(1)
    # bot.arm.set_ee_pose_components(x=0.45962715, y=0.0096373, z=0.34793092)
    # time.sleep(1)
    # bot.arm.set_ee_pose_components(x=0.55962715, y=0.0096373, z=0.34793092)
    # time.sleep(1)
    # bot.arm.set_ee_pose_components(x=0.35962715, y=0.0096373, z=0.24793092)
    # time.sleep(1)
    # bot.arm.set_ee_pose_components(x=0.65962715, y=0.0096373, z=0.34793092)


if __name__ == '__main__':
    main()
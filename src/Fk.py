import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

   

def main(args=None):
        # initialize Interbotix Manipulator
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
    )
    
    
    
    bot.arm.set_joint_positions([1.1545, 0.2341, -0.6067, -1.2143, 1.0479, 1.2701])

if __name__ == '__main__':
    main()
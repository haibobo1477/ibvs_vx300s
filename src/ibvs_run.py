import ibvs
import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector


import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
import modern_robotics as mr



class ibvs_run:
    def __init__(self):
        # ====== 配置 ======
        self.TAG_SIZE_M = 0.02  # TODO: 填你的 AprilTag 边长(米)，例如 4cm 就是 0.04

        # ====== RealSense 管道 ======
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # ====== 启动摄像头并提取相机内参 ======
        self.pipeline.start(config)
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()
        
        self.fx, self.fy = intr.fx, intr.fy
        self.cx, self.cy = intr.ppx, intr.ppy
        
        print(f"[INFO] Color Intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")

        # AprilTag 检测器启动
        self.at_detector = Detector(
            families="tagStandard41h12",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
           )

        # ====== 机械臂 ======
        robot_startup()
        self.bot = InterbotixManipulatorXS(robot_model="vx300s")
        self.bot.core.robot_set_operating_modes("group", "arm", "velocity")
        
        self.joint_names = self.bot.arm.group_info.joint_names
        self.idx_map = self.bot.core.js_index_map
    
    
    def read_frames(self):
        frames = self.pipeline.wait_for_frames()
        cf = frames.get_color_frame()
        df = frames.get_depth_frame()
        if not cf or not df:
            return None, None, None
        color = np.asanyarray(cf.get_data())
        depth = np.asanyarray(df.get_data())
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        
        return color, depth, frames, gray


    def get_joint_positions(self):
        js = self.bot.core.joint_states
        q = np.array([js.position[self.idx_map[n]] for n in self.joint_names], dtype=np.float64)
        return q



    def detect_tag(self, gray, color_image):
        
        detections = self.at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.TAG_SIZE_M
        )
        
        # only one tag should be dectected
        for det in detections:
            # 2D 可视化
            pts = det.corners.astype(int)
            cv2.polylines(color_image, [pts], True, (0, 255, 0), 2)
            cX, cY = map(int, det.center)
            cv2.circle(color_image, (cX, cY), 3, (0, 0, 255), -1)
            cv2.putText(color_image, f"id:{det.tag_id}", (pts[0][0], pts[0][1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 三维位姿（相机坐标系）：R(3x3), t(3x1, 米)
            R = det.pose_R.astype(np.float64)
            t = det.pose_t.reshape(3, 1).astype(np.float64)
             
        return pts, t
    
    
    
    def control_law(self, error_corners, x, y, Z, cTb, bTe, Blist):
        """

        everytime get the error and calculate the joint velocity one time
        return the joint velocity for each time deteced the
            
        """
        q = self.bot.arm.get_joint_positions()
        
        
        L = ibvs.L_point(x, y, Z)
        cVb = ibvs.base_to_camera(cTb)
        bVe = ibvs.end_to_base(bTe)
        eJe = mr.JacobianBody(Blist, q)
        
        
        dq = ( L @ cVb @ bVe @ eJe) @ error_corners
        
        return dq
    
    

























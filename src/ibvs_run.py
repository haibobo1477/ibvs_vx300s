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
        self._lambda = 0.0003

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
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.8,
            debug=0
           )

        # ====== 机械臂 ======
        # robot_startup()
        self.bot = InterbotixManipulatorXS(robot_model="vx300s")
        # self.bot.core.robot_set_operating_modes("group", "arm", "velocity")
        
        # self.joint_names = self.bot.arm.group_info.joint_names
        # self.idx_map = self.bot.core.js_index_map
    
    def L_point(self, x, y, Z):
        """
           输入:
            x, y, Z 数字
           输出:
        L : (2 x 6) 交互矩阵
        """
        L = np.array([
            [-1/Z,      0, x/Z,  x*y, -(1+x*x),  y],
            [0,     -1/Z, y/Z,  1+y*y,   -x*y, -x]
        ])

        return L

    
    def set_camera(self):
        frames = self.pipeline.wait_for_frames()
        cf = frames.get_color_frame()
        if not cf:
            return None, None, None
        color = np.asanyarray(cf.get_data())

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        
        return color, gray


    def detect_tag(self, gray, color_image):
        
        detections = self.at_detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=[self.fx, self.fy, self.cx, self.cy],
        tag_size=self.TAG_SIZE_M
    )
    
        if len(detections) == 0:
            return None, None, None, None

    # 只取第一个检测到的 tag
        det = detections[0]

    # 2D 可视化
        pts = det.corners.astype(int)
        cv2.polylines(color_image, [pts], True, (0, 255, 0), 2)
        cX, cY = map(int, det.center)
        cv2.circle(color_image, (cX, cY), 3, (0, 0, 255), -1)
        cv2.putText(color_image, f"id:{det.tag_id}", (pts[0][0], pts[0][1]-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 三维位姿（相机坐标系）：t(3x1, 米)
        R = det.pose_R.astype(np.float64)
        t = det.pose_t.reshape(3, 1).astype(np.float64)
        # Z = np.linalg.norm(t)
        Z_center = t[2, 0]

        # 中心三维位姿
        
    # tag 半边长
        # s = self.TAG_SIZE_M / 2.0

    # Tag 坐标系下的四个角点 (顺序和 detector 的 pts 对齐)
        # tag_corners_3d = np.array([
        #       [ s, -s, 0],   # 右下
        #       [ s,  s, 0],   # 右上
        #       [-s,  s, 0],   # 左上
        #       [-s, -s, 0]    # 左下
        # ], dtype=np.float64).T  # shape (3,4)

    # 转到相机坐标系
        # corners_cam = R @ tag_corners_3d + t  # shape (3,4)

    # 取每个角点的 Z
        # Z_corners = corners_cam[2, :]   # shape (4,)


        # x = (pts[:,0] - self.cx) / self.fx
        # y = (pts[:,1] - self.cy) / self.fy

    
        return cX, cY, Z_center, t


    # def draw_reference(self, color_image, square_half=60):
    #     h, w, _ = color_image.shape
    #     cx_img, cy_img = w // 2, h // 2   # 图像中心
    
    # # 修改顺序：右下 -> 右上 -> 左上 -> 左下
    #     square_pts = [
    #         (cx_img + square_half, cy_img + square_half),  # 右下
    #         (cx_img + square_half, cy_img - square_half),  # 右上
    #         (cx_img - square_half, cy_img - square_half),  # 左上
    #         (cx_img - square_half, cy_img + square_half)   # 左下
    # ]
        

    # # 画角点
    #     for pt in square_pts:
    #         cv2.circle(color_image, pt, 3, (0, 0, 0), -1)  # 黑色角点

    #     return np.array(square_pts, dtype=np.float32)

    def draw_reference(self, color_image):
        # 获取图像宽高
        h, w, _ = color_image.shape
        cx_img, cy_img = w // 2, h // 2   # 图像中心

    # 在图像中心画一个红点
        cv2.circle(color_image, (cx_img, cy_img), 5, (0, 0, 255), -1)

    # 可选：加文字标记
        cv2.putText(color_image, "Center", (cx_img + 10, cy_img),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # 返回中心点像素坐标
        return np.array([cx_img, cy_img], dtype=np.float32)
    

    def get_jointstate(self):
        
        q = self.bot.arm.get_joint_positions()
        ee = self.bot.arm.get_ee_pose()
        
        return q, ee
    

    # def control_law(self, q, error_u, error_v, error_Z, bRc, Slist):
    #     """
    #      IBVS
    #     """

    #     dp = np.array([error_u, error_v, error_Z, 0.0, 0.0, 0.0]).T

    #     Rv = np.vstack((np.hstack((bRc, np.zeros((3, 3)))),
    #                     np.hstack((np.zeros((3, 3)), bRc))))
        
    #     v = Rv @ dp

    #     Js = mr.JacobianSpace(Slist, q)

    #     Js_inv = np.linalg.pinv(Js)

    #     dq = Js_inv @ v
            
    #     return dq
    

    def control_law(self, q, error_corners, x, y, Z, cTb, Slist, lam):
        """
         IBVS:
            dq = λ * (Ls * cVb * Js)^+ * (s - s*)
        """
            
        L = ibvs.L_point(x, y, Z)
        cVb = ibvs.base_to_camera(cTb)
        # bVe = ibvs.end_to_base(bTe)
        # eJe = mr.JacobianBody(Blist, q)

        Js = mr.JacobianSpace(Slist, q)

        J_img = L @ cVb @ Js

        J_pinv = np.linalg.pinv(J_img)
        

        dq = - lam * J_pinv @ error_corners
        
        return dq
    




    def move_robotic(self, q):
        self.bot.arm.set_joint_positions(q)


    # def move_robotic(self, x, y, z):
    #     self.bot.arm.set_ee_pose_components(x, y, z)
    

    # def move_robotic(self, dq, max_vel=0.2):
            
    #         """
    #            控制机械臂关节速度
    #            dq: numpy.ndarray, shape (n,1) or (n,), 关节速度 (rad/s)
    #            max_vel: 每个关节最大速度 (rad/s)，默认 0.5
    #         """
    #         dq = np.array(dq).flatten()   # 确保是一维向量

    #            # 安全检查：如果检测不到目标，直接停
    #         if dq is None or dq.shape[0] != len(self.bot.arm.group_info.joint_names):
    #             print("[WARN] dq 无效，停止机械臂")
    #             self.bot.core.robot_write_commands('arm', [0, 0, 0, 0, 0, 0])
    #             return

    #           # 限幅：避免速度过大
    #         dq_clipped = np.clip(dq, -max_vel, max_vel)

    #           # 写入命令
    #         self.bot.core.robot_write_commands('arm', dq_clipped.tolist())




























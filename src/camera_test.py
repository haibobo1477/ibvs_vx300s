import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector

# ====== 配置 ======
TAG_SIZE_M = 0.02  # TODO: 填你的 AprilTag 边长(米)，例如 4cm 就是 0.04

# ====== RealSense 管道 ======
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# AprilTag 检测器
at_detector = Detector(
    families="tagStandard41h12",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# 画三轴的小函数（将相机坐标系下的一段轴投影到图像）
def draw_axes(img, K, R, t, axis_len=0.02):  # 轴长(米)
    # 相机内参矩阵
    fx, fy, cx, cy = K
    Kmat = np.array([[fx, 0,  cx],
                     [0,  fy, cy],
                     [0,  0,  1]], dtype=np.float64)

    # 3D 轴端点（相机坐标系下的 tag 坐标，通过 R,t 把 tag 坐标变到相机坐标系）
    # 但更方便的方法：构造 tag 坐标系下的点，然后用 [R|t] 映射到相机坐标，再投影。
    # 定义 tag 坐标系原点为 tag 中心，z 轴垂出纸面朝向相机。
    # 这里先定义轴在 tag 坐标系下的点：
    origin_tag = np.zeros((3, 1))  # (0,0,0)
    X_tag = np.array([[axis_len, 0, 0]]).T
    Y_tag = np.array([[0, axis_len, 0]]).T
    Z_tag = np.array([[0, 0, axis_len]]).T

    # 把 tag 坐标系点变换到相机坐标系： Pc = R * P_tag + t
    def proj(P_tag):
        Pc = R @ P_tag + t
        x, y, z = Pc.flatten()
        u = fx * (x / z) + cx
        v = fy * (y / z) + cy
        return int(round(u)), int(round(v))

    o = proj(origin_tag)
    px = proj(X_tag)
    py = proj(Y_tag)
    pz = proj(Z_tag)

    # 画线（颜色顺序：X-?, Y-?, Z-?；不强制颜色，默认BGR）
    cv2.line(img, o, px, (0, 0, 255), 2)   # X 轴 红
    cv2.line(img, o, py, (0, 255, 0), 2)   # Y 轴 绿
    cv2.line(img, o, pz, (255, 0, 0), 2)   # Z 轴 蓝

# ====== 启动摄像头并提取相机内参 ======
pipeline.start(config)
profile = pipeline.get_active_profile()
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
intr = color_profile.get_intrinsics()
fx, fy = intr.fx, intr.fy
cx, cy = intr.ppx, intr.ppy
print(f"[INFO] Color Intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # BGR 彩色图
        color_image = np.asanyarray(color_frame.get_data())

        # 转灰度图（单通道）
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # ---- AprilTag 检测 + 三维位姿估计 ----
        detections = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy],
            tag_size=TAG_SIZE_M
        )

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

            # 打印姿态
            print(f"[Tag {det.tag_id}] t (m) = {t.flatten()}")
            print(f"[Tag {det.tag_id}] R =\n{R}")

            # 在图像上画相机坐标系下的三轴
            draw_axes(color_image, (fx, fy, cx, cy), R, t, axis_len=TAG_SIZE_M * 0.75)

        cv2.imshow("RealSense D345i - AprilTag Pose", color_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

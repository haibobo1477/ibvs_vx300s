import cv2
from ibvs_run import ibvs_run   # 假设你的类文件名是 ibvs.py
import numpy as np
import time

def main():
    ibvs = ibvs_run()

    while True:
        # 获取相机帧
        color, gray = ibvs.set_camera()
        if gray is None:
            continue

        # 画中心正方形参考点
        square_pts = ibvs.draw_reference(color)

        # AprilTag 检测
        pts, Z = ibvs.detect_tag(gray, color)
        time.sleep(0.01)
        if pts is not None:

            square_pts = np.array(square_pts, dtype=np.float32)
            pts = np.array(pts, dtype=np.float32)
            
            error = pts - square_pts
            print(square_pts.flatten())
            # print(pts.flatten())
            print(f"检测到 Tag, 距离 Z = {Z:.3f} m, 误差为 e = {error.flatten()}" )
            
            
            q, ee = ibvs.get_jointstate()
            
            dq = ibvs.control_law(q, error, x, y, Z, cTb, bTe, Blist)

        else:
            print("未检测到 Tag")

        # 显示结果
        cv2.imshow("AprilTag + Reference", color)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()






# import cv2
# from ibvs_run import ibvs_run

# def main():
#     ibvs = ibvs_run()

#     while True:
#         # 获取相机帧
#         color, gray = ibvs.set_camera()
#         if gray is None:
#             # 没有拿到新帧，直接跳过这轮
#             continue

#         # AprilTag 检测
#         pts, Z = ibvs.detect_tag(gray, color)

#         if pts is not None:
#             print(f"检测到 Tag, 距离 Z = {Z:.3f} m")
#         else:
#             print("未检测到 Tag")

#         # 显示结果（无论有没有检测到，都显示相机画面）
#         cv2.imshow("AprilTag Detection", color)
#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break

#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()
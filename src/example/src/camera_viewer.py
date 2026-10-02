#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
NODE THU NHẬN & HIỂN THỊ CAMERA FPV XE BFMC (ENHANCED CAMERA VIEWER)
  • Lắng nghe topic: /automobile/image_raw (sensor_msgs/Image)
  • Chuyển đổi sang ảnh OpenCV (BGR8) qua cv_bridge
  • Hiển thị FPS thời gian thực và các vùng quan sát (ROI)
  • Phím tắt 's' để chụp ảnh lưu lại, 'q' hoặc 'Esc' để thoát
=============================================================================
"""

import os
import sys
import time
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraViewerNode:
    def __init__(self):
        rospy.init_node('bfmc_camera_viewer', anonymous=False)
        self.bridge = CvBridge()

        self.save_dir = "/root/bfmc_simulator_ws/camera_snapshots"
        os.makedirs(self.save_dir, exist_ok=True)
        self.snapshot_idx = 1

        # Đo FPS
        self.prev_time = time.time()
        self.fps = 0.0

        # Subscribe topic camera FPV của xe
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback, queue_size=1)

        print("\n" + "="*65)
        print("   📷 BFMC CAMERA FPV VIEWER & DATA RECORDER")
        print("="*65)
        print("Đang lắng nghe luồng ảnh từ topic: /automobile/image_raw ...")
        print("Phím tắt trên cửa sổ Camera:")
        print("  • Phím 's' : Chụp và lưu ảnh hiện tại (Snapshot)")
        print("  • Phím 'q' / 'Esc': Thoát cửa sổ xem camera")
        print("="*65 + "\n")

    def image_callback(self, msg):
        try:
            # 1. Chuyển đổi ROS Image sang OpenCV NumPy BGR8
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Lỗi chuyển đổi CvBridge: {e}")
            return

        # Tính toán FPS
        now = time.time()
        dt = now - self.prev_time
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        self.prev_time = now

        # Tạo bản sao để vẽ Overlay hiển thị thông tin
        display = frame.copy()
        h, w, _ = display.shape

        # 2. Vẽ gợi ý các vùng phân tích (Regions of Interest - ROI)
        # ROI trên: Nhận diện Đèn giao thông / Biển báo (0% -> 60% chiều cao)
        cv2.rectangle(display, (0, 0), (w, int(h * 0.55)), (0, 255, 255), 1)
        cv2.putText(display, "ROI: DEN GIAO THONG & BIEN BAO", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # ROI dưới: Nhận diện Làn đường (Lane Detection - 55% -> 100% chiều cao)
        cv2.rectangle(display, (0, int(h * 0.55)), (w, h), (0, 255, 0), 1)
        cv2.putText(display, "ROI: BAM LAN DUONG", (10, int(h * 0.55) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # 3. Vẽ vạch tâm đường chuẩn
        cv2.line(display, (w // 2, int(h * 0.6)), (w // 2, h), (255, 0, 0), 1)

        # 4. Hiển thị thông số FPS và kích thước
        info_text = f"Res: {w}x{h} | FPS: {self.fps:.1f} | Nhan 's': Chup anh"
        cv2.putText(display, info_text, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # 5. Hiển thị lên cửa sổ OpenCV
        cv2.imshow("BFMC - FPV Camera Feed", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            filename = os.path.join(self.save_dir, f"snapshot_{self.snapshot_idx:04d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"[ĐÃ LƯU] Ảnh chụp được lưu tại: {filename}")
            self.snapshot_idx += 1
        elif key == ord('q') or key == 27:
            rospy.signal_shutdown("User exit")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = CameraViewerNode()
        viewer.run()
    except rospy.ROSInterruptException:
        pass

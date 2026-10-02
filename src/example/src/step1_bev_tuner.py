#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
BƯỚC 1: TIỀN XỬ LÝ ẢNH & CĂN CHỈNH GÓC NHÌN CHIM BAY (BIRD'S EYE VIEW - BEV TUNER)
Mục tiêu:
  1. Lấy dữ liệu ảnh FPV trực tiếp từ camera xe (/automobile/image_raw).
  2. Dùng thanh trượt Trackbar để trực quan hóa và căn chỉnh:
     • 4 điểm hình thang phối cảnh trên mặt đường (Source ROI).
     • Ngưỡng nhị phân vạch sơn trắng (Binary Threshold).
  3. Biến đổi ma trận phối cảnh (Homography 3x3) sang góc nhìn thẳng đứng từ trên cao.
  4. Hiển thị song song: Màn hình FPV thực tế (trái) và Mặt phẳng BEV (phải).
=============================================================================
"""

import sys
import json
import time
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class BEVTuner:
    def __init__(self):
        rospy.init_node('bfmc_step1_bev_tuner', anonymous=False)
        self.bridge = CvBridge()

        self.img_w = 640
        self.img_h = 480

        # Trạng thái hiển thị
        self.current_frame = None
        self.view_binary = True       # True = Xem ảnh nhị phân đen trắng | False = Xem màu gốc
        self.fps = 0.0
        self.last_time = time.time()

        # Tên cửa sổ OpenCV
        self.win_name = "BFMC - PHAN 1: BIRD'S EYE VIEW TUNER"
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)

        # -------------------------------------------------------------
        # KHỞI TẠO CÁC THANH TRƯỢT TƯƠNG TÁC (TRACKBARS)
        # Các giá trị mặc định đã được hiệu chuẩn tối ưu cho Gazebo BFMC
        # -------------------------------------------------------------
        # 1. Ngưỡng sáng nhị phân (Vạch sơn Gazebo có giá trị xám ~178)
        cv2.createTrackbar("Threshold", self.win_name, 135, 255, self.on_trackbar)
        
        # 2. Tọa độ Y đáy hình thang (gần mũi xe) và Y đỉnh (xa sát chân trời)
        cv2.createTrackbar("Bottom Y", self.win_name, 245, 480, self.on_trackbar)
        cv2.createTrackbar("Top Y", self.win_name, 140, 300, self.on_trackbar)
        
        # 3. Độ dịch mép X đáy (Bottom Inset) và mép X đỉnh (Top Inset)
        cv2.createTrackbar("Bottom Inset X", self.win_name, 40, 200, self.on_trackbar)
        cv2.createTrackbar("Top Inset X", self.win_name, 190, 300, self.on_trackbar)

        # 4. Độ lề trái/phải trên ảnh BEV đích
        cv2.createTrackbar("BEV Margin X", self.win_name, 120, 250, self.on_trackbar)

        # Đăng ký nhận hình ảnh từ Camera Gazebo
        self.image_sub = rospy.Subscriber('/automobile/image_raw', Image, self.image_callback, queue_size=1)

        print("\n" + "="*75)
        print("   🔍  BFMC STEP 1: BIRD'S EYE VIEW (BEV) INTERACTIVE TUNER")
        print("="*75)
        print("  • HƯỚNG DẪN DÙNG CHUỘT KÉO THANH TRƯỢT:")
        print("     - Threshold      : Tăng/giảm để lọc sạch nền nhựa đường, chỉ giữ vạch trắng.")
        print("     - Bottom Y/Top Y : Giới hạn khoảng cách gần/xa cần quan sát trên mặt đường.")
        print("     - Inset X        : Khép mở độ nghiêng 2 cạnh hình thang theo vạch kẻ.")
        print("     - BEV Margin X   : Độ dạt 2 bên lề trên ảnh chim bay (BEV).")
        print("\n  • PHÍM TẮT TRÊN MÀN HÌNH:")
        print("     - Phím 't': Chuyển đổi giữa chế độ Nhị phân (Binary) và Màu gốc (Color).")
        print("     - Phím 's': Lưu và in tọa độ ma trận hiện tại ra Terminal.")
        print("     - Phím 'r': Reset về thông số chuẩn ban đầu.")
        print("     - Phím 'q': Thoát chương trình.")
        print("="*75 + "\n")

    def on_trackbar(self, val):
        pass

    def get_trackbar_values(self):
        """Lấy giá trị hiện tại từ các thanh trượt"""
        thresh = cv2.getTrackbarPos("Threshold", self.win_name)
        bot_y = cv2.getTrackbarPos("Bottom Y", self.win_name)
        top_y = cv2.getTrackbarPos("Top Y", self.win_name)
        bot_inset = cv2.getTrackbarPos("Bottom Inset X", self.win_name)
        top_inset = cv2.getTrackbarPos("Top Inset X", self.win_name)
        bev_margin = cv2.getTrackbarPos("BEV Margin X", self.win_name)

        # Ràng buộc an toàn
        bot_y = max(bot_y, top_y + 20)

        # 4 điểm góc hình thang nguồn (Source Points trên ảnh Camera)
        src_points = np.float32([
            [bot_inset, bot_y],                      # Đáy trái
            [self.img_w - bot_inset, bot_y],         # Đáy phải
            [top_inset, top_y],                      # Đỉnh trái
            [self.img_w - top_inset, top_y]          # Đỉnh phải
        ])

        # 4 điểm chữ nhật đích (Destination Points trên ảnh Chim bay BEV)
        dst_points = np.float32([
            [bev_margin, self.img_h],                # Đáy trái BEV
            [self.img_w - bev_margin, self.img_h],   # Đáy phải BEV
            [bev_margin, 0],                         # Đỉnh trái BEV
            [self.img_w - bev_margin, 0]             # Đỉnh phải BEV
        ])

        return thresh, src_points, dst_points

    def image_callback(self, msg):
        now = time.time()
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(now - self.last_time, 0.001))
        self.last_time = now

        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        self.process_and_display()

    def process_and_display(self):
        if self.current_frame is None:
            return

        frame = self.current_frame.copy()
        thresh, src_points, dst_points = self.get_trackbar_values()

        # 1. Tính ma trận chuyển đổi phối cảnh 3x3 M (Perspective Transform Matrix)
        M = cv2.getPerspectiveTransform(src_points, dst_points)

        # 2. Xử lý ảnh nhị phân bắt vạch sơn trắng
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
        binary_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        # 3. Biến đổi ảnh sang Bird's Eye View
        if self.view_binary:
            # Chiếu ảnh nhị phân
            bev_img = cv2.warpPerspective(binary_bgr, M, (self.img_w, self.img_h), flags=cv2.INTER_LINEAR)
        else:
            # Chiếu ảnh màu gốc
            bev_img = cv2.warpPerspective(frame, M, (self.img_w, self.img_h), flags=cv2.INTER_LINEAR)

        # 4. Vẽ khung hình thang ROI lên ảnh FPV bên trái
        fpv_display = frame.copy()
        pts = src_points.reshape((-1, 1, 2)).astype(np.int32)
        cv2.polylines(fpv_display, [pts], isClosed=True, color=(0, 255, 255), thickness=2)
        
        # Vẽ các chấm tròn đánh dấu 4 góc hình thang: Đáy (Đỏ), Đỉnh (Xanh dương)
        cv2.circle(fpv_display, (int(src_points[0][0]), int(src_points[0][1])), 6, (0, 0, 255), -1)
        cv2.circle(fpv_display, (int(src_points[1][0]), int(src_points[1][1])), 6, (0, 0, 255), -1)
        cv2.circle(fpv_display, (int(src_points[2][0]), int(src_points[2][1])), 6, (255, 100, 0), -1)
        cv2.circle(fpv_display, (int(src_points[3][0]), int(src_points[3][1])), 6, (255, 100, 0), -1)

        # 5. Vẽ lưới hỗ trợ đo độ thẳng trên ảnh BEV bên phải (Grid Overlay)
        bev_display = bev_img.copy()
        # Đường tim tâm xe (Màu vàng đứt khúc)
        cv2.line(bev_display, (self.img_w // 2, 0), (self.img_w // 2, self.img_h), (0, 255, 255), 1)
        # Các vạch ngang tham chiếu cự ly (cách nhau 80px)
        for y_ref in range(80, self.img_h, 80):
            cv2.line(bev_display, (0, y_ref), (self.img_w, y_ref), (80, 80, 80), 1)
            cv2.putText(bev_display, f"{y_ref}px", (10, y_ref - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        # 6. Ghép 2 ảnh cạnh nhau
        f_small = cv2.resize(fpv_display, (self.img_w // 2, self.img_h // 2))
        b_small = cv2.resize(bev_display, (self.img_w // 2, self.img_h // 2))
        combined = np.hstack((f_small, b_small))

        # 7. Vẽ thanh trạng thái Dashboard
        cv2.rectangle(combined, (0, 0), (combined.shape[1], 46), (25, 25, 25), -1)
        mode_str = "BINARY (Trang-Den)" if self.view_binary else "COLOR (Mau Goc)"
        txt1 = f"FPS: {self.fps:4.1f} | Che do: {mode_str} (Nhan 't' de doi) | Phim 's': In ma tran"
        txt2 = f"ROI: Y=[{int(src_points[2][1])}, {int(src_points[0][1])}] | X_bot=[{int(src_points[0][0])}, {int(src_points[1][0])}] | Thresh={thresh}"
        cv2.putText(combined, txt1, (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(combined, txt2, (10, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow(self.win_name, combined)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            rospy.signal_shutdown("Thoát chương trình")
            sys.exit(0)
        elif key == ord('t'):
            self.view_binary = not self.view_binary
            print(f"[MODE] Đã chuyển chế độ xem: {'NHỊ PHÂN (BINARY)' if self.view_binary else 'MÀU GỐC (COLOR)'}")
        elif key == ord('s'):
            self.save_and_print_calibration(src_points, dst_points, thresh)
        elif key == ord('r'):
            self.reset_calibration()

    def save_and_print_calibration(self, src, dst, thresh):
        print("\n" + "="*70)
        print("💾 TỌA ĐỘ HIỆU CHUẨN ĐƯỢC CHỌN (Copy dùng cho Phần 2):")
        print("="*70)
        print(f"THRESHOLD = {thresh}")
        print("SRC_POINTS = np.float32([")
        for pt in src:
            print(f"    [{pt[0]:.1f}, {pt[1]:.1f}],")
        print("])")
        print("DST_POINTS = np.float32([")
        for pt in dst:
            print(f"    [{pt[0]:.1f}, {pt[1]:.1f}],")
        print("])")
        print("="*70 + "\n")

    def reset_calibration(self):
        cv2.setTrackbarPos("Threshold", self.win_name, 135)
        cv2.setTrackbarPos("Bottom Y", self.win_name, 245)
        cv2.setTrackbarPos("Top Y", self.win_name, 140)
        cv2.setTrackbarPos("Bottom Inset X", self.win_name, 40)
        cv2.setTrackbarPos("Top Inset X", self.win_name, 190)
        cv2.setTrackbarPos("BEV Margin X", self.win_name, 120)
        print("[RESET] Đã khôi phục các thông số BEV mặc định tối ưu.")


if __name__ == '__main__':
    try:
        tuner = BEVTuner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

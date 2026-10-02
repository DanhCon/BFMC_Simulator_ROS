#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
BƯỚC 3: BỘ ĐIỀU KHIỂN BÁM LÀN TỰ HÀNH & BỘ TINH CHỈNH BIRD'S EYE VIEW TRỰC TIẾP
(INTEGRATED LIVE BEV CALIBRATION & PURE PURSUIT LANE FOLLOWER)

Tính năng tích hợp 2 trong 1:
  1. VỪA CHẠY XE VỪA CĂNG CHỈNH (LIVE TUNING WHILE DRIVING):
     • Tích hợp trực tiếp 8 thanh trượt Trackbars ngay trên cửa sổ OpenCV.
     • Kéo thanh trượt thay đổi ma trận phối cảnh Bird's Eye View tức thì trong O(1).
     • Nhấn phím 's' để in và tự động lưu bộ thông số đã căn chỉnh vào file cấu hình.
  2. BỘ ĐIỀU KHIỂN BÁM LÀN PURE PURSUIT THỰC CHIẾN:
     • Ghép cặp đỉnh Histogram thông minh theo khẩu độ vật lý làn đường.
     • Cửa sổ trượt dự đoán tiếp tuyến (Tangent tracking) + Curvature Guard.
     • Bộ lọc thông thấp khử rung lắc vô lăng (Low-Pass Filter).
     • Điều tốc thích ứng góc cua: Tự động hãm ga khi ôm cua gắt chống trượt văng.
  3. BẢNG PHÍM NÓNG ĐIỀU KHIỂN NHANH:
     • 'p' : Tạm dừng (phanh xe) / Tiếp tục tự hành.
     • 's' : LƯU & XUẤT THÔNG SỐ HIỆU CHUẨN RA TERMINAL / FILE JSON.
     • 't' : Chuyển đổi hiển thị BEV giữa Ảnh Nhị Phân và Ảnh Màu Gốc.
     • 'r' : Khôi phục toàn bộ thông số về mặc định (Reset).
     • '+' / '-' : Tăng / Giảm tốc độ cơ sở (đồng bộ lên thanh trượt).
     • '[' / ']' : Đưa điểm nhìn Lookahead lại gần / ra xa (đồng bộ lên thanh trượt).
     • 'q' : Dừng phanh khẩn cấp và thoát chương trình.
=============================================================================
"""

import sys
import os
import time
import json
import collections
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class PurePursuitLaneFollowerWithTuner:
    def __init__(self):
        rospy.init_node('bfmc_step3_lane_follower', anonymous=False)
        self.bridge = CvBridge()

        # Publishers & Subscribers (queue_size=10 để chống drop packet trong ROS socket)
        self.cmd_pub = rospy.Publisher('/automobile/command', String, queue_size=10)
        self.image_sub = rospy.Subscriber('/automobile/image_raw', Image, self.image_callback, queue_size=1)

        self.img_w = 640
        self.img_h = 480
        self.lane_width_bev = 210.0      # Độ rộng làn trên BEV (~37 cm)
        self.half_lane_bev = 105.0

        # =====================================================================
        # 📌 BỘ THÔNG SỐ CƠ SỞ (BASELINE CALIBRATION PARAMETERS)
        # =====================================================================
        self.default_threshold = 126
        self.default_bot_y = 429
        self.default_top_y = 170
        self.default_bot_inset = 0
        self.default_top_inset = 207
        self.default_bev_margin = 138
        self.default_lookahead_y = 260
        self.default_speed = 30          # cm/s (tương đương 0.30 m/s)

        self.threshold = self.default_threshold
        self.lookahead_y = self.default_lookahead_y
        self.base_speed = float(self.default_speed) / 100.0
        self.curve_speed = 0.16          # Tốc độ khi ôm cua gắt (m/s)
        self.max_steer = 23.0            # Giới hạn góc lái cơ học (độ)
        self.pursuit_gain = 1.15         # Hệ số khuếch đại Pure Pursuit phản xạ nhạy

        # Cấu hình Cửa sổ trượt
        self.nwindows = 9
        self.margin = 95                 # Biên mở rộng bắt khúc cua gắt
        self.minpix = 15

        # Bộ lọc làm mượt tay lái & bộ nhớ làn
        self.prev_steer = 0.0
        self.prev_target_x = 320.0
        self.is_active = True            # Trạng thái chạy xe
        self.view_binary = False         # False: Xem màu gốc trên BEV, True: Xem nhị phân

        self.fps = 0.0
        self.last_time = time.time()
        self.win_name = "BFMC - PHAN 3: PURE PURSUIT & BEV TUNER"

        # Khởi tạo Cửa sổ OpenCV & Các Thanh Trượt (Trackbars)
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)

        cv2.createTrackbar("Threshold", self.win_name, self.default_threshold, 255, self.on_trackbar)
        cv2.createTrackbar("Bottom Y", self.win_name, self.default_bot_y, 480, self.on_trackbar)
        cv2.createTrackbar("Top Y", self.win_name, self.default_top_y, 350, self.on_trackbar)
        cv2.createTrackbar("Bottom Inset X", self.win_name, self.default_bot_inset, 250, self.on_trackbar)
        cv2.createTrackbar("Top Inset X", self.win_name, self.default_top_inset, 300, self.on_trackbar)
        cv2.createTrackbar("BEV Margin X", self.win_name, self.default_bev_margin, 250, self.on_trackbar)
        cv2.createTrackbar("Lookahead Y", self.win_name, self.default_lookahead_y, 380, self.on_trackbar)
        cv2.createTrackbar("Speed (cm/s)", self.win_name, self.default_speed, 60, self.on_trackbar)

        # Cập nhật ma trận phối cảnh ban đầu
        self.update_calibration_from_trackbars()

        print("\n" + "="*78)
        print("   🏎️  BFMC PHẦN 3: PURE PURSUIT LANE FOLLOWER TÍCH HỢP BEV LIVE TUNER")
        print("="*78)
        print("  • BỘ TINH CHỈNH BIRD'S EYE VIEW TRỰC TIẾP:")
        print("     - Kéo các thanh trượt trên cửa sổ OpenCV để căng chỉnh hình học phối cảnh.")
        print("     - Phím 's' : LƯU & XUẤT mã nguồn thông số hiệu chuẩn đã chọn ra Terminal.")
        print("     - Phím 't' : Chuyển đổi hiển thị BEV giữa Ảnh Nhị Phân và Ảnh Màu Gốc.")
        print("     - Phím 'r' : Reset các thanh trượt về giá trị gốc ban đầu.")
        print("\n  • ĐIỀU KHIỂN XE LĂN BÁNH:")
        print("     - Phím 'p' : Tạm dừng (Pause phanh xe) / Tiếp tục chạy (Resume).")
        print("     - Phím '+' / '-': Tăng / Giảm tốc độ chạy thẳng (đồng bộ thanh trượt).")
        print("     - Phím '[' / ']': Đưa điểm nhìn Lookahead lại GẦN / ra XA.")
        print("     - Phím 'q' : Dừng phanh khẩn cấp và thoát chương trình.")
        print("="*78 + "\n")

    def on_trackbar(self, val):
        pass

    def update_calibration_from_trackbars(self):
        """Đọc tức thời giá trị từ 8 thanh trượt và tái cấu trúc ma trận M"""
        self.threshold = cv2.getTrackbarPos("Threshold", self.win_name)
        bot_y = cv2.getTrackbarPos("Bottom Y", self.win_name)
        top_y = cv2.getTrackbarPos("Top Y", self.win_name)
        bot_inset = cv2.getTrackbarPos("Bottom Inset X", self.win_name)
        top_inset = cv2.getTrackbarPos("Top Inset X", self.win_name)
        bev_margin = cv2.getTrackbarPos("BEV Margin X", self.win_name)

        lk_y = cv2.getTrackbarPos("Lookahead Y", self.win_name)
        if lk_y > 100:
            self.lookahead_y = lk_y

        spd = cv2.getTrackbarPos("Speed (cm/s)", self.win_name)
        if spd >= 10:
            self.base_speed = float(spd) / 100.0

        # Ràng buộc an toàn tránh hình thang bị lộn ngược
        bot_y = max(bot_y, top_y + 20)

        # 4 điểm góc hình thang nguồn trên ảnh FPV
        self.src_points = np.float32([
            [bot_inset, bot_y],
            [self.img_w - bot_inset, bot_y],
            [top_inset, top_y],
            [self.img_w - top_inset, top_y]
        ])

        # 4 điểm chữ nhật đích trên ảnh BEV
        self.dst_points = np.float32([
            [bev_margin, self.img_h],
            [self.img_w - bev_margin, self.img_h],
            [bev_margin, 0],
            [self.img_w - bev_margin, 0]
        ])

        self.M_warp = cv2.getPerspectiveTransform(self.src_points, self.dst_points)

    def reset_calibration(self):
        """Khôi phục lại toàn bộ thanh trượt về giá trị cơ sở"""
        cv2.setTrackbarPos("Threshold", self.win_name, self.default_threshold)
        cv2.setTrackbarPos("Bottom Y", self.win_name, self.default_bot_y)
        cv2.setTrackbarPos("Top Y", self.win_name, self.default_top_y)
        cv2.setTrackbarPos("Bottom Inset X", self.win_name, self.default_bot_inset)
        cv2.setTrackbarPos("Top Inset X", self.win_name, self.default_top_inset)
        cv2.setTrackbarPos("BEV Margin X", self.win_name, self.default_bev_margin)
        cv2.setTrackbarPos("Lookahead Y", self.win_name, self.default_lookahead_y)
        cv2.setTrackbarPos("Speed (cm/s)", self.win_name, self.default_speed)
        self.update_calibration_from_trackbars()
        print("\n[RESET] Đã khôi phục toàn bộ thông số về giá trị ban đầu!\n")

    def save_calibration(self):
        """In mã nguồn thông số ra Terminal và lưu cấu hình ra file JSON"""
        code_str = f"""
======================================================================
💾 TỌA ĐỘ HIỆU CHUẨN BEV HIỆN TẠI (Copy lưu dùng cho Phần 2 & 3):
======================================================================
THRESHOLD = {self.threshold}
SRC_POINTS = np.float32([
    [{self.src_points[0][0]:.1f}, {self.src_points[0][1]:.1f}],
    [{self.src_points[1][0]:.1f}, {self.src_points[1][1]:.1f}],
    [{self.src_points[2][0]:.1f}, {self.src_points[2][1]:.1f}],
    [{self.src_points[3][0]:.1f}, {self.src_points[3][1]:.1f}],
])
DST_POINTS = np.float32([
    [{self.dst_points[0][0]:.1f}, {self.dst_points[0][1]:.1f}],
    [{self.dst_points[1][0]:.1f}, {self.dst_points[1][1]:.1f}],
    [{self.dst_points[2][0]:.1f}, {self.dst_points[2][1]:.1f}],
    [{self.dst_points[3][0]:.1f}, {self.dst_points[3][1]:.1f}],
])
LOOKAHEAD_Y = {self.lookahead_y}
BASE_SPEED = {self.base_speed:.2f}
======================================================================
"""
        print(code_str)

        calib_data = {
            "threshold": int(self.threshold),
            "src_points": self.src_points.tolist(),
            "dst_points": self.dst_points.tolist(),
            "lookahead_y": int(self.lookahead_y),
            "base_speed": float(self.base_speed)
        }
        for save_path in [
            "/root/bfmc_simulator_ws/src/src/example/src/bev_calib.json",
            "/home/danh/FABLAB/BFMC_Fablab/bev_calib.json",
            "/tmp/bev_calib.json"
        ]:
            try:
                with open(save_path, "w") as f:
                    json.dump(calib_data, f, indent=4)
                print(f"[SAVE] Đã lưu thông số vào: {save_path}")
            except Exception:
                pass

    def preprocess_to_bev(self, frame):
        """Lọc vạch trắng và biến đổi sang Bird's Eye View"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)
        bev = cv2.warpPerspective(binary, self.M_warp, (self.img_w, self.img_h), flags=cv2.INTER_LINEAR)
        return bev

    def track_line_sliding_windows(self, bev, base_x, debug_img, color):
        """Cửa sổ trượt dự đoán tiếp tuyến theo độ dốc dx/dy kèm Curvature Guard"""
        h, w = bev.shape
        nonzero = bev.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        win_h = h // self.nwindows
        cur_x = float(base_x)
        dx_dy = 0.0
        line_inds = []

        for i in range(self.nwindows):
            y_low = h - (i + 1) * win_h
            y_high = h - i * win_h
            win_x_low = int(cur_x - self.margin)
            win_x_high = int(cur_x + self.margin)

            if debug_img is not None:
                cv2.rectangle(debug_img, (max(0, win_x_low), y_low), (min(w, win_x_high), y_high), color, 1)

            good_inds = ((nonzeroy >= y_low) & (nonzeroy < y_high) & 
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            line_inds.append(good_inds)

            if len(good_inds) > self.minpix:
                new_x = float(np.mean(nonzerox[good_inds]))
                dx_dy = 0.65 * ((new_x - cur_x) / win_h) + 0.35 * dx_dy
                cur_x = new_x + dx_dy * win_h
            else:
                cur_x = cur_x + dx_dy * win_h

        line_inds = np.concatenate(line_inds) if len(line_inds) > 0 else np.array([])
        if len(line_inds) > 40:
            fit = np.polyfit(nonzeroy[line_inds], nonzerox[line_inds], 2)
            # Curvature Guard: Chỉ hạ bậc khi độ cong phi lý tuyệt đối (|a| > 0.008, bán kính < 60px)
            if abs(fit[0]) > 0.008:
                lin_fit = np.polyfit(nonzeroy[line_inds], nonzerox[line_inds], 1)
                fit = np.array([0.0, lin_fit[0], lin_fit[1]])
            return fit, line_inds
        return None, np.array([])

    def compute_pure_pursuit_control(self, center_x_vals, ploty):
        """
        Thuật toán điều khiển lái đón đầu Pure Pursuit thích ứng đa dải:
        • Tự động thu ngắn Lookahead khi gặp cua gắt (chống chém cua / văng cua).
        • Tự động hạ tốc độ sâu xuống 0.13 m/s ở khúc cua tay áo (triệt tiêu trượt lốp).
        • Phản xạ đảo lái tức thì khi qua khúc cua chữ S (Chicane).
        """
        # 1. Tự động điều chỉnh cự ly Lookahead theo độ gắt của góc cua
        steer_abs = abs(self.prev_steer)
        if steer_abs > 6.0:
            curve_factor = min(1.0, (steer_abs - 6.0) / 14.0)
            # Đường thẳng nhìn xa (240-260px), vào cua kéo điểm nhìn về sát cản trước (330-350px)
            eff_lookahead_y = int(self.lookahead_y + curve_factor * (340 - self.lookahead_y))
        else:
            eff_lookahead_y = self.lookahead_y

        eff_lookahead_y = max(140, min(eff_lookahead_y, 400))

        if center_x_vals is None:
            target_x = self.prev_target_x
            mode = "MEMORY_COASTING"
        else:
            target_x = center_x_vals[eff_lookahead_y]
            mode = "TRACKING_ACTIVE"

        self.prev_target_x = target_x

        # Tọa độ mũi xe đặt ở chính giữa đáy ảnh BEV (320, 480)
        car_x = float(self.img_w // 2)      # 320 px
        car_y = float(self.img_h)           # 480 px

        # Vector từ tâm xe tới điểm nhìn trước Lookahead
        dx = target_x - car_x
        dy = car_y - float(eff_lookahead_y)  # Cự ly dọc trục nhìn trước thích ứng

        # Tính góc lệch hướng xe tới mục tiêu
        alpha = np.arctan2(dx, dy)
        steer_target = float(np.degrees(alpha)) * self.pursuit_gain
        steer_target = max(min(steer_target, self.max_steer), -self.max_steer)

        # Lọc thông thấp thích ứng: Đảo lái cực nhanh khi qua khúc cua chữ S (Chicane)
        if (steer_target * self.prev_steer) < -1.0:
            # Đang đổi hướng lái trái ➔ phải: tăng tốc phản xạ 90%
            smooth_steer = 0.90 * steer_target + 0.10 * self.prev_steer
        else:
            smooth_steer = 0.70 * steer_target + 0.30 * self.prev_steer
        self.prev_steer = smooth_steer

        # Điều tốc tự động thích ứng sâu (Deep Cornering Deceleration)
        steer_mag = abs(smooth_steer)
        if steer_mag > 16.0:
            # Cua tay áo Hairpin / góc vuông gắt: hãm ga sâu xuống 0.13 m/s để lốp xe bám đường 100%
            speed = 0.13
        elif steer_mag > 7.0:
            ratio = (steer_mag - 7.0) / 9.0
            speed = self.base_speed - ratio * (self.base_speed - 0.13)
        else:
            speed = self.base_speed

        target_point = (int(np.clip(target_x, 10, self.img_w - 10)), int(eff_lookahead_y))
        return smooth_steer, speed, target_point, mode

    def image_callback(self, msg):
        now = time.time()
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(now - self.last_time, 0.001))
        self.last_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        h, w, _ = frame.shape

        # 0. Đồng bộ hóa thông số thanh trượt theo thời gian thực
        self.update_calibration_from_trackbars()

        # 1. Biến đổi ảnh sang BEV
        bev = self.preprocess_to_bev(frame)

        if self.view_binary:
            debug_bev = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        else:
            # Chiếu ảnh camera màu gốc sang BEV để dễ quan sát thực địa
            debug_bev = cv2.warpPerspective(frame, self.M_warp, (w, h), flags=cv2.INTER_LINEAR)
            # Pha trộn mờ vạch nhị phân lên ảnh màu
            bin_mask = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
            debug_bev = cv2.addWeighted(debug_bev, 0.75, bin_mask, 0.25, 0)

        # 2. Tìm chân vạch qua Histogram
        hist = np.sum(bev[260:, :], axis=0)
        peaks = []
        for x in range(30, w - 30):
            if hist[x] > 400 and hist[x] == np.max(hist[max(0, x-30):min(w, x+30)]):
                peaks.append((int(hist[x]), x))
        peaks.sort(reverse=True, key=lambda p: p[0])

        div_base = None
        right_base = None

        # Ghép cặp thông minh: Tìm cặp đỉnh đại diện cho 2 vạch trái/phải (khoảng cách 120px - 320px)
        best_pair = None
        for i in range(len(peaks)):
            for j in range(i + 1, len(peaks)):
                px1, px2 = peaks[i][1], peaks[j][1]
                dist = abs(px1 - px2)
                if 120 <= dist <= 320:
                    best_pair = (min(px1, px2), max(px1, px2))
                    break
            if best_pair is not None:
                break

        if best_pair is not None:
            div_base, right_base = best_pair
        elif len(peaks) > 0:
            px = peaks[0][1]
            if px < w // 2:
                div_base = px
                right_base = min(w - 20, px + int(self.lane_width_bev))
            else:
                right_base = px
                div_base = max(20, px - int(self.lane_width_bev))
        else:
            div_base = 200
            right_base = 450

        # 3. Chạy Cửa sổ trượt
        div_fit, div_inds = self.track_line_sliding_windows(bev, div_base, debug_bev, color=(0, 255, 0))
        right_fit, right_inds = self.track_line_sliding_windows(bev, right_base, debug_bev, color=(255, 140, 0))

        # Tô màu vạch bắt được
        nonzero = bev.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        if len(div_inds) > 0: debug_bev[nonzeroy[div_inds], nonzerox[div_inds]] = [0, 255, 0]
        if len(right_inds) > 0: debug_bev[nonzeroy[right_inds], nonzerox[right_inds]] = [255, 140, 0]

        # 4. Khớp phương trình và tính Tâm làn
        ploty = np.linspace(0, h - 1, h)
        div_x_vals = (div_fit[0]*ploty**2 + div_fit[1]*ploty + div_fit[2]) if div_fit is not None else None
        right_x_vals = (right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]) if right_fit is not None else None

        if div_x_vals is not None:
            pts_div = np.array([np.transpose(np.vstack([div_x_vals, ploty]))], np.int32)
            cv2.polylines(debug_bev, pts_div, isClosed=False, color=(0, 255, 0), thickness=2)

        if right_x_vals is not None:
            pts_right = np.array([np.transpose(np.vstack([right_x_vals, ploty]))], np.int32)
            cv2.polylines(debug_bev, pts_right, isClosed=False, color=(255, 140, 0), thickness=2)

        # Mô hình Đa Giả Thiết tái cấu trúc tâm làn kèm kiểm tra khoảng cách
        if div_x_vals is not None and right_x_vals is not None:
            lane_w_lookahead = right_x_vals[self.lookahead_y] - div_x_vals[self.lookahead_y]
            if lane_w_lookahead < 80:
                # Cả 2 cửa sổ vô tình bắt cùng 1 vạch kẻ
                if div_x_vals[self.lookahead_y] < w // 2:
                    center_x_vals = div_x_vals + self.half_lane_bev
                    lane_status = "DIVIDER_ONLY"
                else:
                    center_x_vals = right_x_vals - self.half_lane_bev
                    lane_status = "RIGHT_ONLY"
            else:
                center_x_vals = (div_x_vals + right_x_vals) / 2.0
                lane_status = "DUAL"
        elif div_x_vals is not None:
            center_x_vals = div_x_vals + self.half_lane_bev
            lane_status = "DIVIDER_ONLY"
        elif right_x_vals is not None:
            center_x_vals = right_x_vals - self.half_lane_bev
            lane_status = "RIGHT_ONLY"
        else:
            center_x_vals = None
            lane_status = "LOST"

        # Vẽ đường tâm làn màu ĐỎ
        if center_x_vals is not None:
            pts_center = np.array([np.transpose(np.vstack([center_x_vals, ploty]))], np.int32)
            cv2.polylines(debug_bev, pts_center, isClosed=False, color=(0, 0, 255), thickness=3)

        # 5. TÍNH GÓC LÁI PURE PURSUIT VÀ VẬN TỐC
        steer_angle, speed, target_pt, control_mode = self.compute_pure_pursuit_control(center_x_vals, ploty)

        # 6. GỬI LỆNH ĐIỀU KHIỂN XE NẾU ĐANG ACTIVE
        if self.is_active:
            cmd_speed = json.dumps({"action": "1", "speed": round(float(speed), 2)})
            cmd_steer = json.dumps({"action": "2", "steerAngle": round(float(steer_angle), 2)})
            self.cmd_pub.publish(cmd_speed)
            time.sleep(0.005)  # Tránh tràn hàng đợi subscriber queue_size=1 của Gazebo C++ plugin
            self.cmd_pub.publish(cmd_steer)
        else:
            self.stop_vehicle()

        # 7. VẼ ĐIỂM NHÌN TRƯỚC LOOKAHEAD VÀ MŨI TÊN DẪN HƯỚNG
        cv2.circle(debug_bev, target_pt, 8, (0, 0, 255), -1)
        cv2.circle(debug_bev, target_pt, 12, (255, 255, 255), 2)
        cv2.arrowedLine(debug_bev, (w // 2, h - 20), target_pt, (0, 255, 255), 2, tipLength=0.15)

        # 8. GHÉP 2 MÀN HÌNH FPV VÀ BEV
        fpv_display = frame.copy()
        # Vẽ chu vi hình thang tuần tự theo vòng khép kín: Đáy trái -> Đáy phải -> Đỉnh phải -> Đỉnh trái
        poly_pts = np.array([
            self.src_points[0],
            self.src_points[1],
            self.src_points[3],
            self.src_points[2]
        ], dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(fpv_display, [poly_pts], isClosed=True, color=(0, 255, 255), thickness=2)

        f_small = cv2.resize(fpv_display, (w // 2, h // 2))
        b_small = cv2.resize(debug_bev, (w // 2, h // 2))
        combined = np.hstack((f_small, b_small))

        # Dashboard Telemetry & Hướng dẫn phím nóng (Dùng ASCII để tránh lỗi font ???? trên OpenCV)
        cv2.rectangle(combined, (0, 0), (combined.shape[1], 46), (20, 20, 20), -1)
        state_str = "[RUNNING]" if self.is_active else "[PAUSED]"
        txt1 = f"FPS: {self.fps:4.1f} | {state_str} (Key 'p') | Lane: {lane_status:12s} | Lookahead: Y={self.lookahead_y}px"
        txt2 = f"Steer: {steer_angle:+5.2f} deg | Speed: {speed:.2f} m/s (Base: {self.base_speed:.2f}) | Pure Pursuit"
        cv2.putText(combined, txt1, (12, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(combined, txt2, (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)

        # Banner chân trang hiển thị phím nóng
        cv2.rectangle(combined, (0, combined.shape[0] - 22), (combined.shape[1], combined.shape[0]), (10, 10, 10), -1)
        txt_hint = "[s]: Save Calib | [t]: Binary/Color | [r]: Reset | [p]: Pause | [q]: Quit"
        cv2.putText(combined, txt_hint, (15, combined.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow(self.win_name, combined)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            self.stop_vehicle()
            rospy.signal_shutdown("Thoát chương trình")
            sys.exit(0)
        elif key == ord('p'):
            self.is_active = not self.is_active
            if not self.is_active:
                self.stop_vehicle()
                print("\n[PAUSE] Đã tạm dừng xe (Phanh khẩn cấp).")
            else:
                print("\n[RESUME] Tiếp tục tự hành lăn bánh.")
        elif key == ord('s'):
            self.save_calibration()
        elif key == ord('t'):
            self.view_binary = not self.view_binary
            mode_name = "NHỊ PHÂN (BINARY)" if self.view_binary else "MÀU GỐC + VẠCH LÀN (COLOR OVERLAY)"
            print(f"\n[VIEW] Chuyển chế độ hiển thị BEV sang: {mode_name}")
        elif key == ord('r'):
            self.reset_calibration()
        elif key in [ord('+'), ord('=')]:
            self.base_speed = min(0.60, self.base_speed + 0.02)
            cv2.setTrackbarPos("Speed (cm/s)", self.win_name, int(round(self.base_speed * 100)))
            print(f"[SPEED] Tốc độ chạy thẳng: {self.base_speed:.2f} m/s")
        elif key in [ord('-'), ord('_')]:
            self.base_speed = max(0.15, self.base_speed - 0.02)
            cv2.setTrackbarPos("Speed (cm/s)", self.win_name, int(round(self.base_speed * 100)))
            print(f"[SPEED] Tốc độ chạy thẳng: {self.base_speed:.2f} m/s")
        elif key == ord(']'):
            self.lookahead_y = max(150, self.lookahead_y - 10)
            cv2.setTrackbarPos("Lookahead Y", self.win_name, self.lookahead_y)
            print(f"[LOOKAHEAD] Nhìn xa hơn: Y = {self.lookahead_y} px")
        elif key == ord('['):
            self.lookahead_y = min(360, self.lookahead_y + 10)
            cv2.setTrackbarPos("Lookahead Y", self.win_name, self.lookahead_y)
            print(f"[LOOKAHEAD] Nhìn gần hơn: Y = {self.lookahead_y} px")

    def stop_vehicle(self):
        self.cmd_pub.publish(json.dumps({"action": "3", "steerAngle": 0.0}))
        time.sleep(0.005)
        self.cmd_pub.publish(json.dumps({"action": "1", "speed": 0.0}))


if __name__ == '__main__':
    try:
        follower = PurePursuitLaneFollowerWithTuner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

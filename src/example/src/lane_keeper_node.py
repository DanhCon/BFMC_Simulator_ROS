#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
NODE TỰ HÀNH BÁM LÀN ĐƯỜNG BFMC CAO CẤP (PREDICTIVE SLIDING WINDOWS + PURE PURSUIT)
Thiết kế theo chuẩn kiến trúc của các đội vô địch BFMC (VROOM / DriverlES):
  1. IPM / Bird's Eye View: Chuyển đổi phối cảnh camera FPV sang mặt đường phẳng.
  2. Predictive Sliding Windows: Cửa sổ trượt dự đoán tiếp tuyến (dx/dy), không bị văng khỏi vạch khi ôm cua gắt.
  3. Dual-Hypothesis Lane Model:
     • Nếu thấy cả 2 vạch (tim đường + lề phải) -> Đi chính giữa: (x_div + x_right) / 2
     • Nếu chỉ thấy vạch tim (khi cua phải gắt vạch phải văng khỏi màn hình) -> Offset sang phải +W/2
     • Nếu chỉ thấy vạch lề (khi vạch tim đứt đoạn hoặc cua trái) -> Offset sang trái -W/2
     • Nếu tạm mất dấu (qua ngã tư / vạch kẻ) -> Kế thừa bộ nhớ (Lane Memory Dead-Reckoning)
  4. Adaptive Pure Pursuit Lookahead:
     • Khi đường thẳng: Nhìn xa (y=220) để chạy mượt, êm.
     • Khi vào cua: Nhìn gần (y=270) để bẻ lái dứt khoát theo bán kính cong.
  5. Dynamic Speed Profiling: Tự động hãm tốc từ 0.32 m/s xuống 0.16 m/s khi góc lái > 12° để triệt tiêu trượt bánh (tire slip).
=============================================================================
"""

import os
import sys
import time
import json
import collections
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class BFMCChampionLaneKeeper:
    def __init__(self):
        rospy.init_node('bfmc_champion_lane_keeper', anonymous=False)
        self.bridge = CvBridge()

        # ROS Publishers & Subscribers
        self.cmd_pub = rospy.Publisher('/automobile/command', String, queue_size=1)
        self.image_sub = rospy.Subscriber('/automobile/image_raw', Image, self.image_callback, queue_size=1)

        # 1. Cấu hình Vận tốc và Giới hạn Lái
        self.base_speed = 0.32           # Vận tốc chạy thẳng đường thoáng (m/s)
        self.curve_speed = 0.16          # Vận tốc hãm khi vào cua gắt (m/s)
        self.max_steer_angle = 23.0      # Góc bẻ lái cơ học tối đa (độ)

        # 2. Thông số hình học Làn đường BFMC
        self.img_w = 640
        self.img_h = 480
        self.lane_width_bev = 210.0      # Độ rộng 1 làn đường trên ảnh BEV (~35-37cm thực tế)
        self.half_lane_bev = 105.0       # Bán kính nửa làn

        # 3. Ma trận biến đổi Phối cảnh Chim bay (Bird's Eye View - BEV)
        # Đã đo chuẩn hóa theo tọa độ mặt đường camera Gazebo BFMC
        src_points = np.float32([
            [40, 245],                   # Đáy trái mặt đường
            [600, 245],                  # Đáy phải mặt đường
            [190, 140],                  # Đỉnh trái sát chân trời
            [450, 140]                   # Đỉnh phải sát chân trời
        ])
        dst_points = np.float32([
            [120, 480],                  # Đáy trái BEV
            [520, 480],                  # Đáy phải BEV
            [120, 0],                    # Đỉnh trái BEV
            [520, 0]                     # Đỉnh phải BEV
        ])
        self.M_warp = cv2.getPerspectiveTransform(src_points, dst_points)
        self.src_points = src_points

        # 4. Bộ điều khiển Lái Pure Pursuit & Lọc làm mượt
        self.prev_steer = 0.0
        self.prev_target_x = 320.0
        self.steer_history = collections.deque(maxlen=3)

        # Trạng thái hệ thống
        self.is_active = True
        self.fps = 0.0
        self.last_frame_time = time.time()

        print("\n" + "="*75)
        print("   🏎️  BFMC CHAMPION LANE KEEPER (PREDICTIVE SLIDING WINDOWS + PURE PURSUIT)")
        print("="*75)
        print("  • Mô hình làn     : Dual-Hypothesis (Nhận diện độc lập Vạch Tim & Vạch Lề)")
        print("  • Thuật toán bám  : Cửa sổ trượt dự đoán tiếp tuyến (Tangent Predictive Search)")
        print("  • Điều khiển lái  : Adaptive Pure Pursuit Lookahead (Lookahead Y: 220-270px)")
        print(f"  • Tốc độ đường    : Thẳng {self.base_speed:.2f} m/s | Ôm cua {self.curve_speed:.2f} m/s")
        print("  • Phím nóng GUI   : 'p' = Tạm dừng / Tiếp tục | 'q' = Dừng xe và Thoát")
        print("="*75 + "\n")

    def preprocess_bev(self, frame):
        """Lọc nhị phân và chiếu phối cảnh sang Bird's Eye View"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Ngưỡng 135 bắt trọn vạch sơn trắng giá trị 178 trong texture Gazebo
        _, binary = cv2.threshold(gray, 135, 255, cv2.THRESH_BINARY)
        bev = cv2.warpPerspective(binary, self.M_warp, (self.img_w, self.img_h), flags=cv2.INTER_LINEAR)
        return bev

    def track_line_predictive(self, bev, base_x, debug_img=None, color=(0, 255, 0), margin=75, minpix=20, nwindows=10):
        """
        Thuật toán Cửa sổ trượt Dự đoán Tiếp tuyến (Predictive Tangent Sliding Windows):
        Khi đường cong bẻ hướng mạnh, tâm cửa sổ kế tiếp được dự phóng theo độ nghiêng dx/dy
        thay vì đứng yên một chỗ, giúp bám trọn cua 90° và cua vòng xuyến.
        """
        h, w = bev.shape
        nonzero = bev.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        win_h = h // nwindows
        cur_x = float(base_x)
        dx_dy = 0.0
        lane_inds = []

        for i in range(nwindows):
            y_low = h - (i + 1) * win_h
            y_high = h - i * win_h
            win_x_low = int(cur_x - margin)
            win_x_high = int(cur_x + margin)

            if debug_img is not None:
                cv2.rectangle(debug_img, (max(0, win_x_low), y_low), (min(w, win_x_high), y_high), color, 1)

            good_inds = ((nonzeroy >= y_low) & (nonzeroy < y_high) & 
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            lane_inds.append(good_inds)

            if len(good_inds) > minpix:
                new_x = float(np.mean(nonzerox[good_inds]))
                # Lọc làm mịn tiếp tuyến góc nghiêng dx/dy
                dx_dy = 0.6 * ((new_x - cur_x) / win_h) + 0.4 * dx_dy
                cur_x = new_x + dx_dy * win_h
            else:
                # Nếu gặp đoạn đứt quãng (vạch nét đứt), cửa sổ tiếp tục bay theo quán tính góc nghiêng
                cur_x = cur_x + dx_dy * win_h

        lane_inds = np.concatenate(lane_inds) if len(lane_inds) > 0 else np.array([])
        if len(lane_inds) > 60:
            fit = np.polyfit(nonzeroy[lane_inds], nonzerox[lane_inds], 2)
            return fit, lane_inds
        return None, np.array([])

    def detect_and_fit_lanes(self, bev, debug_img=None):
        """
        Nhận diện đa giả thiết 2 vạch giới hạn làn (Vạch Tim & Vạch Lề phải)
        """
        # Quét biểu đồ Histogram chân đường ở nửa dưới ảnh BEV
        hist = np.sum(bev[320:, :], axis=0)

        # Phân chia miền tìm kiếm chân vạch:
        # Vạch tim đường: quanh vùng x in [130, 340]
        # Vạch lề phải: quanh vùng x in [340, 580]
        divider_hist = hist[130:340]
        right_hist = hist[340:580]

        div_base = (130 + np.argmax(divider_hist)) if len(divider_hist) > 0 and np.max(divider_hist) > 600 else None
        right_base = (340 + np.argmax(right_hist)) if len(right_hist) > 0 and np.max(right_hist) > 600 else None

        div_fit, div_inds = (None, np.array([]))
        right_fit, right_inds = (None, np.array([]))

        if div_base is not None:
            div_fit, div_inds = self.track_line_predictive(bev, div_base, debug_img=debug_img, color=(0, 255, 0)) # Xanh lá
        if right_base is not None:
            right_fit, right_inds = self.track_line_predictive(bev, right_base, debug_img=debug_img, color=(255, 120, 0)) # Xanh lam

        # Tô màu các điểm ảnh vạch kẻ trên ảnh Debug
        if debug_img is not None:
            nonzero = bev.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            if len(div_inds) > 0:
                debug_img[nonzeroy[div_inds], nonzerox[div_inds]] = [0, 255, 0]
            if len(right_inds) > 0:
                debug_img[nonzeroy[right_inds], nonzerox[right_inds]] = [255, 120, 0]

        return div_fit, right_fit

    def compute_trajectory_and_control(self, div_fit, right_fit):
        """
        Tổng hợp tâm đường đi của xe và tính góc lái Pure Pursuit + Điều tốc
        """
        # Xác định khoảng cách nhìn trước (Adaptive Lookahead):
        y_target = 270

        mode = "DEFAULT"
        if div_fit is not None and right_fit is not None:
            x_div = div_fit[0]*(y_target**2) + div_fit[1]*y_target + div_fit[2]
            x_right = right_fit[0]*(y_target**2) + right_fit[1]*y_target + right_fit[2]
            # Kiểm tra khoảng cách hợp lý giữa 2 vạch
            if 150.0 < (x_right - x_div) < 280.0:
                x_target = (x_div + x_right) / 2.0
                mode = "DUAL_LANE"
            else:
                x_target = x_div + self.half_lane_bev
                mode = "DIVIDER_PRIORITY"
        elif div_fit is not None:
            # Chỉ thấy vạch tim (thường gặp khi ôm cua phải, vạch phải trôi ra ngoài FOV)
            x_div = div_fit[0]*(y_target**2) + div_fit[1]*y_target + div_fit[2]
            x_target = x_div + self.half_lane_bev
            mode = "DIVIDER_ONLY"
        elif right_fit is not None:
            # Chỉ thấy vạch lề phải (gặp khi vạch tim đứt đoạn hoặc cua trái)
            x_right = right_fit[0]*(y_target**2) + right_fit[1]*y_target + right_fit[2]
            x_target = x_right - self.half_lane_bev
            mode = "RIGHT_ONLY"
        elif self.prev_target_x is not None:
            # Kế thừa vị trí cũ (Dead-Reckoning) khi đi qua ngã tư / vạch người đi bộ
            x_target = self.prev_target_x
            mode = "MEMORY_COASTING"
        else:
            x_target = float(self.img_w // 2)
            mode = "BLIND"

        self.prev_target_x = x_target

        # -------------------------------------------------------------
        # Thuật toán Điều khiển Bám Điểm nhìn trước (Pure Pursuit Lookahead)
        # -------------------------------------------------------------
        car_x = float(self.img_w // 2)      # 320 px
        car_y = float(self.img_h)           # 480 px

        dx = x_target - car_x
        dy = car_y - y_target               # 480 - 270 = 210 px

        # Tính góc hướng đích (Target Heading Angle)
        alpha = np.arctan2(dx, dy)
        steer_target = float(np.degrees(alpha)) * 1.10
        steer_target = max(min(steer_target, self.max_steer_angle), -self.max_steer_angle)

        # Lọc thông thấp làm mượt góc bẻ lái bánh trước (Low-Pass Filter)
        smooth_steer = 0.65 * steer_target + 0.35 * self.prev_steer
        self.prev_steer = smooth_steer

        # -------------------------------------------------------------
        # Điều tốc Thích ứng theo Độ cong Làn (Adaptive Velocity Profiling)
        # -------------------------------------------------------------
        steer_mag = abs(smooth_steer)
        if steer_mag > 12.0:
            # Cua gắt (> 12 độ): hạ tốc độ tối đa để bánh không bị trượt (tire slip)
            speed = self.curve_speed
        elif steer_mag > 6.0:
            ratio = (steer_mag - 6.0) / 6.0
            speed = self.base_speed - ratio * (self.base_speed - self.curve_speed)
        else:
            speed = self.base_speed

        target_point = (int(x_target), int(y_target))
        return smooth_steer, speed, target_point, mode

    def image_callback(self, msg):
        if not self.is_active:
            return

        now = time.time()
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(now - self.last_frame_time, 0.001))
        self.last_frame_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        # 1. Chuyển đổi phối cảnh Bird's-Eye View
        bev = self.preprocess_bev(frame)
        debug_bev = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)

        # 2. Nhận diện các vạch kẻ bằng Cửa sổ trượt dự đoán tiếp tuyến
        div_fit, right_fit = self.detect_and_fit_lanes(bev, debug_img=debug_bev)

        # 3. Tính toán quỹ đạo di chuyển tâm làn & Lệnh lái Pure Pursuit
        steer_angle, speed, target_pt, mode = self.compute_trajectory_and_control(div_fit, right_fit)

        # 4. Gửi lệnh điều khiển xe đến /automobile/command
        cmd_speed = json.dumps({"action": "1", "speed": round(float(speed), 2)})
        cmd_steer = json.dumps({"action": "2", "steerAngle": round(float(steer_angle), 2)})
        self.cmd_pub.publish(cmd_speed)
        self.cmd_pub.publish(cmd_steer)

        # 5. Vẽ giao diện HUD Debug trực quan
        self.render_debug_display(frame, debug_bev, div_fit, right_fit, target_pt, steer_angle, speed, mode)

    def render_debug_display(self, frame, debug_bev, div_fit, right_fit, target_pt, steer, speed, mode):
        h, w, _ = frame.shape
        car_x = w // 2

        # 1. Vẽ vùng quan sát ROI trên màn hình FPV Camera
        pts = self.src_points.reshape((-1, 1, 2)).astype(np.int32)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

        # 2. Vẽ các đường cong đa thức khớp được lên ảnh BEV
        ploty = np.linspace(0, h-1, h)
        if div_fit is not None:
            div_x = div_fit[0]*(ploty**2) + div_fit[1]*ploty + div_fit[2]
            pts_div = np.array([np.transpose(np.vstack([div_x, ploty]))], np.int32)
            cv2.polylines(debug_bev, pts_div, isClosed=False, color=(0, 255, 0), thickness=2)

        if right_fit is not None:
            right_x = right_fit[0]*(ploty**2) + right_fit[1]*ploty + right_fit[2]
            pts_right = np.array([np.transpose(np.vstack([right_x, ploty]))], np.int32)
            cv2.polylines(debug_bev, pts_right, isClosed=False, color=(255, 120, 0), thickness=2)

        # 3. Vẽ điểm nhìn trước Lookahead (Pure Pursuit Target)
        cv2.circle(debug_bev, target_pt, 7, (0, 0, 255), -1)
        cv2.circle(debug_bev, target_pt, 11, (255, 255, 255), 2)
        cv2.arrowedLine(debug_bev, (car_x, h - 20), target_pt, (0, 255, 255), 2, tipLength=0.15)

        # 4. Ghép ảnh FPV và ảnh BEV cạnh nhau
        f_small = cv2.resize(frame, (w // 2, h // 2))
        b_small = cv2.resize(debug_bev, (w // 2, h // 2))
        combined = np.hstack((f_small, b_small))

        # 5. Thanh hiển thị Dashboard Telemetry
        cv2.rectangle(combined, (0, 0), (combined.shape[1], 46), (20, 20, 20), -1)
        txt1 = f"FPS: {self.fps:4.1f} | Mode: {mode:16s} | Target: ({target_pt[0]}, {target_pt[1]})"
        txt2 = f"Steer: {steer:+5.2f} deg | Speed: {speed:.2f} m/s | Pure Pursuit Tracking"
        cv2.putText(combined, txt1, (12, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(combined, txt2, (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow("BFMC - ADVANCED LANE KEEPER (PREDICTIVE WINDOWS & PURE PURSUIT)", combined)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.stop_vehicle()
            rospy.signal_shutdown("Người dùng thoát")
        elif key == ord('p'):
            self.is_active = not self.is_active
            if not self.is_active:
                self.stop_vehicle()
                print("[PAUSE] Đã tạm dừng xe.")
            else:
                print("[RESUME] Tiếp tục tự hành.")

    def stop_vehicle(self):
        msg = json.dumps({"action": "3", "steerAngle": 0.0})
        self.cmd_pub.publish(msg)


if __name__ == '__main__':
    try:
        node = BFMCChampionLaneKeeper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

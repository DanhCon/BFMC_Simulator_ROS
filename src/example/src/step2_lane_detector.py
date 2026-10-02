#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
BƯỚC 2: NHẬN DIỆN LÀN ĐƯỜNG & KHỚP ĐA THỨC (SLIDING WINDOWS & POLYFIT)
Đã nâng cấp thuật toán bám đường chéo/ôm cua gắt (Adaptive Slope Tracking):
  • Mở rộng biên độ tìm kiếm margin = 80px để không bị văng khỏi vạch chéo.
  • Dự đoán tâm cửa sổ tiếp theo theo tiếp tuyến dx/dy.
  • Chỗ thay đổi thông số BEV được gom gọn ở ngay đầu hàm __init__.
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
from cv_bridge import CvBridge, CvBridgeError


class LaneDetectorVisualizer:
    def __init__(self):
        rospy.init_node('bfmc_step2_lane_detector', anonymous=False)
        self.bridge = CvBridge()

        self.img_w = 640
        self.img_h = 480
        self.lane_width_bev = 210.0      # Độ rộng 1 làn đường trên ảnh BEV (~35-37cm)
        self.half_lane_bev = 105.0

        # =====================================================================
        # 📌 [CHỖ ÁP DỤNG THÔNG SỐ BEV / IPM TẠI ĐÂY]
        # =====================================================================
        # 1. Ngưỡng nhị phân (Threshold)
        self.threshold = 126

        # 2. 4 Điểm nguồn hình thang trên ảnh FPV Camera (SRC_POINTS)
        src_points = np.float32([
                    [0.0, 429.0],
                    [640.0, 429.0],
                    [207.0, 170.0],
                    [433.0, 170.0],
        ])

        # 3. 4 Điểm đích hình chữ nhật trên ảnh Chim bay BEV (DST_POINTS)
        dst_points = np.float32([
    [138.0, 480.0],
    [502.0, 480.0],
    [138.0, 0.0],
    [502.0, 0.0],
        ])
        # =====================================================================

        self.M_warp = cv2.getPerspectiveTransform(src_points, dst_points)
        self.src_points = src_points
        self.dst_points = dst_points

        # Cấu hình Cửa sổ trượt nâng cao (Chống văng khi cua gắt / đường chéo)
        self.nwindows = 9
        self.margin = 80                 # Mở rộng margin 80px để bắt trọn cua chéo
        self.minpix = 15                 # Ngưỡng kích hoạt tối thiểu

        # Tùy chọn hiển thị
        self.show_histogram = True
        self.show_windows = True
        self.show_curves = True

        self.current_frame = None
        self.fps = 0.0
        self.last_time = time.time()
        self.win_name = "BFMC - PHAN 2: LANE DETECTOR & SLIDING WINDOWS"

        # Đăng ký nhận hình ảnh FPV
        self.image_sub = rospy.Subscriber('/automobile/image_raw', Image, self.image_callback, queue_size=1)

        print("\n" + "="*75)
        print("   🔍  BFMC STEP 2: LANE DETECTOR & SLIDING WINDOWS VISUALIZER")
        print("="*75)
        print("  • Mô-đun này giữ XE ĐỨNG YÊN để bạn thoải mái quan sát thuật toán.")
        print("  • Cửa sổ Xanh Lá : Cửa sổ trượt bám Vạch Tim (Center Divider)")
        print("  • Cửa sổ Xanh Lam: Cửa sổ trượt bám Vạch Lề (Right Border)")
        print("  • Đường Đỏ Rực   : Quỹ đạo Tâm làn đường xe cần bám theo (Centerline)")
        print("\n  • PHÍM TẮT TƯƠNG TÁC TRÊN MÀN HÌNH:")
        print("     - Phím 'h': Bật/Tắt biểu đồ Histogram chân đường.")
        print("     - Phím 'w': Bật/Tắt hiển thị khung hộp cửa sổ trượt.")
        print("     - Phím 'c': Bật/Tắt hiển thị đường cong đa thức.")
        print("     - Phím 's': Chụp ảnh phân tích lưu ra file.")
        print("     - Phím 'q': Thoát chương trình.")
        print("="*75 + "\n")

    def image_callback(self, msg):
        now = time.time()
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(now - self.last_time, 0.001))
        self.last_time = now

        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        self.process_and_visualize()

    def preprocess_to_bev(self, frame):
        """Lọc vạch trắng và biến đổi sang Bird's Eye View"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)
        bev = cv2.warpPerspective(binary, self.M_warp, (self.img_w, self.img_h), flags=cv2.INTER_LINEAR)
        return bev

    def track_line_sliding_windows(self, bev, base_x, debug_img, color, win_name_tag="Line"):
        """
        Cửa sổ trượt Dự đoán Tiếp tuyến (Predictive Tangent Windows):
        Theo dõi độ dốc dx/dy để bẻ lái cửa sổ bám trọn đường cong uốn lượn.
        """
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

            # Vẽ khung cửa sổ trượt
            if self.show_windows and debug_img is not None:
                cv2.rectangle(debug_img, (max(0, win_x_low), y_low), (min(w, win_x_high), y_high), color, 1)

            good_inds = ((nonzeroy >= y_low) & (nonzeroy < y_high) & 
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            line_inds.append(good_inds)

            # Nếu cửa sổ bắt trúng điểm ảnh, cập nhật tâm và độ nghiêng tiếp tuyến
            if len(good_inds) > self.minpix:
                new_x = float(np.mean(nonzerox[good_inds]))
                dx_dy = 0.65 * ((new_x - cur_x) / win_h) + 0.35 * dx_dy
                cur_x = new_x + dx_dy * win_h
            else:
                # Quán tính tiếp tuyến khi gặp nét đứt
                cur_x = cur_x + dx_dy * win_h

        line_inds = np.concatenate(line_inds) if len(line_inds) > 0 else np.array([])
        
        # Khớp phương trình Parabol bậc 2: x = ay² + by + c
        if len(line_inds) > 40:
            fit = np.polyfit(nonzeroy[line_inds], nonzerox[line_inds], 2)
            if abs(fit[0]) > 0.003:
                lin_fit = np.polyfit(nonzeroy[line_inds], nonzerox[line_inds], 1)
                fit = np.array([0.0, lin_fit[0], lin_fit[1]])
            return fit, line_inds
        return None, np.array([])

    def draw_histogram(self, bev, display_img):
        """Vẽ biểu đồ Histogram mật độ điểm ảnh ở nửa dưới màn hình"""
        h, w = bev.shape
        hist = np.sum(bev[int(h * 0.55):, :], axis=0)
        max_val = np.max(hist)
        if max_val == 0:
            return

        hist_h = 70
        hist_img = np.zeros((hist_h, w, 3), dtype=np.uint8)
        norm_hist = (hist / max_val) * (hist_h - 10)
        for x in range(w):
            val = int(norm_hist[x])
            if val > 0:
                cv2.line(hist_img, (x, hist_h - 1), (x, hist_h - 1 - val), (0, 255, 255), 1)

        overlay = display_img[h - hist_h:h, 0:w]
        cv2.addWeighted(hist_img, 0.7, overlay, 0.3, 0, overlay)
        cv2.putText(display_img, "HISTOGRAM MẬT ĐỘ CHÂN ĐƯỜNG", (15, h - hist_h + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1, cv2.LINE_AA)

    def process_and_visualize(self):
        if self.current_frame is None:
            return

        frame = self.current_frame.copy()
        h, w, _ = frame.shape

        # 1. Biến đổi ảnh FPV sang Bird's-Eye View
        bev = self.preprocess_to_bev(frame)
        debug_bev = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)

        # 2. Tìm điểm xuất phát (Bases) qua Histogram
        hist = np.sum(bev[260:, :], axis=0)

        # Tìm 2 đỉnh lớn nhất cách nhau tối thiểu 120px
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

        # 3. Theo dõi 2 vạch bằng Cửa sổ trượt
        div_fit, div_inds = (None, np.array([]))
        right_fit, right_inds = (None, np.array([]))

        if div_base is not None:
            div_fit, div_inds = self.track_line_sliding_windows(
                bev, div_base, debug_bev, color=(0, 255, 0), win_name_tag="Divider")
        if right_base is not None:
            right_fit, right_inds = self.track_line_sliding_windows(
                bev, right_base, debug_bev, color=(255, 140, 0), win_name_tag="Right")

        # Tô màu các điểm ảnh bắt trúng
        nonzero = bev.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        if len(div_inds) > 0:
            debug_bev[nonzeroy[div_inds], nonzerox[div_inds]] = [0, 255, 0]      # Xanh lá
        if len(right_inds) > 0:
            debug_bev[nonzeroy[right_inds], nonzerox[right_inds]] = [255, 140, 0]  # Xanh lam

        # 4. Khớp và vẽ các đường cong Parabol
        ploty = np.linspace(0, h - 1, h)
        mode = "MẤT LÀN"

        if self.show_curves:
            div_x_vals = None
            right_x_vals = None

            if div_fit is not None:
                div_x_vals = div_fit[0]*(ploty**2) + div_fit[1]*ploty + div_fit[2]
                pts_div = np.array([np.transpose(np.vstack([div_x_vals, ploty]))], np.int32)
                cv2.polylines(debug_bev, pts_div, isClosed=False, color=(0, 255, 0), thickness=2)

            if right_fit is not None:
                right_x_vals = right_fit[0]*(ploty**2) + right_fit[1]*ploty + right_fit[2]
                pts_right = np.array([np.transpose(np.vstack([right_x_vals, ploty]))], np.int32)
                cv2.polylines(debug_bev, pts_right, isClosed=False, color=(255, 140, 0), thickness=2)

            # 5. TÍNH TOÁN ĐƯỜNG TÂM LÀN (CENTERLINE TRAJECTORY)
            if div_x_vals is not None and right_x_vals is not None:
                center_x_vals = (div_x_vals + right_x_vals) / 2.0
                mode = "DUAL (Đủ 2 vạch)"
            elif div_x_vals is not None:
                center_x_vals = div_x_vals + self.half_lane_bev
                mode = "TIM DUY NHẤT (+W/2)"
            elif right_x_vals is not None:
                center_x_vals = right_x_vals - self.half_lane_bev
                mode = "LỀ DUY NHẤT (-W/2)"
            else:
                center_x_vals = None

            if center_x_vals is not None:
                pts_center = np.array([np.transpose(np.vstack([center_x_vals, ploty]))], np.int32)
                cv2.polylines(debug_bev, pts_center, isClosed=False, color=(0, 0, 255), thickness=3)

                # Điểm nhìn trước Lookahead
                target_lookahead = (int(center_x_vals[250]), 250)
                cv2.circle(debug_bev, target_lookahead, 8, (0, 0, 255), -1)
                cv2.circle(debug_bev, target_lookahead, 12, (255, 255, 255), 2)
                cv2.arrowedLine(debug_bev, (w // 2, h - 20), target_lookahead, (0, 255, 255), 2, tipLength=0.15)

        if self.show_histogram:
            self.draw_histogram(bev, debug_bev)

        # 6. Vẽ khung hình thang ROI lên ảnh FPV
        fpv_display = frame.copy()
        pts = self.src_points.reshape((-1, 1, 2)).astype(np.int32)
        cv2.polylines(fpv_display, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

        # 7. Ghép 2 ảnh cạnh nhau
        f_small = cv2.resize(fpv_display, (w // 2, h // 2))
        b_small = cv2.resize(debug_bev, (w // 2, h // 2))
        combined = np.hstack((f_small, b_small))

        # 8. Dashboard
        cv2.rectangle(combined, (0, 0), (combined.shape[1], 46), (20, 20, 20), -1)
        txt1 = f"FPS: {self.fps:4.1f} | Chế độ: {mode:20s} | Cửa sổ: {'BẬT' if self.show_windows else 'TẮT'} (Phím 'w')"
        txt2 = f"Vạch Tim: {len(div_inds):4d} pts | Vạch Lề: {len(right_inds):4d} pts | Margin={self.margin}px | Phím 'h': Hist"
        cv2.putText(combined, txt1, (12, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(combined, txt2, (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow(self.win_name, combined)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            rospy.signal_shutdown("Thoát chương trình")
            sys.exit(0)
        elif key == ord('h'):
            self.show_histogram = not self.show_histogram
        elif key == ord('w'):
            self.show_windows = not self.show_windows
        elif key == ord('c'):
            self.show_curves = not self.show_curves
        elif key == ord('s'):
            filename = f"/home/danh/FABLAB/BFMC_Fablab/lane_detect_snap_{int(time.time())}.png"
            cv2.imwrite(filename, combined)
            print(f"[SNAPSHOT] Đã lưu ảnh chụp phân tích: {filename}")


if __name__ == '__main__':
    try:
        detector = LaneDetectorVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

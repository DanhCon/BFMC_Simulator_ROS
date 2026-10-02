#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
NODE ĐIỀU KHIỂN XE TỰ HÀNH BFMC QUA TAY CẦM BLUETOOTH (RC DUAL-STICK MODE)
  • Cần Trái (Lên / Xuống) : TIẾN / LÙI (Throttle / Reverse)
  • Cần Phải (Trái / Phải) : BẺ LÁI (Steering Left / Right)
=============================================================================
"""

import os
import sys
import time
import json
import subprocess

# Chạy Pygame ở chế độ Headless / Background cho Terminal
os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import pygame
import rospy
from std_msgs.msg import String


def sync_device_nodes():
    """Tự động đồng bộ device node từ host vào container nếu bị thiếu"""
    if not os.path.exists("/dev/input/js0"):
        try:
            subprocess.run(["mknod", "/dev/input/js0", "c", "13", "0"], stderr=subprocess.DEVNULL)
            for i in range(16, 32):
                dev_path = f"/dev/input/event{i}"
                if not os.path.exists(dev_path):
                    subprocess.run(["mknod", dev_path, "c", "13", str(64 + i)], stderr=subprocess.DEVNULL)
            subprocess.run(["chmod", "-R", "666", "/dev/input"], stderr=subprocess.DEVNULL)
        except Exception:
            pass


class GamepadTeleopNode:
    def __init__(self):
        # 1. Khởi tạo ROS Node & Publisher
        rospy.init_node('bfmc_gamepad_teleop', anonymous=False)
        self.cmd_pub = rospy.Publisher('/automobile/command', String, queue_size=1)
        
        # 2. Cấu hình tham số điều khiển
        self.max_steer_angle = 25.0       # Góc lái tối đa (độ, BFMC chuẩn: ±25°)
        self.max_speed = 0.45             # Tốc độ tối đa mặc định (m/s)
        self.speed_step = 0.05            # Bước tăng/giảm tốc độ max
        self.min_max_speed = 0.15         # Giới hạn tốc độ nhỏ nhất
        self.max_max_speed = 1.20         # Giới hạn tốc độ lớn nhất
        self.deadzone = 0.08              # Vùng chết analog chống trôi cần

        # Biến trạng thái
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.prev_speed = 0.0
        self.prev_steer = 0.0
        self.emergency_stopped = False

        # 3. Khởi tạo Pygame Joystick
        sync_device_nodes()
        pygame.init()
        pygame.joystick.init()
        self.joystick = None

    def wait_for_gamepad(self):
        """Chờ và tự động nhận diện khi tay cầm Bluetooth được kết nối"""
        print("\n" + "="*68)
        print("      🎮 BFMC BLUETOOTH GAMEPAD (DUAL-STICK MODE)")
        print("="*68)
        
        while not rospy.is_shutdown():
            sync_device_nodes()
            pygame.joystick.quit()
            pygame.joystick.init()
            pygame.event.pump()
            
            count = pygame.joystick.get_count()
            if count > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(f"[THÀNH CÔNG] Đã nhận diện tay cầm: '{self.joystick.get_name()}'")
                print(f"  • Số trục Analog (Axes) : {self.joystick.get_numaxes()}")
                print(f"  • Số nút bấm (Buttons)  : {self.joystick.get_numbuttons()}")
                print("="*68)
                self._print_guide()
                return True
            else:
                sys.stdout.write("\r[TÌM KIẾM] Đang đợi kết nối tay cầm Bluetooth...")
                sys.stdout.flush()
                time.sleep(1.0)
        return False

    def _print_guide(self):
        print("\n📋 SƠ ĐỒ ĐIỀU KHIỂN (RC DUAL-STICK):")
        print("  • 🕹️ CẦN TRÁI  (Đẩy Lên / Kéo Xuống) : TIẾN / LÙI (Ga từ 0 -> Max Speed)")
        print("  • 🕹️ CẦN PHẢI  (Gạt Trái / Gạt Phải) : BẺ LÁI (Góc lái từ -25° -> +25°)")
        print("  • 🛑 Nút Tròn (hoặc nút B)           : PHANH KHẨN CẤP (Dừng tức thì)")
        print("  • ⚡ D-Pad Lên/Xuống hoặc R1/L1      : Tăng / Giảm Max Speed")
        print("  • ❌ Nhấn Ctrl + C                   : Thoát chương trình\n")
        print("="*68 + "\n")

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def send_speed(self, speed):
        """Gửi lệnh tốc độ: Action 1"""
        speed = round(float(speed), 3)
        if abs(speed - self.prev_speed) >= 0.02 or (speed == 0.0 and self.prev_speed != 0.0):
            msg = json.dumps({"action": "1", "speed": speed})
            self.cmd_pub.publish(msg)
            self.prev_speed = speed

    def send_steer(self, steer_angle):
        """Gửi lệnh góc lái: Action 2"""
        steer_angle = round(float(steer_angle), 2)
        if abs(steer_angle - self.prev_steer) >= 0.5 or (steer_angle == 0.0 and self.prev_steer != 0.0):
            msg = json.dumps({"action": "2", "steerAngle": steer_angle})
            self.cmd_pub.publish(msg)
            self.prev_steer = steer_angle

    def send_brake(self):
        """Gửi lệnh phanh dừng khẩn cấp: Action 3"""
        msg = json.dumps({"action": "3", "steerAngle": 0.0})
        self.cmd_pub.publish(msg)
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.prev_speed = 0.0
        self.prev_steer = 0.0
        self.emergency_stopped = True

    def render_hud(self):
        """Hiển thị bảng điều khiển trực quan trên Terminal"""
        # Thanh hiển thị góc lái Cần Phải
        bar_len = 21
        steer_ratio = (self.current_steer + self.max_steer_angle) / (2 * self.max_steer_angle)
        marker_pos = int(steer_ratio * (bar_len - 1))
        steer_bar = [" "] * bar_len
        steer_bar[bar_len // 2] = "|"
        steer_bar[marker_pos] = "█"
        steer_str = "".join(steer_bar)

        # Thanh hiển thị tốc độ Cần Trái
        speed_ratio = min(max(abs(self.current_speed) / self.max_speed, 0.0), 1.0)
        filled = int(speed_ratio * 12)
        speed_bar = "█" * filled + "░" * (12 - filled)

        status_text = "PHANH DỪNG!" if self.emergency_stopped else "ĐANG CHẠY"

        hud = (
            f"\r[CẦN PHẢI - LÁI] ◄ [{steer_str}] ► {self.current_steer:+5.1f}° | "
            f"[CẦN TRÁI - GA] [{speed_bar}] {self.current_speed:+5.2f} m/s (Max: {self.max_speed:.2f}) | "
            f"[{status_text}]  "
        )
        sys.stdout.write(hud)
        sys.stdout.flush()

    def run(self):
        if not self.wait_for_gamepad():
            return

        rate = rospy.Rate(30)  # Tần số 30 Hz

        # Xác định trục Cần Phải X (thường là Axis 3 hoặc Axis 2)
        right_stick_x_axis = 3
        if self.joystick.get_numaxes() <= 3:
            right_stick_x_axis = 2

        while not rospy.is_shutdown():
            pygame.event.pump()

            # 1. Đọc nút bấm phanh khẩn cấp & chỉnh max speed
            for btn_idx in range(self.joystick.get_numbuttons()):
                if self.joystick.get_button(btn_idx):
                    # Nút 1 (Tròn trên PS, B trên Xbox): Phanh khẩn cấp
                    if btn_idx in [1, 2]:
                        self.send_brake()
                    elif btn_idx == 4:  # L1
                        self.max_speed = max(self.max_speed - 0.01, self.min_max_speed)
                    elif btn_idx == 5:  # R1
                        self.max_speed = min(self.max_speed + 0.01, self.max_max_speed)

            # Đọc D-Pad
            if self.joystick.get_numhats() > 0:
                hat_x, hat_y = self.joystick.get_hat(0)
                if hat_y == 1:
                    self.max_speed = min(self.max_speed + 0.02, self.max_max_speed)
                elif hat_y == -1:
                    self.max_speed = max(self.max_speed - 0.02, self.min_max_speed)

            # 2. ĐIỀU KHIỂN TIẾN / LÙI: CẦN ANALOG TRÁI (Left Stick Y - Axis 1)
            # Đẩy lên (âm) -> Tiến (+), Kéo xuống (dương) -> Lùi (-)
            raw_y = self.joystick.get_axis(1)
            raw_speed_axis = - self._apply_deadzone(raw_y)
            self.current_speed = raw_speed_axis * self.max_speed

            # 3. ĐIỀU KHIỂN BẺ LÁI: CẦN ANALOG PHẢI (Right Stick X - Axis 3)
            # Gạt trái (-) -> Lái trái (-), Gạt phải (+) -> Lái phải (+)
            raw_steer_axis = self._apply_deadzone(self.joystick.get_axis(right_stick_x_axis))
            self.current_steer = raw_steer_axis * self.max_steer_angle

            # Tự động nhả phanh khẩn cấp khi người dùng gạt cần điều khiển trở lại
            if abs(self.current_speed) > 0.02 or abs(self.current_steer) > 1.0:
                self.emergency_stopped = False

            # Gửi lệnh điều khiển qua ROS Topic
            if not self.emergency_stopped:
                self.send_speed(self.current_speed)
                self.send_steer(self.current_steer)

            # Cập nhật HUD
            self.render_hud()

            rate.sleep()

        # Khi tắt node
        print("\n[DỪNG] Ngắt kết nối - Dừng xe an toàn.")
        self.send_brake()


if __name__ == '__main__':
    try:
        node = GamepadTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

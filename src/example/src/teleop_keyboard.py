#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
NODE ĐIỀU KHIỂN BÀN PHÍM THÔNG MINH (SMART KEYBOARD TELEOP - EASY MODE)
Tối ưu hóa trải nghiệm lái mượt mà như Game đua xe:
  • CHẾ ĐỘ ARCADE (Mặc định):
      - TỰ ĐỘNG TRẢ LÁI: Nhả phím A/D hoặc Mũi tên -> Bánh xe tự động thẳng lái (0°).
      - TỰ ĐỘNG HÃM GA: Nhả phím W -> Xe từ từ giảm tốc dừng lại an toàn.
      - GIỮ PHÍM ĐỂ LÁI: Giữ đè phím W để tiến, giữ A/D để bo cua mượt mà.
  • CHẾ ĐỘ CRUISE (Phím 'M' để chuyển đổi):
      - Khóa ga và giữ nguyên góc lái (chạy rảnh tay).
  • PHÍM TẮT:
      - W / [▲] : Tiến / Tăng ga
      - S / [▼] : Giảm ga / Phanh / Lùi
      - A / [◀] : Rẽ Trái (nhả phím tự trả lái)
      - D / [▶] : Rẽ Phải (nhả phím tự trả lái)
      - Space   : Phanh khẩn cấp lập tức
      - M       : Đổi chế độ [ARCADE 🎮] <-> [CRUISE 🚗]
      - Q       : Dừng xe và Thoát
=============================================================================
"""

import sys
import select
import termios
import tty
import json
import time
import os

import rospy
from std_msgs.msg import String


class SmartKeyboardTeleop:
    def __init__(self):
        rospy.init_node('bfmc_smart_keyboard_teleop', anonymous=False)
        self.pub = rospy.Publisher('/automobile/command', String, queue_size=1)

        # Chế độ lái: 'ARCADE' (dễ lái, tự trả lái) hoặc 'CRUISE' (giữ ga)
        self.mode = 'ARCADE'

        # Giá trị điều khiển
        self.speed = 0.0              # m/s
        self.steer = 0.0              # độ

        # Cấu hình bước lái & tốc độ an toàn (vừa phải, không bị trượt bánh)
        self.target_speed = 0.22      # m/s khi nhấn giữ W
        self.max_speed = 0.35         # m/s tối đa
        self.reverse_speed = -0.18    # m/s khi lùi
        self.max_steer = 21.0         # độ bẻ lái tối đa
        self.steer_rate = 6.0         # độ bẻ lái mỗi nhịp nhấn
        self.steer_decay = 0.65       # tốc độ trả lái về tâm (càng nhỏ trả càng nhanh)

        # Lưu thời điểm bấm phím gần nhất để nhận diện nhả phím
        self.last_key_time = time.time()
        self.last_steer_time = 0.0
        self.last_gas_time = 0.0

        # Kiểm tra TTY
        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None

    def get_key(self):
        if not self.is_tty:
            time.sleep(0.05)
            return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.04)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if rlist2:
                    key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_command(self):
        cmd_speed = json.dumps({"action": "1", "speed": round(float(self.speed), 3)})
        cmd_steer = json.dumps({"action": "2", "steerAngle": round(float(self.steer), 2)})
        self.pub.publish(cmd_speed)
        self.pub.publish(cmd_steer)

    def emergency_brake(self):
        self.speed = 0.0
        self.steer = 0.0
        self.pub.publish(json.dumps({"action": "3", "steerAngle": 0.0}))

    def update_physics(self):
        """Tự động trả lái và hãm ga khi người lái nhả phím (Chế độ ARCADE)"""
        now = time.time()

        if self.mode == 'ARCADE':
            # 1. Tự động trả thẳng lái nếu nhả phím A / D quá 0.12 giây
            if now - self.last_steer_time > 0.12:
                if abs(self.steer) > 0.5:
                    self.steer *= self.steer_decay
                else:
                    self.steer = 0.0

            # 2. Tự động hãm ga trôi xe nếu nhả phím W / S quá 0.18 giây
            if now - self.last_gas_time > 0.18:
                if abs(self.speed) > 0.03:
                    self.speed *= 0.82
                else:
                    self.speed = 0.0

    def render_hud(self):
        steer_bar_len = 8
        norm_steer = int(round((self.steer / self.max_steer) * steer_bar_len))
        if norm_steer < 0:
            steer_bar = "[" + " " * (steer_bar_len + norm_steer) + "◀" * abs(norm_steer) + "|" + " " * steer_bar_len + "]"
        elif norm_steer > 0:
            steer_bar = "[" + " " * steer_bar_len + "|" + "▶" * norm_steer + " " * (steer_bar_len - norm_steer) + "]"
        else:
            steer_bar = "[" + " " * steer_bar_len + "|" + " " * steer_bar_len + "]"

        mode_badge = "🎮 ARCADE (Tự trả lái)" if self.mode == 'ARCADE' else "🚗 CRUISE (Khóa ga)"
        status = "TIẾN ⬆️ " if self.speed > 0.02 else ("LÙI ⬇️ " if self.speed < -0.02 else "DỪNG ⏹️ ")

        sys.stdout.write(f"\r  [{mode_badge}] | Vận tốc: {self.speed:+5.2f} m/s ({status}) | Lái: {self.steer:+5.1f}° {steer_bar}   ")
        sys.stdout.flush()

    def print_banner(self):
        if self.is_tty:
            os.system('clear')
        print("="*75)
        print("   🎮  BFMC SMART KEYBOARD TELEOP — BẢN LÁI DỄ (ARCADE MODE)")
        print("="*75)
        print("  • CÁC ĐIỂM CẢI TIẾN GIÚP LÁI DỄ DÀNG:")
        print("     ✅ TỰ ĐỘNG TRẢ LÁI: Nhả phím A/D hoặc mũi tên -> Xe tự trả thẳng lái!")
        print("     ✅ TỰ ĐỘNG HÃM GA:  Nhả phím W -> Xe tự từ từ dừng lại, không sợ đâm tường!")
        print("     ✅ TỐC ĐỘ AN TOÀN:  Tối ưu 0.22 m/s ôm cua mượt mà, không bị trượt bánh.")
        print("\n  • PHÍM ĐIỀU KHIỂN:")
        print("     - W / [▲] : Tiến tới (Nhấn giữ để chạy)")
        print("     - S / [▼] : Giảm tốc / Lùi")
        print("     - A / [◀] : Bẻ lái Trái (Nhả phím tự trả lái)")
        print("     - D / [▶] : Bẻ lái Phải (Nhả phím tự trả lái)")
        print("     - Space   : Phanh khẩn cấp dừng ngay")
        print("     - Phím M  : Chuyển đổi qua lại giữa ARCADE và CRUISE")
        print("     - Phím Q  : Thoát chương trình")
        print("="*75 + "\n")
        print("  🟢 ĐANG CHẠY — HÃY NHẤN VÀ GIỮ PHÍM ĐỂ LÁI:")

    def run(self):
        self.print_banner()

        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                now = time.time()

                if key in ['w', 'W', '\x1b[A']:          # TIẾN
                    self.speed = min(self.speed + 0.05, self.max_speed) if self.speed >= 0 else min(self.speed + 0.08, self.max_speed)
                    if self.speed < 0.15: self.speed = 0.18
                    self.last_gas_time = now

                elif key in ['s', 'S', '\x1b[B']:        # LÙI / PHANH
                    if self.speed > 0.05:
                        self.speed = max(0.0, self.speed - 0.08)
                    else:
                        self.speed = max(self.reverse_speed, self.speed - 0.05)
                    self.last_gas_time = now

                elif key in ['a', 'A', '\x1b[D']:        # TRÁI
                    self.steer = max(-self.max_steer, self.steer - self.steer_rate)
                    self.last_steer_time = now

                elif key in ['d', 'D', '\x1b[C']:        # PHẢI
                    self.steer = min(self.max_steer, self.steer + self.steer_rate)
                    self.last_steer_time = now

                elif key in ['m', 'M']:                  # ĐỔI CHẾ ĐỘ
                    self.mode = 'CRUISE' if self.mode == 'ARCADE' else 'ARCADE'
                    print(f"\n[CHẾ ĐỘ] Đã chuyển sang: {self.mode}")

                elif key == ' ':                         # PHANH GẤP
                    self.emergency_brake()

                elif key in ['q', 'Q', '\x03']:          # THOÁT
                    self.emergency_brake()
                    print("\n\n[DỪNG XE] Đã dừng xe an toàn và thoát chương trình.")
                    break

                self.update_physics()
                self.publish_command()
                self.render_hud()
                time.sleep(0.03)

        finally:
            if self.is_tty and self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    try:
        teleop = SmartKeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

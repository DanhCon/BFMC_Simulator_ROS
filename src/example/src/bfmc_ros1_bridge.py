#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lightweight Bidirectional ROS 1 <-> ROS 2 Bridge for BFMC Simulator
Runs inside Docker container bfmc_sim (ROS 1 Noetic)

- Subscribes to /automobile/image_raw (ROS 1) -> sends JPEG frames to ROS 2 over TCP (port 9876).
- Receives JSON steering/speed commands from ROS 2 over TCP -> publishes to /automobile/command (ROS 1).
"""

import rospy
import cv2
import socket
import struct
import threading
import json
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

HOST = '127.0.0.1'
PORT = 9876

class BfmcRos1Bridge:
    def __init__(self):
        rospy.init_node('bfmc_ros1_socket_bridge', anonymous=False)
        self.bridge = CvBridge()
        
        self.cmd_pub = rospy.Publisher('/automobile/command', String, queue_size=10)
        self.image_sub = rospy.Subscriber('/automobile/image_raw', Image, self.image_callback, queue_size=1)
        
        self.client_sock = None
        self.sock_lock = threading.Lock()
        self.running = True
        
        # Start TCP Server thread
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()
        rospy.loginfo("[BFMC ROS1 Bridge] Server started on %s:%d. Waiting for ROS 2 connection...", HOST, PORT)

    def _server_loop(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen(1)

        while self.running and not rospy.is_shutdown():
            try:
                server_sock.settimeout(1.0)
                conn, addr = server_sock.accept()
                rospy.loginfo("[BFMC ROS1 Bridge] ROS 2 Bridge connected from %s", addr)
                with self.sock_lock:
                    self.client_sock = conn
                
                # Start receiver thread for incoming commands from ROS 2
                self._receive_commands(conn)
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn("[BFMC ROS1 Bridge] Server loop error: %s", e)
                time.sleep(1.0)

    def _receive_commands(self, conn):
        """Continuously receive commands from ROS 2 and publish to /automobile/command"""
        while self.running and not rospy.is_shutdown():
            try:
                # Read 4-byte prefix length
                header = conn.recv(4)
                if not header or len(header) < 4:
                    rospy.logwarn("[BFMC ROS1 Bridge] Client disconnected.")
                    break
                cmd_len = struct.unpack("!I", header)[0]
                
                # Read payload
                cmd_data = b""
                while len(cmd_data) < cmd_len:
                    chunk = conn.recv(cmd_len - len(cmd_data))
                    if not chunk:
                        break
                    cmd_data += chunk
                
                if len(cmd_data) == cmd_len:
                    cmd_str = cmd_data.decode('utf-8')
                    # Publish directly to ROS 1 topic
                    self.cmd_pub.publish(cmd_str)
            except Exception as e:
                rospy.logwarn("[BFMC ROS1 Bridge] Error receiving command: %s", e)
                break
        
        with self.sock_lock:
            if self.client_sock == conn:
                self.client_sock = None
        conn.close()

    def image_callback(self, data):
        """Send camera frame to ROS 2 if client is connected"""
        with self.sock_lock:
            sock = self.client_sock

        if sock is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Compress to JPEG with high quality (fast and saves bandwidth)
            success, enc_img = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not success:
                return
            
            payload = enc_img.tobytes()
            # Send 4-byte length + payload
            header = struct.pack("!I", len(payload))
            sock.sendall(header + payload)
        except Exception as e:
            # Client will be cleared on receive failure or error
            pass

    def stop(self):
        self.running = False
        with self.sock_lock:
            if self.client_sock:
                self.client_sock.close()

if __name__ == '__main__':
    bridge = BfmcRos1Bridge()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()

#!/usr/bin/env python3
import math
import os
import socket
import struct
import time

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import tf

# -------------------- Existing Client Setup --------------------
CMD_HOST  = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = str(os.getenv("PROTO", "tcp"))

sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
    sock_tel.listen(1)
    print(f"[client] waiting for telemetry TCP on {TEL_HOST}:{TEL_PORT}...")
    conn, _ = sock_tel.accept()
    sock_tel = conn
    print("[client] connected to udp_diff telemetry")

def send_cmd(v: float, w: float):
    packet = struct.pack("<2f", v, w)
    sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))

def recv_all(sock, size):
    buf = b""
    while len(buf) < size:
        chunk = sock.recv(size - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def recv_tel():
    if PROTO == "udp":
        data, _ = sock_tel.recvfrom(65535)
    else:
        size_bytes = sock_tel.recv(4)
        if not size_bytes:
            return None
        size = struct.unpack("<I", size_bytes)[0]
        data = recv_all(sock_tel, size)

    if not data.startswith(b"WBTG"):
        print('WBT2 -> WBTG')
        return None

    header_size = 4 + 9 * 4
    odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz = struct.unpack("<9f", data[4:header_size])
    n = struct.unpack("<I", data[header_size:header_size + 4])[0]
    ranges = []
    if n > 0:
        ranges = struct.unpack(f"<{n}f", data[header_size + 4:header_size + 4 + 4 * n])

    return odom_x, odom_y, odom_th, (vx, vy, vth), (wx, wy, wz), ranges

# -------------------- ROS Setup --------------------
rospy.init_node('udp_diff_ros_client')

odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
# Subscriber for velocity commands
def cmd_vel_cb(msg: Twist):
    send_cmd(msg.linear.x, msg.angular.z)

rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)

# Odometry message setup
odom_msg = Odometry()
odom_msg.header.frame_id = "odom"
odom_msg.child_frame_id = "base_link"

# TF broadcaster (optional)
tf_broadcaster = tf.TransformBroadcaster()

rate = rospy.Rate(50)  # 50 Hz update loop

try:
    while not rospy.is_shutdown():
        tel = recv_tel()
        if not tel:
            continue

        x, y, th, vel, gyro, ranges = tel
        vx, vy, vth = vel

        # Publish odometry
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom_msg.pose.pose.orientation = Quaternion(*quat)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth

        odom_pub.publish(odom_msg)

        # Broadcast TF
        tf_broadcaster.sendTransform(
            (x, y, 0),
            quat,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        rate.sleep()

except rospy.ROSInterruptException:
    print("[client] stop")
    send_cmd(0.0, 0.0)



import os
import time
import math
import socket
import struct

from collections import namedtuple


Point = namedtuple("Point", ("x", "y"))

# Не "хардкодьте" адреса
CMD_HOST  = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = str(os.getenv("PROTO", "tcp"))


targets = (
    Point(-0.75, 0),
    Point(0.5, 5),
    Point(-0.05, 8),
    Point(1, 14),
    Point(4.75, 14.5),
    Point(1, 25),
    None, 
)


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


def wrap_pi(a: float):
    return (a + math.pi) % (2 * math.pi) - math.pi


def move2point(pos: Point, theta: float, target: Point | None, kp: float = 1.25):
    if not target:
        return (100, 0), False

    dx, dy = target.x - pos.x, target.y - pos.y 


    dist = math.hypot(dx, dy)   
    if dist < 0.25:
        print(f"[nav] nav done. going further")
        return (0, 0), True

    # yaw_err = wrap_pi(wrap_pi(math.atan2(dy, dx) + math.pi) - wrap_pi(theta))

    norm_theta = wrap_pi(theta)
    desired_heading = wrap_pi(math.atan2(dy, dx) + math.pi)
    yaw_err = wrap_pi(desired_heading - wrap_pi(norm_theta))

    # print(f'{yaw_err=}', end='    ')

    def clamp(x, mx):
        return math.copysign(x, mx) if abs(x) > mx else x
    w = kp * yaw_err
    # v = -max(clamp(dist * 0.5, 4), 0.5)
    v = -100

    return (v * 5, w), False

try:
    tmr = time.time()
    current = 0
    while True:
        tel = recv_tel()
        if not tel: continue

        x, y, th, vel, gyro, ranges = tel
        cmd, arrived = move2point(Point(x, y), th, targets[current])

        if arrived: 
            current += 1
            if current == 5: 
                v, w = cmd 
                cmd = v, w * 3
            if current == 6:
                cmd = 0, 0            

        send_cmd(*cmd)
        v, w = cmd

        if time.time() - tmr > 1.5:
            print(f"pos=({x:.2f}, {y:.2f}, {th:.2f}), cmd=({v=:.2f}, {w=:.2f}), target=({targets[current]})")
            tmr = time.time()

except KeyboardInterrupt:
    print("[client] stop")
    send_cmd(0.0, 0.0)


#!/usr/bin/env python3
import json
import math
import os
import select
import socket
import sys
import termios
import threading
import tty
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


UDP_LISTEN_PORT = 5005
UDP_FEEDBACK_PORT = 5006
DEFAULT_PI_IP = '192.168.1.2'

HAND_GUARD_M = 1.35     # 13.5 cm physical hand reading scaled 10x
WALL_GUARD_M = 0.45
AVOID_RADIUS_M = 0.95
CLOSE_PUSH_M = 0.55
GOAL_TOL_M = 0.20
MANUAL_SPEED = 0.45
CONE_COS_LIMIT = 0.64   # ~ +/-50 deg forward cone

# Green pads + home pad coordinates from dronesim_world.sdf
PATROL_PADS = [
    (-3.5, 2.2),   # pad 1
    (0.5, 4.2),    # pad 3
    (3.5, -2.4),   # pad 2
    (0.0, 0.0),    # home
]


class IntegratedUAVController(Node):
    def __init__(self):
        super().__init__('integrated_uav_controller')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', UDP_LISTEN_PORT))
        self.sock.setblocking(False)
        self.pi_ip = DEFAULT_PI_IP

        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub_joypi_scan = self.create_publisher(LaserScan, '/joypi_scan', 10)
        self.pub_noisy = self.create_publisher(LaserScan, '/scan_noisy', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.wp_idx = 0
        self.goal_x, self.goal_y = PATROL_PADS[0]
        self.patrol = True
        self.arrived_logged = False

        self.cmd_mode = 'AUTO'
        self.last_pi_cmd = None
        self.pi_dist_m = 10.0
        self.min_lidar_m = 10.0
        self.ranges = []
        self.angle_min = -math.pi
        self.angle_inc = (2.0 * math.pi) / 36.0
        self.bypass_dir = 0

        self.alive = True
        self.reading_coords = False

        threading.Thread(target=self.key_loop, daemon=True).start()
        self.create_timer(0.1, self.control_tick)

        self.get_logger().info('ready: [1/2/3/0] pads, [G] x y, [M] patrol, [WASD] manual, [H] hover, [Space] estop')
        self.get_logger().info(f'patrol -> ({self.goal_x:.1f}, {self.goal_y:.1f})')

    def go_to(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.patrol = False
        self.arrived_logged = False
        self.bypass_dir = 0
        self.cmd_mode = 'AUTO'
        self.get_logger().info(f'target -> ({x:.2f}, {y:.2f})')

    def key_loop(self):
        try:
            fd = os.open('/dev/tty', os.O_RDONLY)
        except OSError:
            return

        saved = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.alive:
                if not select.select([fd], [], [], 0.05)[0]:
                    continue
                ch = os.read(fd, 1).decode('utf-8', errors='ignore')
                if ch == '\x03':
                    os.kill(os.getpid(), 2)
                    break

                if ch == '\x1b':
                    if select.select([fd], [], [], 0.05)[0]:
                        seq = os.read(fd, 2).decode('utf-8', errors='ignore')
                        arrow_map = {'[A': 'W', '[B': 'S', '[D': 'A', '[C': 'D'}
                        if seq in arrow_map:
                            self.cmd_mode = arrow_map[seq]
                            self.get_logger().info(f'key -> {self.cmd_mode}')
                    continue

                k = ch.upper()
                if k in ('W', 'A', 'S', 'D', 'H'):
                    self.cmd_mode = k
                    self.get_logger().info(f'key -> {self.cmd_mode}')
                elif k == 'M':
                    self.patrol = True
                    self.arrived_logged = False
                    self.bypass_dir = 0
                    self.cmd_mode = 'AUTO'
                    self.get_logger().info(f'patrol resumed -> ({self.goal_x:.2f}, {self.goal_y:.2f})')
                elif k == '1':
                    self.go_to(-3.5, 2.2)
                elif k == '2':
                    self.go_to(3.5, -2.4)
                elif k == '3':
                    self.go_to(0.5, 4.2)
                elif k == '0':
                    self.go_to(0.0, 0.0)
                elif k == ' ':
                    self.cmd_mode = 'ESTOP'
                    self.get_logger().info('key -> ESTOP')
                elif k == 'G':
                    self.reading_coords = True
                    termios.tcsetattr(fd, termios.TCSADRAIN, saved)
                    sys.stdout.write('\nEnter target X Y (m): ')
                    sys.stdout.flush()
                    buf = ''
                    while True:
                        c = os.read(fd, 1).decode('utf-8', errors='ignore')
                        if c in ('\n', '\r'):
                            break
                        buf += c
                    try:
                        parts = buf.strip().split()
                        if len(parts) >= 2:
                            self.go_to(float(parts[0]), float(parts[1]))
                        else:
                            self.get_logger().warn('need two floats like: 3.5 -2.4')
                    except ValueError:
                        self.get_logger().warn('bad coordinate input')
                    tty.setraw(fd)
                    self.reading_coords = False
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            os.close(fd)

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def on_scan(self, msg):
        r = np.array(msg.ranges, dtype=np.float32)
        r = np.nan_to_num(r, nan=msg.range_max, posinf=msg.range_max, neginf=msg.range_min)
        self.min_lidar_m = float(np.min(r))
        self.ranges = r.tolist()
        self.angle_min = float(msg.angle_min)
        self.angle_inc = float(msg.angle_increment)

    def send_pi_feedback(self, spin):
        if self.cmd_mode == 'ESTOP':
            mode = 'STOP'
        elif self.pi_dist_m < HAND_GUARD_M:
            mode = 'HAND STOP'
        elif self.cmd_mode in ('W', 'A', 'S', 'D'):
            mode = 'MANUAL'
        elif self.cmd_mode == 'H':
            mode = 'HOVER'
        else:
            mode = 'AUTO'

        pkt = {
            'motor': 'SPIN' if spin else 'STOP',
            'mode': mode,
            'obs_dist': round(float(self.min_lidar_m), 2),
        }
        try:
            self.sock.sendto(json.dumps(pkt).encode('utf-8'), (self.pi_ip, UDP_FEEDBACK_PORT))
        except Exception:
            pass

    def poll_pi_udp(self):
        latest = None
        # Flush kernel queue on every tick; without this Gazebo frame drops cause ~2s lag
        while True:
            try:
                raw, addr = self.sock.recvfrom(1024)
                self.pi_ip = addr[0]
                latest = json.loads(raw.decode('utf-8'))
            except BlockingIOError:
                break
            except Exception:
                break

        if latest is None:
            return

        self.pi_dist_m = float(latest.get('dist_m', 10.0))
        pi_cmd = str(latest.get('cmd', 'AUTO'))

        # Edge-trigger Pi commands so 10 Hz packets don't wipe out laptop keystrokes
        if self.last_pi_cmd is None:
            self.last_pi_cmd = pi_cmd
        elif pi_cmd != self.last_pi_cmd:
            self.last_pi_cmd = pi_cmd
            self.cmd_mode = pi_cmd
            self.get_logger().info(f'joypi -> {self.cmd_mode}')

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0
        scan.range_min = 0.12
        scan.range_max = 10.0

        rays = [10.0] * 360
        for i in range(170, 190):
            rays[i] = self.pi_dist_m
        scan.ranges = rays

        self.pub_joypi_scan.publish(scan)
        self.publish_noisy(scan)

    def publish_noisy(self, scan):
        r_min, r_max = float(scan.range_min), float(scan.range_max)
        r = np.array(scan.ranges, dtype=np.float32)
        r = np.nan_to_num(r, nan=r_max, posinf=r_max, neginf=r_min)

        r_noisy = np.clip(r + np.random.normal(0.0, 0.20, size=r.shape), r_min, r_max)
        u = np.random.rand(*r.shape)
        r_noisy[u < 0.08] = 0.0
        r_noisy[(u >= 0.08) & (u < 0.15)] = r_max

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = r_noisy.astype(float).tolist()
        self.pub_noisy.publish(out)

    def control_tick(self):
        self.poll_pi_udp()
        if self.reading_coords:
            self.send_pi_feedback(False)
            return

        twist = Twist()

        # Hard E-Stop
        if self.cmd_mode == 'ESTOP':
            self.pub_cmd.publish(twist)
            self.send_pi_feedback(False)
            self.get_logger().error('ESTOP active (press M or 1/2/3/0)', throttle_duration_sec=2.0)
            return

        # Physical ultrasonic hand cutoff
        if self.pi_dist_m < HAND_GUARD_M:
            self.pub_cmd.publish(twist)
            self.send_pi_feedback(False)
            self.get_logger().warn(f'hand override ({self.pi_dist_m:.2f}m) -> hover', throttle_duration_sec=1.5)
            return

        # Manual control + wall guard
        if self.cmd_mode in ('W', 'A', 'S', 'D', 'H'):
            if self.min_lidar_m < WALL_GUARD_M and self.cmd_mode != 'H':
                self.pub_cmd.publish(twist)
                self.send_pi_feedback(False)
                self.get_logger().warn('wall brake (<0.45m)', throttle_duration_sec=1.5)
                return

            if self.cmd_mode == 'W':
                twist.linear.x = MANUAL_SPEED
            elif self.cmd_mode == 'S':
                twist.linear.x = -MANUAL_SPEED
            elif self.cmd_mode == 'A':
                twist.linear.y = MANUAL_SPEED
            elif self.cmd_mode == 'D':
                twist.linear.y = -MANUAL_SPEED

            self.pub_cmd.publish(twist)
            self.send_pi_feedback(self.cmd_mode != 'H')
            return

        # Autonomous waypoint flight
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        if dist < GOAL_TOL_M:
            self.bypass_dir = 0
            if self.patrol:
                self.get_logger().info(f'reached ({self.goal_x:.1f}, {self.goal_y:.1f})')
                self.wp_idx = (self.wp_idx + 1) % len(PATROL_PADS)
                self.goal_x, self.goal_y = PATROL_PADS[self.wp_idx]
                dx = self.goal_x - self.x
                dy = self.goal_y - self.y
                dist = math.hypot(dx, dy)
            else:
                if not self.arrived_logged:
                    self.get_logger().info(f'arrived at ({self.x:.2f}, {self.y:.2f}) -> hovering')
                    self.arrived_logged = True
                self.pub_cmd.publish(twist)
                self.send_pi_feedback(False)
                return

        if dist < 1e-6:
            self.pub_cmd.publish(twist)
            self.send_pi_feedback(False)
            return

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        gx = (dx * cy + dy * sy) / dist
        gy = (-dx * sy + dy * cy) / dist

        front_min = 10.0
        obs_ang = 0.0
        for i, r_val in enumerate(self.ranges):
            ang = self.angle_min + i * self.angle_inc
            if (math.cos(ang) * gx + math.sin(ang) * gy) > CONE_COS_LIMIT and r_val < front_min:
                front_min = r_val
                obs_ang = ang

        speed = min(0.45, max(0.25, dist * 0.6))

        if front_min < AVOID_RADIUS_M and dist > 0.30:
            ax, ay = -math.cos(obs_ang), -math.sin(obs_ang)
            t1x, t1y = -math.sin(obs_ang), math.cos(obs_ang)
            t2x, t2y = math.sin(obs_ang), -math.cos(obs_ang)

            # Latch tangent side until clear to avoid left/right chattering
            if self.bypass_dir == 0:
                self.bypass_dir = 1 if (t1x * gx + t1y * gy) >= (t2x * gx + t2y * gy) else -1

            tx, ty = (t1x, t1y) if self.bypass_dir == 1 else (t2x, t2y)

            if front_min < CLOSE_PUSH_M:
                vx, vy = 0.60 * tx + 0.40 * ax, 0.60 * ty + 0.40 * ay
            else:
                vx, vy = 0.80 * tx + 0.20 * gx, 0.80 * ty + 0.20 * gy

            mag = math.hypot(vx, vy) or 1.0
            twist.linear.x = float((vx / mag) * speed)
            twist.linear.y = float((vy / mag) * speed)
            self.get_logger().info(f'bypass ({front_min:.2f}m) -> ({self.goal_x:.1f}, {self.goal_y:.1f})', throttle_duration_sec=1.0)
        else:
            self.bypass_dir = 0
            twist.linear.x = float(gx * speed)
            twist.linear.y = float(gy * speed)

        self.pub_cmd.publish(twist)
        self.send_pi_feedback(True)


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedUAVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.alive = False
        node.send_pi_feedback(False)
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

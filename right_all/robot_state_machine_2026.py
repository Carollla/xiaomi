#!/usr/bin/env python3
"""
Autonomous task state machine for the 2026 Xiaomi Cup track.

The public 2026 problem statement defines six segments:
flagstones, orange-ball search, S/curve road, tunnel search, narrow bridge,
and final soccer/finish.  This state machine maps those tasks to the existing
LCM motion wrapper and ROS2 sensor providers in right_all/.
"""
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String
import time

from detect.race2026_detect import Detection, Race2026Detector
from detect.track_detector import TrackDetector
from detect.gazebo_pose_provider import GazeboPoseProvider


@dataclass(frozen=True)
class Timings:
    flagstone_sec: float = 14.0
    ball_search_max_sec: float = 35.0
    curve_sec: float = 42.0
    tunnel_max_sec: float = 55.0
    bridge_sec: float = 28.0
    final_sec: float = 18.0


class RobotStateMachine2026(Node):
    def __init__(self, imu_provider, d435_provider, rgb_provider, timings: Optional[Timings] = None):
        super().__init__('robot_state_machine_2026')
        self.create_subscription(Bool, '/robot/ready', self.robot_ready_callback, 10)
        self.motion_cmd_publisher = self.create_publisher(Int32MultiArray, '/robot/motion_cmd', 10)
        self.completion_publisher = self.create_publisher(Bool, '/robot/completion', 10)
        self.speech_publisher = self.create_publisher(String, '/robot/voice_text', 10)

        self.imu_provider = imu_provider
        self.d435_provider = d435_provider
        self.rgb_provider = rgb_provider
        self.detector = Race2026Detector()
        self.track_detector = TrackDetector()
        self.pose_provider = GazeboPoseProvider(min_interval=2.0)
        self.timings = timings or Timings()

        self.state = 'WAIT_READY'
        self.robot_ready = False
        self.state_started_at = time.time()
        self.last_cmd: Optional[Tuple[int, ...]] = None
        self.completed_targets: Dict[str, bool] = {'coke': False, 'orange_ball': False, 'soccer': False}
        self.announced_tunnel_targets: Dict[str, bool] = {'coke': False, 'orange_ball': False, 'soccer': False}
        self.seg2_orange_hit = False
        self.seg2_orange_announced = False
        self.seg2_last_seen_at = 0.0
        self.seg2_last_cx = 0.5
        self.orange_hits = 0
        self.pending_tunnel_target = None
        self.height_bar_done = False
        self.block_avoid_done = False
        self.tunnel_lane = 0
        self.tunnel_lanes = ('left', 'middle', 'right')
        self.last_log_at = 0.0
        self.last_pose_log_at = 0.0
        self.last_progress_pose = None
        self.last_progress_at = time.time()
        self.final_progress_y = None
        self.final_progress_at = time.time()
        self.final_boost_at = None
        self.flagstone_stuck_recover_at = None
        self.flagstone_progress_x = None
        self.flagstone_progress_at = time.time()
        self.bridge_descent_boost_at = None
        self.pending_tunnel_target = None

        self.timer = self.create_timer(0.10, self.tick)
        self.get_logger().info('2026荒野寻宝状态机已启动')

    def robot_ready_callback(self, msg):
        self.robot_ready = bool(msg.data)
        if self.robot_ready and self.state == 'WAIT_READY':
            self.transition('SEG1_FLAGSTONE')

    def transition(self, state):
        self.get_logger().info(f'状态切换: {self.state} -> {state}')
        self.state = state
        self.state_started_at = time.time()
        self.last_cmd = None
        self.last_progress_pose = None
        self.last_progress_at = self.state_started_at
        self.final_progress_y = None
        self.final_progress_at = self.state_started_at
        self.final_boost_at = None
        self.flagstone_stuck_recover_at = None
        self.flagstone_progress_x = None
        self.flagstone_progress_at = self.state_started_at
        self.bridge_descent_boost_at = None

    def elapsed(self):
        return time.time() - self.state_started_at

    def publish_cmd(self, *values):
        command = tuple(int(v) for v in values)
        if command == self.last_cmd:
            return
        msg = Int32MultiArray()
        msg.data = list(command)
        self.motion_cmd_publisher.publish(msg)
        self.last_cmd = command

    def say(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)

    def finish(self):
        self.publish_cmd(0)
        msg = Bool()
        msg.data = True
        self.completion_publisher.publish(msg)
        self.transition('FINISH')

    def current_images(self):
        return self.rgb_provider.get_latest_image(), self.d435_provider.get_latest_depth()

    def current_pose(self):
        return self.pose_provider.get_pose()

    def imu_yaw_rad(self):
        return math.radians(self.imu_provider.get_yaw())

    def imu_roll_pitch_rad(self):
        imu = getattr(self.imu_provider, 'latest_imu_data', None)
        if imu is None:
            return 0.0, 0.0
        quat = imu.orientation
        roll, pitch, _ = self.imu_provider._quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
        return roll, pitch

    def centered_target_cmd(self, detection: Detection, forward=120, max_yaw=260):
        error = detection.cx - 0.5
        if abs(error) > 0.18:
            return (12, 70, 0, int(-max_yaw if error < 0 else max_yaw), 0)
        if abs(error) > 0.07:
            return (12, forward, 0, int(-max_yaw * 0.45 if error < 0 else max_yaw * 0.45), 0)
        return (12, forward, 0, 0, 0)

    def follow_yellow_or_cruise(self, image, speed=120):
        if image is None:
            self.publish_cmd(12, 95, 0, 0, 0)
            return
        border = self.detector.detect_yellow_border(image)
        if not border.found or border.confidence < 0.45:
            self.publish_cmd(12, 90, 0, 0, 0)
            return
        error = border.cx - 0.50
        yaw = int(max(-180, min(180, -error * 300)))
        self.publish_cmd(12, speed, 0, yaw, 0)

    @staticmethod
    def angle_error(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def turn_to_yaw(self, pose, target_yaw, max_rate=420, tolerance=0.12):
        if pose is None:
            return self.turn_to_imu_yaw(target_yaw, max_rate=max_rate, tolerance=tolerance)
        error = self.angle_error(target_yaw, pose.yaw)
        if abs(error) <= tolerance:
            self.publish_cmd(0)
            return True
        yaw = int(max(-max_rate, min(max_rate, abs(error) * 700)))
        if yaw < 180:
            yaw = 180
        if error > 0:
            self.publish_cmd(4, yaw)
        else:
            self.publish_cmd(5, -yaw)
        self.maybe_log(
            f'对准赛道: pose=({pose.x:.2f},{pose.y:.2f}) yaw={pose.yaw:.2f} target={target_yaw:.2f} err={error:.2f}',
            interval=1.0,
        )
        return False

    def turn_to_imu_yaw(self, target_yaw, max_rate=420, tolerance=0.12):
        current = self.imu_yaw_rad()
        error = self.angle_error(target_yaw, current)
        if abs(error) <= tolerance:
            self.publish_cmd(0)
            return True
        yaw = int(max(-max_rate, min(max_rate, abs(error) * 700)))
        if yaw < 180:
            yaw = 180
        if error > 0:
            self.publish_cmd(4, yaw)
        else:
            self.publish_cmd(5, -yaw)
        self.maybe_log(f'imu turn yaw={current:.2f} target={target_yaw:.2f} err={error:.2f}', interval=1.0)
        return False

    def high_step_turn_to_yaw(self, pose, target_yaw, max_rate=260, tolerance=0.14):
        if pose is None:
            current = self.imu_yaw_rad()
            x = 0.0
            y = 0.0
        else:
            current = pose.yaw
            x = pose.x
            y = pose.y
        error = self.angle_error(target_yaw, current)
        if abs(error) <= tolerance:
            self.publish_cmd(0)
            return True
        yaw = int(max(190, min(max_rate, abs(error) * 320)))
        cmd = yaw if error > 0 else -yaw
        self.publish_cmd(20, 18, 0, 285, -95, cmd, 180)
        self.maybe_log(
            f'high_step_turn pose=({x:.2f},{y:.2f}) yaw={current:.2f} target={target_yaw:.2f} '
            f'err={error:.2f} cmd={cmd}',
            interval=0.8,
        )
        return False

    def drive_to_waypoint(self, pose, waypoint, speed=115, arrive=0.35, max_yaw=360):
        if pose is None:
            self.publish_cmd(12, 70, 0, 0, 0)
            return False
        dx = waypoint[0] - pose.x
        dy = waypoint[1] - pose.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.angle_error(target_yaw, pose.yaw)
        yaw = int(max(-max_yaw, min(max_yaw, yaw_error * 650)))
        forward = speed if abs(yaw_error) < 0.65 else max(45, int(speed * 0.45))
        self.publish_cmd(12, forward, 0, yaw, 0)
        self.maybe_log(
            f'导航到点: target=({waypoint[0]:.2f},{waypoint[1]:.2f}) pose=({pose.x:.2f},{pose.y:.2f}) '
            f'd={dist:.2f} yaw_err={yaw_error:.2f}',
            interval=1.0,
        )
        return dist <= arrive

    def drive_lane_y(self, pose, target_y, lane_x=0.45, speed=100, arrive=0.25, max_yaw=260):
        if pose is None:
            yaw_error = self.angle_error(1.57, self.imu_yaw_rad())
            yaw = int(max(-max_yaw, min(max_yaw, yaw_error * 360)))
            forward = speed if abs(yaw_error) < 0.45 else max(45, int(speed * 0.55))
            self.publish_cmd(12, forward, 0, yaw, 0)
            self.maybe_log(f'lane_y imu target_y={target_y:.2f} yaw_err={yaw_error:.2f}', interval=1.0)
            return False
        lateral_error = lane_x - pose.x
        yaw_error = self.angle_error(1.57, pose.yaw)
        yaw = int(max(-max_yaw, min(max_yaw, yaw_error * 360 + lateral_error * 80)))
        strafe = int(max(-120, min(120, -lateral_error * 260)))
        forward = speed if abs(yaw_error) < 0.45 else max(45, int(speed * 0.55))
        if pose.z < 0.18:
            yaw = int(max(-150, min(150, yaw_error * 260)))
            strafe = 0
            forward = min(82, max(48, forward))
        self.publish_cmd(12, forward, strafe, yaw, 0)
        self.maybe_log(
            f'lane_y target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) '
            f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} strafe={strafe}',
            interval=1.0,
        )
        return pose.y >= target_y - arrive

    def track_boundary_hint(self, image):
        result = self.track_detector.detect(image) if image is not None else None
        if result is None or not result.found or result.confidence < 0.45:
            return 0, 0.0, 0.0, 0.0
        yaw_hint = int(max(-45, min(45, -result.error * 45.0 - result.heading_deg * 0.7)))
        return yaw_hint, result.error, result.heading_deg, result.confidence

    def yellow_boundary_layout(self, image):
        if image is None:
            return False, False, 0.0, 0.0
        mask = self.track_detector.get_yellow_mask(image)
        h, w = mask.shape
        roi = mask[int(h * 0.45):, :]
        left = roi[:, : int(w * 0.42)]
        right = roi[:, int(w * 0.58):]
        left_ratio = float(np.count_nonzero(left)) / max(1, left.size)
        right_ratio = float(np.count_nonzero(right)) / max(1, right.size)
        return left_ratio > 0.006, right_ratio > 0.006, left_ratio, right_ratio

    def ready_to_turn_after_flagstone(self, pose, image):
        if pose is None:
            self.publish_cmd(0)
            self.maybe_log('flagstone exit waiting: reliable pose unavailable', interval=1.0)
            return False
        left_seen, right_seen, left_ratio, right_ratio = self.yellow_boundary_layout(image)
        off_flagstones = pose.x > 2.60 and pose.z > 0.18 and abs(pose.roll) < 0.42 and abs(pose.pitch) < 0.42
        inside_lane = -0.22 <= pose.y <= 0.22
        yaw_straight = abs(self.angle_error(0.0, pose.yaw)) < 0.28
        boundary_ok = (left_seen and right_seen) or abs(pose.y) < 0.18
        self.maybe_log(
            f'flagstone exit check pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
            f'off={off_flagstones} lane={inside_lane} yaw={yaw_straight} yellow=({left_seen},{right_seen}) '
            f'ratio=({left_ratio:.3f},{right_ratio:.3f})',
            interval=1.0,
        )
        return off_flagstones and inside_lane and yaw_straight and boundary_ok

    def drive_flagstone_x(self, pose, image=None, depth=None, target_x=2.25, lane_y=0.0):
        phase = self.elapsed()
        if pose is not None and pose.z < 0.12:
            self.publish_cmd(0)
            self.maybe_log(
                f'flagstone hold low_pose pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f}',
                interval=0.8,
            )
            return False

        if pose is None:
            yaw_error = self.angle_error(0.0, self.imu_yaw_rad())
            yaw = int(max(-80, min(80, yaw_error * 220)))
            if self.flagstone_stuck_recover_at is not None:
                self.publish_cmd(20, 64, 0, 312, -58, yaw, 235)
            else:
                self.publish_cmd(20, 52, 0, 292, -66, yaw, 190)
            self.maybe_log(f'flagstone imu target_x={target_x:.2f} yaw_err={yaw_error:.2f}', interval=0.8)
            return False

        target_yaw = 0.0
        if abs(pose.y - lane_y) > 0.16:
            target_yaw = max(-0.14, min(0.14, (lane_y - pose.y) * 0.35))
        yaw_error = self.angle_error(target_yaw, pose.yaw)
        yaw = int(max(-140, min(140, yaw_error * 240)))
        lateral_error = lane_y - pose.y
        strafe = int(max(-10, min(10, lateral_error * 55)))
        track_yaw, track_error, track_heading, track_conf = self.track_boundary_hint(image)
        if abs(pose.y - lane_y) > 0.24 and track_conf > 0.45:
            yaw = int(max(-95, min(95, yaw + int(track_yaw * 0.45))))

        # Official 2026 stones are 30 cm long, 5 cm high, with 20 cm gaps.
        # The Gazebo mesh places their leading edges near these x positions.
        stone_edges = (0.60, 1.10, 1.60, 2.10)
        nearest_edge = min(stone_edges, key=lambda edge: abs(pose.x - edge))
        edge_window = abs(pose.x - nearest_edge) < 0.115
        rear_clear_zone = any(edge - 0.05 <= pose.x <= edge + 0.20 for edge in stone_edges)
        on_stone = any(start + 0.05 <= pose.x <= start + 0.30 for start in stone_edges)
        in_gap = any(start + 0.30 < pose.x < next_start for start, next_start in zip(stone_edges, stone_edges[1:]))

        if pose.x < 0.56 and abs(yaw_error) > 0.42:
            yaw_turn = int(max(180, min(360, abs(yaw_error) * 540)))
            if yaw_error > 0:
                self.publish_cmd(4, yaw_turn)
                cmd = yaw_turn
            else:
                self.publish_cmd(5, -yaw_turn)
                cmd = -yaw_turn
            self.maybe_log(
                f'flagstone pre-align pose=({pose.x:.2f},{pose.y:.2f}) yaw_err={yaw_error:.2f} '
                f'cmd={cmd} before_edge={nearest_edge:.2f}',
                interval=0.8,
            )
            return False

        if phase < 4.2 or pose.x < 0.48:
            height = 300
            pitch = -58
            step_height = 190
            speed = 46 if abs(yaw_error) < 0.25 else 34
        elif edge_window:
            height = 310
            pitch = -60
            step_height = 230
            speed = 60 if abs(yaw_error) < 0.30 else 44
            yaw = int(max(-55, min(55, yaw)))
            strafe = 0
        elif on_stone:
            height = 295
            pitch = -66
            step_height = 190
            speed = 62 if abs(yaw_error) < 0.26 else 44
            strafe = int(max(-6, min(6, strafe)))
        elif in_gap:
            height = 298
            pitch = -68
            step_height = 200
            speed = 54 if abs(yaw_error) < 0.28 else 40
            strafe = int(max(-6, min(6, strafe)))
        else:
            height = 282
            pitch = -58
            step_height = 150
            speed = 62 if abs(yaw_error) < 0.24 else 46
        obstacle_near, obstacle_ratio, obstacle_median = self.detector.obstacle_depth(depth, near_threshold=0.70)
        if obstacle_near and pose.x < 2.35:
            height = max(height, 292)
            step_height = max(step_height, 185)
            speed = min(speed, 38)
        now = time.time()
        forward_stuck = self.is_forward_stuck(pose, min_dx=0.035, interval=2.8)
        if forward_stuck:
            if self.flagstone_stuck_recover_at is None:
                self.flagstone_stuck_recover_at = now
            recover_t = now - self.flagstone_stuck_recover_at
            if recover_t < 4.6:
                height = max(height, 314)
                pitch = -54
                step_height = max(step_height, 242)
                speed = max(speed, 66)
                yaw = int(max(-38, min(38, yaw)))
                strafe = 0
                self.maybe_log(
                    f'flagstone rear-leg clear pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'edge={nearest_edge:.2f} recover={recover_t:.1f}',
                    interval=0.6,
                )
            else:
                self.flagstone_stuck_recover_at = None
        else:
            self.flagstone_stuck_recover_at = None
        if rear_clear_zone and pose.x < 2.35:
            height = max(height, 306)
            step_height = max(step_height, 220)
            speed = max(speed, 58 if abs(yaw_error) < 0.30 else 42)
            yaw = int(max(-60, min(60, yaw)))
            strafe = int(max(-3, min(3, strafe)))
        if pose.z < 0.18:
            speed = min(speed, 32)
            height = max(height, 292)
            step_height = max(step_height, 185)
        if abs(yaw_error) > 0.34 and pose.x < 2.45:
            yaw = int(max(-280, min(280, yaw_error * 360)))
            speed = min(speed, 28)
            height = max(height, 300)
            pitch = min(pitch, -70)
            step_height = max(step_height, 205)
            strafe = int(max(-4, min(4, strafe)))
        if abs(yaw_error) > 0.46 and pose.x < 2.45:
            yaw = int(max(-320, min(320, yaw_error * 390)))
            speed = min(speed, 20)
            strafe = 0
        if rear_clear_zone and pose.x < 2.35:
            yaw = int(max(-60, min(60, yaw)))
            strafe = int(max(-3, min(3, strafe)))
            speed = max(speed, 58 if abs(yaw_error) < 0.30 else 42)
        self.publish_cmd(20, speed, strafe, height, pitch, yaw, step_height)
        self.maybe_log(
            f'flagstone target_x={target_x:.2f} pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
            f'target_yaw={target_yaw:.2f} yaw_err={yaw_error:.2f} yaw={yaw} '
            f'speed={speed} strafe={strafe} height={height} pitch={pitch} step={step_height} '
            f'edge={nearest_edge:.2f}/{edge_window} rear_clear={rear_clear_zone} on_stone={on_stone} gap={in_gap} '
            f'track_err={track_error:.2f} track_head={track_heading:.1f} track_conf={track_conf:.2f} '
            f'obstacle={obstacle_near}/{obstacle_ratio:.2f}/{obstacle_median:.2f} stuck={forward_stuck}',
            interval=0.8,
        )
        return pose.x >= target_x

    def drive_final_y(self, pose, target_y=15.60):
        if pose is None:
            self.publish_cmd(0)
            self.maybe_log(f'final_lane waiting for reliable pose target_y={target_y:.2f}', interval=1.0)
            return False

        # Stay in the right-side exit lane reached after the bridge/slope.
        # A hard pull to the original center lane tips the robot on this section.
        if pose.y < 13.55:
            lane_x = 3.25
        elif pose.y < 14.80:
            lane_x = 3.35
        else:
            lane_x = 2.65
        lateral_error = lane_x - pose.x
        if pose.y >= 14.80:
            target_yaw = 1.57
        else:
            target_yaw = 1.57 - max(-0.22, min(0.22, lateral_error * 0.18))
        yaw_error = self.angle_error(target_yaw, pose.yaw)
        now = time.time()
        if self.final_progress_y is None or pose.y > self.final_progress_y + 0.05:
            self.final_progress_y = pose.y
            self.final_progress_at = now
        final_stalled = now - self.final_progress_at > 7.0
        if pose.y < 13.45:
            target_yaw = 1.57
            yaw_error = self.angle_error(target_yaw, pose.yaw)
            roll, imu_pitch = self.imu_roll_pitch_rad()
            yaw = int(max(-120, min(120, yaw_error * 220)))
            unstable_descend = (
                pose.z < 0.32
                or abs(pose.roll) > 0.32
                or abs(pose.pitch) > 0.32
                or abs(roll) > 0.32
                or abs(imu_pitch) > 0.32
            )
            if unstable_descend:
                yaw = int(max(-70, min(70, yaw_error * 160)))
                strafe = int(max(-28, min(28, -(lane_x - pose.x) * 45)))
                self.publish_cmd(20, 40, strafe, 270, -65, yaw, 165)
                mode = 'final_pitch_recover'
                self.maybe_log(
                    f'final_lane mode={mode} target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} x_err={lateral_error:.2f} '
                    f'pitch={pose.pitch:.2f}/{imu_pitch:.2f} stalled={final_stalled}',
                    interval=1.0,
                )
                return pose.y >= target_y - 0.05
            if pose.z > 0.34:
                forward = 54
                mode = 'final_descend_drive'
            elif pose.z < 0.13:
                phase = self.elapsed() % 4.2
                if phase < 1.7:
                    self.publish_cmd(0)
                    mode = 'final_stand_reset'
                else:
                    self.publish_cmd(20, 58 if final_stalled else 42, 0, 260, -45, yaw, 135)
                    mode = 'final_low_crawl'
                self.maybe_log(
                    f'final_lane mode={mode} target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} x_err={lateral_error:.2f} '
                    f'stalled={final_stalled}',
                    interval=1.0,
                )
                return pose.y >= target_y - 0.05
            elif abs(yaw_error) > 0.35:
                forward = 28
                mode = 'final_align_drive'
            else:
                forward = 88 if final_stalled else 68
                if final_stalled and pose.z > 0.20 and abs(yaw_error) < 0.45:
                    if self.final_boost_at is None:
                        self.final_boost_at = now
                    boost_t = now - self.final_boost_at
                    if boost_t < 0.25:
                        self.publish_cmd(0)
                    elif boost_t < 0.95:
                        self.publish_cmd(17)
                    elif boost_t < 1.55:
                        self.publish_cmd(0)
                    else:
                        self.final_boost_at = None
                    mode = 'final_jump_boost'
                else:
                    self.publish_cmd(12, forward, 0, yaw, 0)
                    mode = 'final_drive_out'
                self.maybe_log(
                    f'final_lane mode={mode} target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} x_err={lateral_error:.2f} '
                    f'stalled={final_stalled}',
                    interval=1.0,
                )
                return pose.y >= target_y - 0.05
            self.publish_cmd(12, forward, 0, yaw, 0)
            self.maybe_log(
                f'final_lane mode={mode} target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} x_err={lateral_error:.2f} '
                f'stalled={final_stalled}',
                interval=1.0,
            )
            return pose.y >= target_y - 0.05
        if abs(yaw_error) > 0.36 and pose.z > 0.16:
            yaw_turn = int(max(160, min(300, abs(yaw_error) * 520)))
            if yaw_error > 0:
                self.publish_cmd(4, yaw_turn)
                cmd_yaw = yaw_turn
            else:
                self.publish_cmd(5, -yaw_turn)
                cmd_yaw = -yaw_turn
            self.maybe_log(
                f'final_lane mode=final_realign target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) '
                f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} cmd={cmd_yaw}',
                interval=1.0,
            )
            return False
        yaw = int(max(-180, min(180, yaw_error * 360 + lateral_error * 25)))
        if abs(yaw_error) > 0.26:
            forward = 36
        else:
            if pose.y < 13.65:
                forward = 58
            else:
                forward = 76 if pose.y < 14.80 else 92
        strafe_limit = 35 if pose.y < 14.80 else 18
        strafe = int(max(-strafe_limit, min(strafe_limit, -lateral_error * 75)))
        self.publish_cmd(12, forward, strafe, yaw, 0)
        self.maybe_log(
            f'final_lane target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) '
            f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} x_err={lateral_error:.2f} '
            f'forward={forward} strafe={strafe} yaw={yaw}',
            interval=1.0,
        )
        return pose.y >= target_y - 0.05

    def high_step_forward_y(self, pose, target_y, speed=90, height=230, pitch=-60, max_turn=260):
        if pose is None:
            yaw_error = self.angle_error(1.57, self.imu_yaw_rad())
            if abs(yaw_error) > 0.22:
                yaw = int(max(180, min(max_turn, abs(yaw_error) * 650)))
                if yaw_error > 0:
                    self.publish_cmd(4, yaw)
                else:
                    self.publish_cmd(5, -yaw)
            else:
                self.publish_cmd(10, speed, height, pitch)
            self.maybe_log(f'high_step imu target_y={target_y:.2f} yaw_err={yaw_error:.2f}', interval=1.0)
            return False
        yaw_error = self.angle_error(1.57, pose.yaw)
        if abs(yaw_error) > 0.18:
            yaw = int(max(180, min(max_turn, abs(yaw_error) * 650)))
            if yaw_error > 0:
                self.publish_cmd(4, yaw)
            else:
                self.publish_cmd(5, -yaw)
        else:
            self.publish_cmd(10, speed, height, pitch)
        self.maybe_log(
            f'high_step target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) yaw_err={yaw_error:.2f}',
            interval=1.0,
        )
        return pose.y >= target_y

    def drive_bridge_descent(self, pose, target_y=13.55):
        if pose is None:
            yaw_error = self.angle_error(1.57, self.imu_yaw_rad())
            yaw = int(max(-90, min(90, yaw_error * 180)))
            self.publish_cmd(20, 42, 0, 280, -65, yaw, 170)
            self.maybe_log(f'bridge_descent imu target_y={target_y:.2f} yaw_err={yaw_error:.2f}', interval=1.0)
            return False

        lane_x = 3.05
        lateral_error = lane_x - pose.x
        target_yaw = 1.57 - max(-0.22, min(0.22, lateral_error * 0.35))
        yaw_error = self.angle_error(target_yaw, pose.yaw)
        yaw = int(max(-120, min(120, yaw_error * 220)))
        strafe = int(max(-26, min(26, -lateral_error * 65)))
        stuck = self.is_stuck(pose, min_move=0.055, interval=5.5)

        if pose.z > 0.34 and abs(yaw_error) < 0.32:
            self.publish_cmd(14, 62, 215)
            mode = 'bridge_top_walk'
        elif pose.z > 0.24:
            forward = 58 if abs(yaw_error) < 0.48 else 42
            self.publish_cmd(20, forward, strafe, 278, -88, yaw, 170)
            mode = 'bridge_slope_crawl'
        else:
            if stuck:
                forward = 72
                height = 300
                pitch = -55
                step = 195
                self.bridge_descent_boost_at = time.time()
            else:
                forward = 46 if abs(yaw_error) > 0.42 else 56
                height = 288
                pitch = -66
                step = 178
            self.publish_cmd(20, forward, strafe, height, pitch, yaw, step)
            mode = 'bridge_lip_rear_leg_crawl'

        self.maybe_log(
            f'bridge_descent mode={mode} target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) '
            f'z={pose.z:.3f} yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} '
            f'x_err={lateral_error:.2f} strafe={strafe} stuck={stuck}',
            interval=1.0,
        )
        return pose.y >= target_y

    def is_stuck(self, pose, min_move=0.12, interval=4.0):
        if pose is None:
            return False
        now = time.time()
        if self.last_progress_pose is None:
            self.last_progress_pose = (pose.x, pose.y)
            self.last_progress_at = now
            return False
        moved = math.hypot(pose.x - self.last_progress_pose[0], pose.y - self.last_progress_pose[1])
        if moved >= min_move:
            self.last_progress_pose = (pose.x, pose.y)
            self.last_progress_at = now
            return False
        return now - self.last_progress_at >= interval

    def is_forward_stuck(self, pose, min_dx=0.04, interval=3.0):
        if pose is None:
            return False
        now = time.time()
        if self.flagstone_progress_x is None or pose.x < self.flagstone_progress_x - 0.04:
            self.flagstone_progress_x = pose.x
            self.flagstone_progress_at = now
            return False
        if pose.x - self.flagstone_progress_x >= min_dx:
            self.flagstone_progress_x = pose.x
            self.flagstone_progress_at = now
            return False
        return now - self.flagstone_progress_at >= interval

    def maybe_log(self, text, interval=2.0):
        now = time.time()
        if now - self.last_log_at >= interval:
            self.get_logger().info(text)
            self.last_log_at = now

    def maybe_log_pose(self, pose, interval=2.0):
        now = time.time()
        if now - self.last_pose_log_at < interval:
            return
        if pose is None:
            self.get_logger().info(
                f'imu state={self.state} t={self.elapsed():.1f}s yaw={self.imu_yaw_rad():.3f}'
            )
            self.last_pose_log_at = now
            return
        self.get_logger().info(
            f'位姿 state={self.state} t={self.elapsed():.1f}s '
            f'x={pose.x:.3f} y={pose.y:.3f} z={pose.z:.3f} yaw={pose.yaw:.3f}'
        )
        self.last_pose_log_at = now

    def robot_is_down(self, pose):
        if pose is None:
            return False
        roll, pitch = self.imu_roll_pitch_rad()
        if self.state in ('SEG5_JUMP_DOWN', 'SEG6_SOCCER', 'SEG6_FINISH_CIRCLE'):
            if pose.z > 0.13 and abs(roll) < 1.15 and abs(pose.roll) < 1.15:
                return False
        if abs(roll) > 0.95 or abs(pitch) > 0.85:
            return True
        return pose.z < 0.080 or abs(pose.roll) > 0.95 or abs(pose.pitch) > 0.85

    def tick(self):
        if self.state == 'FINISH':
            return
        if not self.robot_ready and self.state == 'WAIT_READY':
            self.publish_cmd(0)
            return

        image, depth = self.current_images()
        pose = self.current_pose()
        self.maybe_log_pose(pose)

        if self.robot_is_down(pose):
            self.maybe_log(
                f'检测到疑似跌倒/低姿态: z={pose.z:.3f} roll={pose.roll:.2f} pitch={pose.pitch:.2f}，停止发前进命令',
                interval=1.0,
            )
            self.publish_cmd(0)
            return

        if self.state == 'SEG1_FLAGSTONE':
            if self.elapsed() < 3.0:
                self.publish_cmd(0)
            else:
                self.drive_flagstone_x(pose, image=image, depth=depth, target_x=2.75, lane_y=0.0)
            if self.ready_to_turn_after_flagstone(pose, image):
                self.transition('SEG1_TURN_TO_BALLS')

        elif self.state == 'SEG1_TURN_TO_BALLS':
            if self.elapsed() < 2.0:
                self.publish_cmd(0)
                return

            if pose is None:
                self.publish_cmd(0)
                self.maybe_log('flagstone turn waiting: reliable pose unavailable', interval=0.8)
                return

            stable_for_turn = pose.z > 0.17 and abs(pose.roll) < 0.55 and abs(pose.pitch) < 0.55
            at_turn_gate = pose.x >= 2.58
            in_safe_lane = -0.36 <= pose.y <= 0.36
            if not stable_for_turn:
                self.publish_cmd(0)
                self.maybe_log(
                    f'flagstone turn hold: unstable pose=({pose.x:.2f},{pose.y:.2f}) '
                    f'z={pose.z:.3f} roll={pose.roll:.2f} pitch={pose.pitch:.2f}',
                    interval=0.8,
                )
                return
            if not at_turn_gate or not in_safe_lane:
                self.drive_flagstone_x(pose, image=image, depth=depth, target_x=2.75, lane_y=0.0)
                self.maybe_log(
                    f'flagstone turn approach: gate={at_turn_gate} lane={in_safe_lane} '
                    f'pose=({pose.x:.2f},{pose.y:.2f}) yaw={pose.yaw:.2f}',
                    interval=0.8,
                )
                return

            aligned = self.high_step_turn_to_yaw(pose, 1.60, max_rate=340, tolerance=0.14)
            if aligned or (pose is not None and pose.yaw > 1.35):
                self.transition('SEG2_ENTER_BALLS')

        elif self.state == 'SEG2_ENTER_BALLS':
            self.drive_lane_y(pose, 2.35, lane_x=0.0, speed=72, arrive=0.25, max_yaw=180)
            if pose is not None and pose.y > 2.15 and abs(pose.x) < 0.28:
                self.transition('SEG2_ORANGE_SEARCH')

        elif self.state == 'SEG2_ORANGE_SEARCH':
            target = self.detector.attach_depth(self.detector.detect_orange_ball(image), depth)
            if target.found and target.confidence > 0.55:
                self.maybe_log(
                    f'橙球候选 cx={target.cx:.2f} cy={target.cy:.2f} area={target.area_ratio:.4f} '
                    f'conf={target.confidence:.2f} dist={target.distance}'
                )
                if not self.seg2_orange_announced:
                    self.seg2_orange_announced = True
                    self.say('orange ball detected')
                self.seg2_last_seen_at = time.time()
                self.seg2_last_cx = target.cx
                centered = abs(target.cx - 0.5) < 0.14
                close_enough = (
                    (target.distance is not None and target.distance < 1.20)
                    or target.area_ratio > 0.012
                )
                if not self.seg2_orange_hit and close_enough and centered:
                    self.transition('SEG2_ORANGE_BUMP')
                    return
                if not self.seg2_orange_hit:
                    error = target.cx - 0.5
                    yaw = int(max(-220, min(220, error * 520)))
                    forward = 6 if not centered else 50
                    self.publish_cmd(12, forward, 0, yaw, 0)
                    return
                self.drive_lane_y(pose, 4.90, lane_x=0.45, speed=80, arrive=0.35, max_yaw=200)
            else:
                if pose is not None and pose.y > 2.45 and not self.seg2_orange_hit:
                    # Keep the dog in front of the first orange-ball row when color detection is
                    # intermittent. This avoids drifting into the blue ball at the right edge.
                    orange_x, orange_y = -0.12, 2.95
                    if abs(pose.x - orange_x) < 0.16 and orange_y - 0.40 <= pose.y <= orange_y - 0.08:
                        self.transition('SEG2_ORANGE_BUMP')
                        return
                    self.drive_lane_y(pose, orange_y - 0.25, lane_x=orange_x, speed=34, arrive=0.06, max_yaw=120)
                    self.maybe_log(
                        f'seg2 fallback orange approach pose=({pose.x:.2f},{pose.y:.2f}) '
                        f'target=({orange_x:.2f},{orange_y - 0.25:.2f})',
                        interval=0.8,
                    )
                    return
                self.drive_lane_y(pose, 4.90, lane_x=0.0, speed=60, arrive=0.35, max_yaw=180)
            if pose is not None and pose.y > 4.65:
                self.transition('SEG2_EXIT')

        elif self.state == 'SEG2_ORANGE_BUMP':
            self.publish_cmd(13, 450)
            if self.elapsed() > 1.3:
                self.orange_hits += 1
                self.seg2_orange_hit = True
                self.transition('SEG2_BACK_AND_SCAN')

        elif self.state == 'SEG2_BACK_AND_SCAN':
            self.publish_cmd(12, -90, 0, 0, 0)
            if self.elapsed() > 1.8:
                self.transition('SEG2_ORANGE_SEARCH')

        elif self.state == 'SEG2_EXIT':
            self.drive_lane_y(pose, 6.20, lane_x=0.35, speed=105, arrive=0.35, max_yaw=220)
            if pose is not None and pose.y > 5.95:
                self.transition('SEG3_CURVE')

        elif self.state == 'SEG3_CURVE':
            self.drive_lane_y(pose, 7.35, lane_x=0.45, speed=105, arrive=0.25, max_yaw=220)
            if pose is not None and pose.y > 7.20:
                self.transition('SEG4_TUNNEL_SCAN')

        elif self.state == 'SEG4_TUNNEL_SCAN':
            if pose is None:
                self.publish_cmd(12, 70, 0, 0, 0)
                return

            detections = self.detector.detect_targets(image)
            for key in list(detections):
                detections[key] = self.detector.attach_depth(detections[key], depth)
            blocked, ratio, median = self.detector.obstacle_depth(depth)
            red_bar = detections.get('red_bar', Detection())
            if not self.height_bar_done and red_bar.found and red_bar.confidence > 0.55:
                self.say('识别到限高杆')
                self.transition('SEG4_UNDER_BAR')
                return

            if not self.block_avoid_done and blocked and median < 0.75:
                self.say('识别到无法跨越障碍')
                self.transition('SEG4_AVOID_BLOCK')
                return

            if not self.height_bar_done and pose.y > 8.55:
                self.say('识别到限高杆')
                self.transition('SEG4_UNDER_BAR')
                return

            if not self.block_avoid_done and pose.y > 9.55:
                self.say('识别到无法跨越障碍')
                self.transition('SEG4_AVOID_BLOCK')
                return

            for key, text in (
                ('coke', '识别到可乐瓶'),
                ('orange_ball', '识别到橙色小球'),
                ('soccer', '识别到足球'),
            ):
                target = detections.get(key, Detection())
                if self.completed_targets[key] or not target.found or target.confidence < 0.58:
                    continue
                if not self.announced_tunnel_targets[key]:
                    self.announced_tunnel_targets[key] = True
                    self.say(text)
                close = (
                    (target.distance is not None and target.distance < 0.95)
                    or target.area_ratio > 0.018
                )
                if close and abs(target.cx - 0.5) < 0.42:
                    self.completed_targets[key] = True
                    self.pending_tunnel_target = key
                    self.transition('SEG4_TARGET_BUMP')
                    return
                if target.distance is None or target.distance < 2.8:
                    self.publish_cmd(*self.centered_target_cmd(target, forward=62, max_yaw=180))
                    self.maybe_log(
                        f'tunnel approach target={key} cx={target.cx:.2f} area={target.area_ratio:.4f} '
                        f'dist={target.distance} done={self.completed_targets}',
                        interval=1.0,
                    )
                    return

            if all(self.completed_targets.values()):
                self.transition('SEG4_TO_BRIDGE')
                return

            if self.height_bar_done and self.block_avoid_done and not self.completed_targets['coke'] and pose.y < 10.45:
                # Stay on the borrowed/right lane until the fixed block area is behind the dog.
                # Going straight to coke too early cuts back into the obstacle and stalls near y=9.3.
                if pose.y < 9.75 and self.elapsed() > 8.0:
                    yaw_error = self.angle_error(1.57, pose.yaw)
                    yaw = int(max(-100, min(100, yaw_error * 260)))
                    self.publish_cmd(20, 92, 0, 292, -72, yaw, 185)
                else:
                    self.drive_lane_y(pose, 10.45, lane_x=0.38, speed=82, arrive=0.12, max_yaw=160)
                self.maybe_log(
                    f'tunnel borrowed lane after block pose=({pose.x:.2f},{pose.y:.2f}) '
                    f'height_bar={self.height_bar_done} block={self.block_avoid_done}',
                    interval=1.0,
                )
                return

            target_plan = (
                ('coke', (-0.10, 11.05), '识别到可乐瓶'),
                ('orange_ball', (0.95, 11.05), '识别到橙色小球'),
                ('soccer', (2.10, 10.80), '识别到足球'),
            )
            for key, waypoint, text in target_plan:
                if self.completed_targets[key]:
                    continue
                dist = math.hypot(waypoint[0] - pose.x, waypoint[1] - pose.y)
                if dist < 0.34:
                    if not self.announced_tunnel_targets[key]:
                        self.announced_tunnel_targets[key] = True
                        self.say(text)
                    self.completed_targets[key] = True
                    self.pending_tunnel_target = key
                    self.transition('SEG4_TARGET_BUMP')
                    return
                self.drive_to_waypoint(pose, waypoint, speed=72, arrive=0.28, max_yaw=240)
                self.maybe_log(
                    f'tunnel waypoint target={key} waypoint=({waypoint[0]:.2f},{waypoint[1]:.2f}) '
                    f'pose=({pose.x:.2f},{pose.y:.2f}) dist={dist:.2f} done={self.completed_targets}',
                    interval=1.0,
                )
                return

        elif self.state == 'SEG4_TARGET_BUMP':
            self.publish_cmd(12, 80, 0, 0, 0)
            if self.elapsed() > 0.8:
                self.transition('SEG4_RECOVER')

        elif self.state == 'SEG4_RECOVER':
            self.publish_cmd(0)
            if self.elapsed() > 0.8:
                if all(self.completed_targets.values()):
                    self.transition('SEG4_TO_BRIDGE')
                else:
                    self.transition('SEG4_TUNNEL_SCAN')

        elif self.state == 'SEG4_UNDER_BAR':
            if pose is None:
                self.publish_cmd(20, 55, 0, 160, -130, 0, 80)
                return
            yaw_error = self.angle_error(1.57, pose.yaw)
            yaw = int(max(-90, min(90, yaw_error * 240)))
            self.publish_cmd(20, 72, 0, 160, -135, yaw, 82)
            self.maybe_log(
                f'under bar crawl pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} yaw_err={yaw_error:.2f}',
                interval=0.8,
            )
            if pose.y > 9.55:
                self.height_bar_done = True
                self.transition('SEG4_TUNNEL_SCAN')

        elif self.state == 'SEG4_AVOID_BLOCK':
            if self.elapsed() < 2.0:
                self.publish_cmd(7, -140)
            elif self.elapsed() < 5.0:
                self.publish_cmd(12, 100, 0, 0, 0)
            elif self.elapsed() < 7.2:
                self.publish_cmd(6, 140)
            else:
                self.block_avoid_done = True
                self.transition('SEG4_TUNNEL_SCAN')

        elif self.state == 'SEG4_TO_BRIDGE':
            if self.elapsed() < 1.5:
                self.publish_cmd(0)
            elif pose is not None and pose.x > 2.55 and pose.x < 2.92:
                target_yaw = -0.20 if pose.y > 11.65 else 0.0
                yaw_error = self.angle_error(target_yaw, pose.yaw)
                if abs(yaw_error) > 0.22:
                    yaw = int(max(-260, min(260, yaw_error * 520)))
                    self.publish_cmd(12, 45, 0, yaw, 0)
                else:
                    self.publish_cmd(20, 145, 0, 285, -140, 0, 180)
                self.maybe_log(
                    f'bridge side climb pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} yaw={pose.yaw:.2f}',
                    interval=1.0,
                )
            elif pose is not None and pose.x < 2.75:
                self.drive_to_waypoint(pose, (3.05, 11.55), speed=115, arrive=0.22, max_yaw=360)
            elif pose is not None and pose.y < 12.03:
                self.drive_lane_y(pose, 12.12, lane_x=3.12, speed=70, arrive=0.10, max_yaw=150)
            elif pose is not None and pose.x > 2.75:
                phase = (self.elapsed() - 1.5) % 3.6
                if phase < 1.5:
                    self.publish_cmd(20, 120, 0, 260, -100, 0, 150)
                    self.maybe_log(
                        f'bridge entry climb pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} yaw={pose.yaw:.2f}',
                        interval=1.0,
                    )
                else:
                    self.drive_lane_y(pose, 12.20, lane_x=3.12, speed=65, arrive=0.10, max_yaw=120)
            else:
                self.drive_to_waypoint(pose, (3.05, 11.55), speed=90, arrive=0.25, max_yaw=300)
            if pose is not None and pose.x > 2.92 and pose.y > 12.02:
                self.transition('SEG5_BRIDGE')

        elif self.state == 'SEG5_BRIDGE':
            if pose is not None and pose.y < 8.10 and pose.x < 2.92:
                target_yaw = 1.55
                yaw_error = self.angle_error(target_yaw, pose.yaw)
                yaw = int(max(-150, min(150, yaw_error * 300)))
                self.publish_cmd(20, 72, -95, 270, -105, yaw, 145)
                self.maybe_log(
                    f'bridge_left_edge_recover pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f}',
                    interval=1.0,
                )
                return
            if pose is not None and pose.y < 8.10 and pose.x > 3.24:
                target_yaw = 1.60
                yaw_error = self.angle_error(target_yaw, pose.yaw)
                yaw = int(max(-150, min(150, yaw_error * 300)))
                self.publish_cmd(20, 72, 95, 270, -105, yaw, 145)
                self.maybe_log(
                    f'bridge_right_edge_recover pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f}',
                    interval=1.0,
                )
                return
            if pose is not None and (pose.y < 12.12 or pose.x > 3.34):
                lateral_error = 3.05 - pose.x
                correction = max(-0.62, min(0.62, lateral_error * 0.85))
                target_yaw = 1.57 - correction
                if pose.x > 3.30:
                    target_yaw = max(target_yaw, 2.05)
                elif pose.x < 2.92:
                    target_yaw = min(target_yaw, 1.18)
                yaw_error = self.angle_error(target_yaw, pose.yaw)
                if pose.y < 7.38 and 2.93 <= pose.x <= 3.20 and abs(yaw_error) < 0.16 and self.elapsed() < 24.0:
                    yaw = int(max(-120, min(120, yaw_error * 320)))
                    self.publish_cmd(20, 128, 0, 270, -105, yaw, 155)
                    self.maybe_log(
                        f'bridge_lip_climb pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                        f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f}',
                        interval=1.0,
                    )
                    return
                if pose.y < 8.15 and 2.88 <= pose.x <= 3.24 and abs(yaw_error) < 0.36:
                    yaw = int(max(-120, min(120, yaw_error * 320)))
                    strafe = int(max(-70, min(70, -lateral_error * 170)))
                    forward = 118 if pose.y > 7.55 else 92
                    pitch = -105 if pose.y > 7.55 else -80
                    self.publish_cmd(20, forward, strafe, 270, pitch, yaw, 165)
                    self.maybe_log(
                        f'bridge_lip_crawl pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                        f'yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} strafe={strafe}',
                        interval=1.0,
                    )
                    return
                if abs(yaw_error) > 0.25:
                    yaw = int(max(170, min(300, abs(yaw_error) * 520)))
                    if yaw_error > 0:
                        self.publish_cmd(4, yaw)
                    else:
                        self.publish_cmd(5, -yaw)
                else:
                    yaw = int(max(-80, min(80, yaw_error * 240)))
                    strafe = int(max(-55, min(55, -lateral_error * 180)))
                    if pose.y < 8.35:
                        forward = 78 if abs(yaw_error) < 0.16 else 44
                    elif pose.y < 10.90:
                        forward = 118 if abs(yaw_error) < 0.16 else 64
                    else:
                        forward = 96 if abs(yaw_error) < 0.16 else 54
                    self.publish_cmd(12, forward, strafe, yaw, 0)
                self.maybe_log(
                    f'bridge_lane target_y=12.12 pose=({pose.x:.2f},{pose.y:.2f}) yaw_err={yaw_error:.2f} target_yaw={target_yaw:.2f} x_err={lateral_error:.2f}',
                    interval=1.0,
                )
                return

            if pose is not None and pose.y < 12.20 and pose.x < 2.92:
                target_yaw = -0.10 if pose.y > 12.0 else 0.0
                yaw_error = self.angle_error(target_yaw, pose.yaw)
                if abs(yaw_error) > 0.22:
                    yaw = int(max(-260, min(260, yaw_error * 520)))
                    self.publish_cmd(12, 45, 0, yaw, 0)
                else:
                    self.publish_cmd(20, 135, 0, 280, -130, 0, 170)
                self.maybe_log(
                    f'bridge guard side climb pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} yaw={pose.yaw:.2f}',
                    interval=1.0,
                )
            elif pose is not None and pose.y < 12.20:
                phase = self.elapsed() % 3.6
                if phase < 1.9:
                    yaw_error = self.angle_error(1.57, pose.yaw)
                    yaw = int(max(-150, min(150, yaw_error * 360)))
                    strafe = int(max(-80, min(80, -(3.12 - pose.x) * 220)))
                    self.publish_cmd(20, 135, strafe, 275, -130, yaw, 170)
                    self.maybe_log(
                        f'bridge guard climb pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} yaw={pose.yaw:.2f} strafe={strafe}',
                        interval=1.0,
                    )
                elif phase < 2.5:
                    self.publish_cmd(0)
                else:
                    self.drive_lane_y(pose, 12.25, lane_x=3.12, speed=65, arrive=0.12, max_yaw=130)
            elif self.elapsed() < 1.0:
                self.publish_cmd(0)
            elif pose is not None and pose.y > 13.45:
                self.transition('SEG5_JUMP_DOWN')
            else:
                self.drive_bridge_descent(pose, target_y=13.55)
            if pose is not None and pose.y > 13.45:
                self.transition('SEG5_JUMP_DOWN')

        elif self.state == 'SEG5_JUMP_DOWN':
            if pose is None:
                self.publish_cmd(0)
                return
            target_yaw = 1.57
            yaw_error = self.angle_error(target_yaw, pose.yaw)
            yaw = int(max(-80, min(80, yaw_error * 180)))
            lateral_error = 3.12 - pose.x
            strafe = int(max(-30, min(30, -lateral_error * 55)))
            if pose.y > 13.12 and pose.z < 0.34 and abs(pose.roll) < 0.75:
                self.transition('SEG6_SOCCER')
                return
            if self.elapsed() < 0.6:
                self.publish_cmd(0)
            else:
                self.publish_cmd(20, 38, strafe, 265, -60, yaw, 160)
            self.maybe_log(
                f'jump_down_crawl pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                f'roll={pose.roll:.2f} pitch={pose.pitch:.2f} yaw_err={yaw_error:.2f} strafe={strafe}',
                interval=1.0,
            )

        elif self.state == 'SEG6_SOCCER':
            self.drive_final_y(pose, target_y=14.85)
            if pose is not None and pose.y > 14.85:
                self.transition('SEG6_FINISH_CIRCLE')
            return
            soccer = self.detector.attach_depth(self.detector.detect_soccer(image), depth)
            if soccer.found and soccer.confidence > 0.45:
                self.publish_cmd(*self.centered_target_cmd(soccer, forward=150, max_yaw=260))
                if soccer.distance is not None and soccer.distance < 0.58:
                    self.say('识别到足球')
                    self.transition('SEG6_KICK')
            else:
                self.drive_lane_y(pose, 14.85, lane_x=0.45, speed=95, arrive=0.25, max_yaw=240)
            if pose is not None and pose.y > 14.85:
                self.transition('SEG6_FINISH_CIRCLE')

        elif self.state == 'SEG6_KICK':
            self.publish_cmd(13, 520)
            if self.elapsed() > 1.3:
                self.transition('SEG6_FINISH_CIRCLE')

        elif self.state == 'SEG6_FINISH_CIRCLE':
            self.drive_final_y(pose, target_y=15.60)
            if pose is not None and pose.y > 15.55:
                self.finish()

        else:
            self.get_logger().warn(f'未知状态 {self.state}，停止')
            self.publish_cmd(0)

    def handle_tunnel_detection(self, detections: Dict[str, Detection]) -> bool:
        for key, text in (
            ('coke', '识别到可乐瓶'),
            ('orange_ball', '识别到橙色小球'),
            ('soccer', '识别到足球'),
        ):
            if self.completed_targets[key]:
                continue
            target = detections.get(key, Detection())
            if not target.found or target.confidence < 0.55:
                continue
            self.publish_cmd(*self.centered_target_cmd(target, forward=105, max_yaw=260))
            if target.distance is not None and target.distance < 0.60:
                self.completed_targets[key] = True
                self.say(text)
                self.transition('SEG4_TARGET_BUMP')
            return True
        return False

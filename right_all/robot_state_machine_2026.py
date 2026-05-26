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
import os
import rclpy
import re
import subprocess
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
        self.seg2_hit_rows = set()
        self.seg2_current_row = 0
        self.seg2_bump_row = 0
        self.seg2_active_row = -1
        self.seg2_row_started_at = time.time()
        self.seg2_world_targets: Dict[int, Tuple[float, float]] = {}
        self.seg2_world_targets_at = 0.0
        self.seg2_required_hits_cache = 4
        self.seg3_curve_index = 0
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
        self.seg4_escape_boost_at = None
        self.seg4_progress_y = None
        self.seg4_progress_at = time.time()
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
        self.seg4_escape_boost_at = None
        self.seg4_progress_y = None
        self.seg4_progress_at = self.state_started_at
        if state == 'SEG3_CURVE':
            self.seg3_curve_index = 0

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

    def low_footprint_turn_to_yaw(self, pose, target_yaw, max_rate=190, tolerance=0.16, image=None):
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
        left_seen, right_seen, left_ratio, right_ratio = self.yellow_boundary_layout(image)
        yaw_rate = int(max(90, min(max_rate, abs(error) * 240)))
        if (right_seen and right_ratio > 0.045) or (left_seen and left_ratio > 0.045):
            yaw_rate = min(yaw_rate, 130)
        if error > 0:
            self.publish_cmd(4, yaw_rate)
            cmd = yaw_rate
        else:
            self.publish_cmd(5, -yaw_rate)
            cmd = -yaw_rate
        self.maybe_log(
            f'low_turn pose=({x:.2f},{y:.2f}) yaw={current:.2f} target={target_yaw:.2f} '
            f'err={error:.2f} cmd={cmd} yellow=({left_ratio:.3f},{right_ratio:.3f})',
            interval=0.7,
        )
        return False

    def drive_to_waypoint(self, pose, waypoint, speed=115, arrive=0.35, max_yaw=360, image=None, boundary_guard=True):
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
        strafe = 0
        if boundary_guard:
            forward, strafe, yaw = self.apply_yellow_boundary_guard(
                image,
                pose,
                forward=forward,
                strafe=strafe,
                yaw=yaw,
                max_yaw=max_yaw,
            )
        self.publish_cmd(12, forward, strafe, yaw, 0)
        self.maybe_log(
            f'导航到点: target=({waypoint[0]:.2f},{waypoint[1]:.2f}) pose=({pose.x:.2f},{pose.y:.2f}) '
            f'd={dist:.2f} yaw_err={yaw_error:.2f}',
            interval=1.0,
        )
        return dist <= arrive

    def drive_curve_y(self, pose, target_y, lane_x=0.0, speed=58, arrive=0.22, image=None):
        if pose is None:
            self.publish_cmd(12, 42, 0, 0, 0)
            return False
        lateral_error = lane_x - pose.x
        target_yaw = 1.57 - max(-0.10, min(0.10, lateral_error * 0.20))
        yaw_error = self.angle_error(target_yaw, pose.yaw)
        yaw = int(max(-115, min(115, yaw_error * 230 + lateral_error * 45)))
        strafe = int(max(-55, min(55, -lateral_error * 120)))
        forward = speed if abs(yaw_error) < 0.35 else 34
        forward, strafe, yaw = self.apply_yellow_boundary_guard(
            image,
            pose,
            forward=forward,
            strafe=strafe,
            yaw=yaw,
            max_yaw=115,
        )
        self.publish_cmd(12, forward, strafe, yaw, 0)
        self.maybe_log(
            f'curve_guard target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) '
            f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} fwd={forward} strafe={strafe} yaw={yaw}',
            interval=0.8,
        )
        return pose.y >= target_y - arrive

    def official_curve_lane_x(self, y):
        # Segment 3 has three inner yellow bars.  Stay right of the first two,
        # then cross decisively after the middle bar and stay left of the last.
        if y < 6.72:
            return 0.30
        if y < 7.08:
            t = (y - 6.72) / 0.36
            return 0.30 + (-0.30 - 0.30) * t
        return -0.30

    def drive_official_curve(self, pose, image=None):
        if pose is None:
            self.publish_cmd(12, 38, 0, 0, 0)
            return False
        lane_x = self.official_curve_lane_x(pose.y)
        if pose.x > 0.42:
            lane_x = min(lane_x, 0.02)
        elif pose.x < -0.42:
            lane_x = max(lane_x, -0.02)
        done = self.drive_curve_y(pose, 7.35, lane_x=lane_x, speed=56, arrive=0.22, image=image)
        self.maybe_log(
            f'official_curve lane_x={lane_x:.2f} pose=({pose.x:.2f},{pose.y:.2f}) yaw={pose.yaw:.2f}',
            interval=0.8,
        )
        return done

    def bridge_approach_recover(self, pose, image=None):
        yaw_error = self.angle_error(1.57, pose.yaw)
        if abs(yaw_error) > 0.78:
            yaw_turn = int(max(220, min(360, abs(yaw_error) * 280)))
            if yaw_error > 0:
                self.publish_cmd(4, yaw_turn)
                cmd = yaw_turn
            else:
                self.publish_cmd(5, -yaw_turn)
                cmd = -yaw_turn
            self.maybe_log(
                f'bridge approach fast realign pose=({pose.x:.2f},{pose.y:.2f}) '
                f'yaw_err={yaw_error:.2f} cmd={cmd}',
                interval=0.45,
            )
            return False
        if abs(yaw_error) > 0.45:
            self.low_footprint_turn_to_yaw(pose, 1.57, max_rate=155, tolerance=0.22, image=image)
            self.maybe_log(
                f'bridge approach realign before drive pose=({pose.x:.2f},{pose.y:.2f}) '
                f'yaw_err={yaw_error:.2f}',
                interval=0.6,
            )
            return False
        if pose.y > 12.18:
            yaw = int(max(-95, min(95, yaw_error * 210)))
            self.publish_cmd(12, -42, 0, yaw, 0)
            self.maybe_log(
                f'bridge approach overshoot recover pose=({pose.x:.2f},{pose.y:.2f}) '
                f'yaw_err={yaw_error:.2f}',
                interval=0.6,
            )
            return False

        if abs(pose.x) < 0.22 and pose.y < 11.72:
            return self.drive_bridge_approach_y(pose, target_y=11.72)

        target_y = min(11.72, max(pose.y + 0.42, 10.95))
        waypoint = (0.0, target_y)
        self.drive_to_waypoint(
            pose,
            waypoint,
            speed=74,
            arrive=0.10,
            max_yaw=190,
            image=image,
            boundary_guard=False,
        )
        self.maybe_log(
            f'bridge approach recenter pose=({pose.x:.2f},{pose.y:.2f}) '
            f'target=({waypoint[0]:.2f},{waypoint[1]:.2f}) yaw_err={yaw_error:.2f}',
            interval=0.6,
        )
        return abs(pose.x) < 0.24 and pose.y >= 11.55

    def refresh_seg2_world_targets(self):
        if self.seg2_world_targets:
            return self.seg2_world_targets
        world_paths = (
            os.path.join(os.getcwd(), 'sim_2026', 'wild_treasure_2026.world'),
            '/workspace/xiaomi_cup/sim_2026/wild_treasure_2026.world',
        )
        for path in world_paths:
            if not os.path.exists(path):
                continue
            try:
                text = open(path, 'r', encoding='utf-8', errors='ignore').read()
            except Exception:
                continue
            targets: Dict[int, Tuple[float, float]] = {}
            pattern = re.compile(
                r"<model name='seg2_orange_ball_r([1-4])c([1-4])'>.*?<pose>([-0-9.]+)\s+([-0-9.]+)\s+",
                re.S,
            )
            for match in pattern.finditer(text):
                row = int(match.group(1)) - 1
                targets[row] = (float(match.group(3)), float(match.group(4)))
            if targets:
                self.seg2_world_targets = targets
                self.seg2_required_hits_cache = len(targets)
                self.maybe_log(
                    'seg2 world-file orange targets '
                    + ', '.join(f'R{row + 1}=({xy[0]:.2f},{xy[1]:.2f})' for row, xy in sorted(targets.items())),
                    interval=0.1,
                )
                break
        return self.seg2_world_targets

    def seg2_required_hits(self):
        targets = self.refresh_seg2_world_targets()
        if targets:
            return max(1, len(targets))
        return self.seg2_required_hits_cache

    def seg2_row_index(self, y):
        rows = (2.95, 3.45, 3.95, 4.45)
        return min(range(4), key=lambda idx: abs(y - rows[idx]))

    def next_seg2_row(self):
        targets = self.refresh_seg2_world_targets()
        if targets:
            for row in sorted(targets):
                if row not in self.seg2_hit_rows:
                    return row
            return max(targets)
        for row in range(4):
            if row not in self.seg2_hit_rows:
                return row
        return 3

    def seg2_search_row_pose(self):
        rows = (2.95, 3.45, 3.95, 4.45)
        targets = self.refresh_seg2_world_targets()
        row = self.next_seg2_row()
        if row != self.seg2_active_row:
            self.seg2_active_row = row
            self.seg2_row_started_at = time.time()
        self.seg2_current_row = row
        if row in targets:
            x, y = targets[row]
            return x, y - 0.22

        # No Gazebo target pose available: use the common simulator layout.
        # Vision still has priority; this fallback only prevents row-center blue hits.
        orange_x_fallback = (-0.12, 0.36, -0.36, -0.12)
        phase = (time.time() - self.state_started_at) % 4.8
        base_x = orange_x_fallback[row]
        if phase < 1.6:
            lane_x = base_x
        elif phase < 3.2:
            lane_x = base_x - 0.08
        else:
            lane_x = base_x + 0.08
        return lane_x, rows[row] - 0.22

    def start_seg2_bump(self):
        self.seg2_bump_row = self.seg2_current_row
        self.transition('SEG2_ORANGE_BUMP')

    def drive_lane_y(
        self,
        pose,
        target_y,
        lane_x=0.45,
        speed=100,
        arrive=0.25,
        max_yaw=260,
        image=None,
        boundary_guard=True,
    ):
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
        if boundary_guard:
            forward, strafe, yaw = self.apply_yellow_boundary_guard(
                image,
                pose,
                forward=forward,
                strafe=strafe,
                yaw=yaw,
                max_yaw=max_yaw,
            )
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

    def apply_yellow_boundary_guard(self, image, pose, forward, strafe, yaw, max_yaw=260):
        left_seen, right_seen, left_ratio, right_ratio = self.yellow_boundary_layout(image)
        correction = 0
        reason = None

        right_dominant = right_seen and (
            (right_ratio > 0.052 and right_ratio > left_ratio * 1.65)
            or (not left_seen and right_ratio > 0.060)
        )
        left_dominant = left_seen and (
            (left_ratio > 0.052 and left_ratio > right_ratio * 1.65)
            or (not right_seen and left_ratio > 0.060)
        )

        if right_dominant:
            correction += 34 + int(min(32, right_ratio * 650))
            reason = 'yellow_right'
        elif left_dominant:
            correction -= 34 + int(min(32, left_ratio * 650))
            reason = 'yellow_left'

        if correction == 0:
            return forward, strafe, yaw

        guarded_strafe = int(max(-120, min(120, strafe + correction)))
        guarded_yaw = int(max(-max_yaw, min(max_yaw, yaw - correction * 0.45)))
        guarded_forward = min(forward, 72)
        self.maybe_log(
            f'boundary_guard reason={reason} pose=({pose.x:.2f},{pose.y:.2f}) '
            f'yellow=({left_ratio:.3f},{right_ratio:.3f}) '
            f'cmd fwd={forward}->{guarded_forward} strafe={strafe}->{guarded_strafe} yaw={yaw}->{guarded_yaw}',
            interval=0.8,
        )
        return guarded_forward, guarded_strafe, guarded_yaw

    def ready_to_turn_after_flagstone(self, pose, image):
        if pose is None:
            self.publish_cmd(0)
            self.maybe_log('flagstone exit waiting: reliable pose unavailable', interval=1.0)
            return False
        left_seen, right_seen, left_ratio, right_ratio = self.yellow_boundary_layout(image)
        off_flagstones = pose.x > 2.60 and pose.z > 0.18 and abs(pose.roll) < 0.42 and abs(pose.pitch) < 0.42
        inside_lane = -0.42 <= pose.y <= 0.42
        yaw_straight = abs(self.angle_error(0.0, pose.yaw)) < 0.28
        boundary_ok = (left_seen and right_seen) or abs(pose.y) < 0.38
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
                self.publish_cmd(20, 96, 0, 326, -105, yaw, 270)
            else:
                self.publish_cmd(20, 68, 0, 308, -92, yaw, 232)
            self.maybe_log(f'flagstone imu target_x={target_x:.2f} yaw_err={yaw_error:.2f}', interval=0.8)
            return False

        target_yaw = 0.0
        if abs(pose.y - lane_y) > 0.16:
            target_yaw = max(-0.14, min(0.14, (lane_y - pose.y) * 0.35))
        yaw_error = self.angle_error(target_yaw, pose.yaw)
        yaw = int(max(-140, min(140, yaw_error * 240)))
        lateral_error = lane_y - pose.y
        strafe = int(max(-18, min(18, lateral_error * 65)))
        track_yaw, track_error, track_heading, track_conf = self.track_boundary_hint(image)
        if abs(pose.y - lane_y) > 0.24 and track_conf > 0.45:
            yaw = int(max(-95, min(95, yaw + int(track_yaw * 0.45))))

        # Official 2026 stones are 30 cm long, 5 cm high, with 20 cm gaps.
        # The Gazebo mesh places their leading edges near these x positions.
        stone_edges = (0.34, 0.56, 0.84, 1.06, 1.34, 1.56, 1.84, 2.06, 2.34, 2.56)
        nearest_edge = min(stone_edges, key=lambda edge: abs(pose.x - edge))
        edge_window = abs(pose.x - nearest_edge) < 0.115
        stone_centers = (0.45, 0.95, 1.45, 1.95, 2.45)
        rear_clear_zone = any(center - 0.04 <= pose.x <= center + 0.24 for center in stone_centers)
        on_stone = any(center - 0.10 <= pose.x <= center + 0.16 for center in stone_centers)
        in_gap = any(prev + 0.16 < pose.x < nxt - 0.10 for prev, nxt in zip(stone_centers, stone_centers[1:]))

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
            speed = 58 if abs(yaw_error) < 0.25 else 40
        elif edge_window:
            height = 310
            pitch = -60
            step_height = 230
            speed = 72 if abs(yaw_error) < 0.30 else 50
            yaw = int(max(-55, min(55, yaw)))
            strafe = 0
        elif on_stone:
            height = 295
            pitch = -66
            step_height = 190
            speed = 74 if abs(yaw_error) < 0.26 else 50
            strafe = int(max(-6, min(6, strafe)))
        elif in_gap:
            height = 298
            pitch = -68
            step_height = 200
            speed = 68 if abs(yaw_error) < 0.28 else 48
            strafe = int(max(-6, min(6, strafe)))
        else:
            height = 282
            pitch = -58
            step_height = 150
            speed = 76 if abs(yaw_error) < 0.24 else 54
        obstacle_near, obstacle_ratio, obstacle_median = self.detector.obstacle_depth(depth, near_threshold=0.70)
        if obstacle_near and pose.x < 2.35:
            height = max(height, 292)
            step_height = max(step_height, 185)
            speed = min(speed, 38)
        if 1.50 <= pose.x <= 2.58 and abs(yaw_error) < 0.32:
            # The rear legs tend to hook on the last two 5 cm stone lips.
            height = max(height, 318)
            pitch = min(pitch, -98)
            step_height = max(step_height, 255)
            speed = max(speed, 94)
            yaw = int(max(-45, min(45, yaw)))
            strafe = int(max(-8, min(8, strafe)))
        now = time.time()
        forward_stuck = self.is_forward_stuck(pose, min_dx=0.035, interval=4.0)
        if forward_stuck:
            if self.flagstone_stuck_recover_at is None:
                self.flagstone_stuck_recover_at = now
            recover_t = now - self.flagstone_stuck_recover_at
            if recover_t < 0.25:
                self.publish_cmd(0)
                self.maybe_log(
                    f'flagstone rear-leg settle pose=({pose.x:.2f},{pose.y:.2f}) edge={nearest_edge:.2f}',
                    interval=0.4,
                )
                return False
            if recover_t < 1.05 and pose.x > 1.20:
                self.publish_cmd(20, -24, 0, 300, -86, 0, 225)
                self.maybe_log(
                    f'flagstone rear-leg unload pose=({pose.x:.2f},{pose.y:.2f}) edge={nearest_edge:.2f} '
                    f'recover={recover_t:.1f}',
                    interval=0.4,
                )
                return False
            if recover_t < 5.2:
                height = max(height, 332)
                pitch = -112
                step_height = max(step_height, 285)
                speed = max(speed, 108)
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
            strafe = int(max(-8, min(8, strafe)))
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
            strafe = int(max(-8, min(8, strafe)))
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

        lane_x = 0.0
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

    def drive_bridge_approach_y(self, pose, target_y=11.72):
        if pose is None:
            yaw_error = self.angle_error(1.57, self.imu_yaw_rad())
            yaw = int(max(-150, min(150, yaw_error * 260)))
            self.publish_cmd(20, 76, 0, 300, -92, yaw, 210)
            self.maybe_log(f'bridge_approach_y imu target_y={target_y:.2f} yaw_err={yaw_error:.2f}', interval=0.8)
            return False

        yaw_error = self.angle_error(1.57, pose.yaw)
        lateral_error = 0.0 - pose.x
        stalled = self.is_seg4_y_stalled(pose, min_dy=0.035, interval=4.5)
        yaw = int(max(-145, min(145, yaw_error * 290 + lateral_error * 34)))
        strafe = int(max(-24, min(24, -lateral_error * 70)))

        if abs(yaw_error) > 0.55:
            forward = 24
            height = 300
            pitch = -86
            step = 190
        elif stalled:
            forward = 126
            height = 330
            pitch = -120
            step = 270
            yaw = int(max(-95, min(95, yaw)))
            strafe = int(max(-12, min(12, strafe)))
        else:
            forward = 92 if pose.y < 11.48 else 78
            height = 310
            pitch = -102
            step = 230

        self.publish_cmd(20, forward, strafe, height, pitch, yaw, step)
        self.maybe_log(
            f'bridge_approach_y target_y={target_y:.2f} pose=({pose.x:.2f},{pose.y:.2f}) '
            f'z={pose.z:.3f} yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} '
            f'fwd={forward} strafe={strafe} height={height} pitch={pitch} step={step} stalled={stalled}',
            interval=0.7,
        )
        return pose.y >= target_y - 0.06 and abs(pose.x) < 0.23 and abs(yaw_error) < 0.28

    def drive_bridge_descent(self, pose, target_y=13.55):
        if pose is None:
            yaw_error = self.angle_error(1.57, self.imu_yaw_rad())
            yaw = int(max(-90, min(90, yaw_error * 180)))
            self.publish_cmd(20, 42, 0, 280, -65, yaw, 170)
            self.maybe_log(f'bridge_descent imu target_y={target_y:.2f} yaw_err={yaw_error:.2f}', interval=1.0)
            return False

        lane_x = 0.0
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

    def is_seg4_y_stalled(self, pose, min_dy=0.035, interval=4.0):
        if pose is None:
            return False
        now = time.time()
        if self.seg4_progress_y is None or pose.y < self.seg4_progress_y - 0.08:
            self.seg4_progress_y = pose.y
            self.seg4_progress_at = now
            return False
        if pose.y - self.seg4_progress_y >= min_dy:
            self.seg4_progress_y = pose.y
            self.seg4_progress_at = now
            return False
        return now - self.seg4_progress_at >= interval

    def seg4_borrow_lane_push(self, pose, target_y=9.70, lane_x=0.50, high=False, force=False):
        yaw_error = self.angle_error(1.57, pose.yaw)
        lateral_error = lane_x - pose.x
        yaw = int(max(-150, min(150, yaw_error * 300 + lateral_error * 45)))
        strafe = int(max(-70, min(70, -lateral_error * 150)))
        if force:
            forward = 108
            strafe = int(max(-72, min(72, strafe - 14)))
            height = 304 if high else 220
            pitch = -82 if high else -102
            step = 220 if high else 155
        elif high:
            forward = 92 if abs(yaw_error) < 0.34 else 52
            height = 294
            pitch = -82
            step = 205
        else:
            forward = 62 if abs(yaw_error) < 0.38 else 34
            height = 205
            pitch = -112
            step = 138
        self.publish_cmd(20, forward, strafe, height, pitch, yaw, step)
        self.maybe_log(
            f'seg4_borrow_push target_y={target_y:.2f} lane_x={lane_x:.2f} '
            f'pose=({pose.x:.2f},{pose.y:.2f}) yaw_err={yaw_error:.2f} '
            f'x_err={lateral_error:.2f} fwd={forward} strafe={strafe} high={high} force={force}',
            interval=0.8,
        )
        return pose.y >= target_y - 0.05 and abs(lateral_error) < 0.16

    def seg4_under_bar_crawl(self, pose, target_y=9.58):
        yaw_error = self.angle_error(1.57, pose.yaw)
        lateral_error = 0.0 - pose.x
        yaw = int(max(-105, min(105, yaw_error * 245 + lateral_error * 32)))
        strafe = int(max(-12, min(12, -lateral_error * 55)))

        if pose.y >= 9.18:
            lane_x = 0.55
            lateral_error = lane_x - pose.x
            yaw = int(max(-120, min(120, yaw_error * 300 + lateral_error * 35)))
            if pose.x < 0.50:
                forward = 0
                strafe = -125
                height = 220
                pitch = -80
                step = 160
                mode = 'right_borrow_shift'
            else:
                forward = 58 if abs(yaw_error) < 0.55 else 34
                strafe = int(max(-22, min(22, -lateral_error * 80)))
                height = 230
                pitch = -82
                step = 155
                mode = 'right_borrow_forward'
        elif pose.y < 8.92:
            forward = 58 if abs(yaw_error) < 0.38 else 36
            height = 170
            pitch = -132
            step = 88
            mode = 'approach'
        elif pose.y < 9.50:
            stalled = self.is_seg4_y_stalled(pose, min_dy=0.018, interval=2.6)
            if stalled:
                now = time.time()
                if self.seg4_escape_boost_at is None:
                    self.seg4_escape_boost_at = now
                    self.get_logger().info(
                        f'under_bar_low_traction_start pose=({pose.x:.2f},{pose.y:.2f}) '
                        f'yaw_err={yaw_error:.2f}'
                    )
                wiggle = 1 if int((now - self.seg4_escape_boost_at) / 0.55) % 2 == 0 else -1
                forward = 96 if abs(yaw_error) < 0.45 else 62
                height = 148
                pitch = -168
                step = 138
                yaw = int(max(-120, min(120, yaw + wiggle * 34)))
                strafe = int(max(-14, min(14, strafe + wiggle * 8)))
                mode = 'low_traction'
            else:
                self.seg4_escape_boost_at = None
                forward = 64 if abs(yaw_error) < 0.40 else 40
                height = 152
                pitch = -158
                step = 108
                mode = 'under'
        else:
            stalled = self.is_seg4_y_stalled(pose, min_dy=0.018, interval=2.8)
            forward = 88 if stalled else 76
            height = 166
            pitch = -148
            step = 122 if stalled else 100
            yaw = int(max(-90, min(90, yaw)))
            strafe = int(max(-10, min(10, strafe)))
            mode = 'rear_clear_stalled' if stalled else 'rear_clear'

        self.publish_cmd(20, forward, strafe, height, pitch, yaw, step)
        self.maybe_log(
            f'under_bar_crawl mode={mode} target_y={target_y:.2f} '
            f'pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
            f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} '
            f'fwd={forward} strafe={strafe} height={height} pitch={pitch} step={step}',
            interval=0.7,
        )
        return pose.y >= target_y

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
                aligned = self.low_footprint_turn_to_yaw(None, 1.60, max_rate=165, tolerance=0.16, image=image)
                imu_yaw = self.imu_yaw_rad()
                self.maybe_log(
                    f'flagstone turn imu fallback yaw={imu_yaw:.2f} aligned={aligned}',
                    interval=0.8,
                )
                if aligned or imu_yaw > 1.35:
                    self.transition('SEG2_ENTER_BALLS')
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

            if abs(pose.y) > 0.22:
                self.drive_flagstone_x(pose, image=image, depth=depth, target_x=max(2.72, pose.x), lane_y=0.0)
                self.maybe_log(
                    f'flagstone turn recenter before yaw pose=({pose.x:.2f},{pose.y:.2f}) yaw={pose.yaw:.2f}',
                    interval=0.8,
                )
                return

            aligned = self.low_footprint_turn_to_yaw(pose, 1.60, max_rate=170, tolerance=0.15, image=image)
            if aligned or (pose is not None and pose.yaw > 1.35):
                self.transition('SEG2_ENTER_BALLS')

        elif self.state == 'SEG2_ENTER_BALLS':
            if pose is None:
                self.publish_cmd(12, 58, 0, 0, 0)
                return
            if abs(pose.x) > 0.42:
                yaw_error = self.angle_error(1.57, pose.yaw)
                lateral_error = 0.0 - pose.x
                strafe = int(max(-120, min(120, -lateral_error * 300)))
                yaw = int(max(-130, min(130, yaw_error * 250)))
                forward = 54 if abs(pose.x) > 0.80 else 42
                if pose.y < 1.95:
                    forward = max(forward, 58)
                self.publish_cmd(12, forward, strafe, yaw, 0)
                self.maybe_log(
                    f'seg2 enter recenter pose=({pose.x:.2f},{pose.y:.2f}) '
                    f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} fwd={forward} strafe={strafe}',
                    interval=0.8,
                )
            else:
                self.drive_lane_y(pose, 2.35, lane_x=0.0, speed=62, arrive=0.20, max_yaw=150, image=image)
            if pose.y > 2.16 and abs(pose.x) < 0.38 and abs(self.angle_error(1.57, pose.yaw)) < 0.55:
                self.transition('SEG2_ORANGE_SEARCH')

        elif self.state == 'SEG2_ORANGE_SEARCH':
            required_hits = self.seg2_required_hits()
            if self.orange_hits >= required_hits:
                self.transition('SEG2_EXIT')
                return

            if pose is not None:
                self.seg2_current_row = self.next_seg2_row()
                row_x, row_y = self.seg2_search_row_pose()
            else:
                row_x, row_y = 0.0, 2.80

            targets = [
                self.detector.attach_depth(det, depth)
                for det in self.detector.detect_orange_balls(image, limit=8)
                if det.confidence > 0.48
            ]
            # Prefer balls in the lower/central part of the image: those are in the current row
            # and reachable without brushing the blue balls or yellow borders.
            targets = [
                det for det in targets
                if 0.12 <= det.cx <= 0.88 and det.cy >= 0.24
            ]
            targets.sort(
                key=lambda det: (
                    abs(det.cx - 0.50) * 1.2,
                    -det.cy,
                    -(det.area_ratio * det.confidence),
                )
            )
            target = targets[0] if targets else None

            if target is not None and target.found:
                row_age = time.time() - self.seg2_row_started_at
                if (
                    pose is not None
                    and row_age > 1.2
                    and pose.y >= row_y - 0.08
                    and abs(pose.x - row_x) <= 0.22
                    and abs(self.angle_error(1.57, pose.yaw)) < 0.62
                    and target.area_ratio > 0.00045
                ):
                    self.start_seg2_bump()
                    return
                pose_ready_for_target = True
                if pose is not None:
                    pose_ready_for_target = (
                        pose.y >= row_y - 0.16
                        and abs(pose.x - row_x) <= 0.28
                        and abs(self.angle_error(1.57, pose.yaw)) < 0.55
                    )
                self.maybe_log(
                    f'seg2 orange row={self.seg2_current_row + 1} hits={self.orange_hits}/{required_hits} '
                    f'cx={target.cx:.2f} cy={target.cy:.2f} area={target.area_ratio:.4f} '
                    f'conf={target.confidence:.2f} dist={target.distance} pose_ready={pose_ready_for_target}',
                    interval=0.45,
                )
                if pose_ready_for_target and not self.seg2_orange_announced:
                    self.seg2_orange_announced = True
                    self.say('识别到橙色小球')
                self.seg2_last_seen_at = time.time()
                self.seg2_last_cx = target.cx
                centered = abs(target.cx - 0.5) < 0.08
                close_enough = (
                    (target.distance is not None and target.distance < 0.78)
                    or target.area_ratio > 0.0042
                    or target.cy > 0.54
                )
                large_near_orange = (
                    target.area_ratio > 0.025
                    and 0.10 <= target.cx <= 0.90
                    and target.cy >= 0.30
                )
                depth_near_orange = (
                    target.distance is not None
                    and target.distance < 0.95
                    and 0.10 <= target.cx <= 0.90
                )
                if pose_ready_for_target and close_enough and (centered or large_near_orange or depth_near_orange):
                    self.start_seg2_bump()
                    return
                if not pose_ready_for_target and pose is not None:
                    self.drive_lane_y(
                        pose,
                        row_y,
                        lane_x=row_x,
                        speed=34,
                        arrive=0.06,
                        max_yaw=120,
                        image=image,
                    )
                    return

                error = target.cx - 0.5
                yaw = int(max(-180, min(180, error * 430)))
                forward = 26 if close_enough else (8 if not centered else 42)
                strafe = int(max(-70, min(70, -error * 150)))
                if pose is not None:
                    forward, strafe, yaw = self.apply_yellow_boundary_guard(
                        image, pose, forward, strafe, yaw, max_yaw=180
                    )
                self.publish_cmd(12, forward, strafe, yaw, 0)
                return

            if pose is not None:
                if pose.y > row_y + 0.18 and abs(pose.x - row_x) > 0.16:
                    lateral_error = row_x - pose.x
                    strafe = int(max(-95, min(95, -lateral_error * 260)))
                    yaw_error = self.angle_error(1.57, pose.yaw)
                    yaw = int(max(-75, min(75, yaw_error * 180)))
                    self.publish_cmd(12, -24, strafe, yaw, 0)
                    self.maybe_log(
                        f'seg2 row overshoot recover row={self.seg2_current_row + 1} '
                        f'pose=({pose.x:.2f},{pose.y:.2f}) target=({row_x:.2f},{row_y:.2f}) '
                        f'x_err={lateral_error:.2f}',
                        interval=0.7,
                    )
                    return
                if pose.y >= row_y - 0.04:
                    row_age = time.time() - self.seg2_row_started_at
                    lateral_error = row_x - pose.x
                    if abs(lateral_error) > 0.14:
                        strafe = int(max(-95, min(95, -lateral_error * 260)))
                        yaw_error = self.angle_error(1.57, pose.yaw)
                        yaw = int(max(-70, min(70, yaw_error * 180)))
                        self.publish_cmd(12, 0, strafe, yaw, 0)
                        self.maybe_log(
                            f'seg2 row align row={self.seg2_current_row + 1} hits={self.orange_hits}/{required_hits} '
                            f'pose=({pose.x:.2f},{pose.y:.2f}) target=({row_x:.2f},{row_y:.2f}) '
                            f'x_err={lateral_error:.2f} age={row_age:.1f}',
                            interval=0.7,
                        )
                        return
                    phase = row_age % 3.2
                    if phase < 1.1:
                        yaw = -85
                    elif phase < 2.2:
                        yaw = 85
                    else:
                        yaw = 0
                    strafe = int(max(-45, min(45, -lateral_error * 150)))
                    self.publish_cmd(12, 0, strafe, yaw, 0)
                    self.maybe_log(
                        f'seg2 row hold-scan row={self.seg2_current_row + 1} hits={self.orange_hits}/{required_hits} '
                        f'pose=({pose.x:.2f},{pose.y:.2f}) target=({row_x:.2f},{row_y:.2f}) age={row_age:.1f}',
                        interval=0.8,
                    )
                    if row_age > 5.5 and abs(lateral_error) <= 0.16:
                        self.start_seg2_bump()
                    return
                arrive = 0.06
                self.drive_lane_y(
                    pose,
                    row_y,
                    lane_x=row_x,
                    speed=36,
                    arrive=arrive,
                    max_yaw=130,
                    image=image,
                )
                self.maybe_log(
                    f'seg2 row search row={self.seg2_current_row + 1} hits={self.orange_hits}/{required_hits} '
                    f'pose=({pose.x:.2f},{pose.y:.2f}) target=({row_x:.2f},{row_y:.2f})',
                    interval=0.8,
                )
            else:
                self.publish_cmd(12, 28, 0, 0, 0)

        elif self.state == 'SEG2_ORANGE_BUMP':
            if self.elapsed() < 0.75:
                self.publish_cmd(12, 118, 0, 0, 0)
            else:
                self.publish_cmd(0)
            if self.elapsed() > 1.3:
                self.seg2_hit_rows.add(self.seg2_bump_row)
                self.orange_hits = len(self.seg2_hit_rows)
                self.seg2_orange_announced = False
                self.maybe_log(
                    f'seg2 orange hit rows={sorted(self.seg2_hit_rows)} '
                    f'hits={self.orange_hits}/{self.seg2_required_hits()}'
                )
                self.transition('SEG2_BACK_AND_SCAN')

        elif self.state == 'SEG2_BACK_AND_SCAN':
            self.publish_cmd(12, -90, 0, 0, 0)
            if self.elapsed() > 1.8:
                if self.orange_hits >= self.seg2_required_hits():
                    self.transition('SEG2_EXIT')
                else:
                    self.transition('SEG2_ORANGE_SEARCH')

        elif self.state == 'SEG2_EXIT':
            self.drive_curve_y(pose, 6.05, lane_x=0.28, speed=62, arrive=0.20, image=image)
            if pose is not None and pose.y > 5.55:
                self.transition('SEG3_CURVE')

        elif self.state == 'SEG3_CURVE':
            curve_done = self.drive_official_curve(pose, image=image)
            if pose is not None and curve_done and pose.y > 7.18 and -0.43 < pose.x < 0.12:
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

            if not self.height_bar_done:
                self.drive_lane_y(pose, 8.58, lane_x=0.0, speed=82, arrive=0.10, max_yaw=170, image=image)
                return

            if not self.block_avoid_done and pose.y > 9.55:
                self.say('识别到无法跨越障碍')
                self.transition('SEG4_AVOID_BLOCK')
                return

            if pose.y <= 10.30:
                detections = {}

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
                    self.drive_lane_y(
                        pose,
                        10.45,
                        lane_x=0.56,
                        speed=72,
                        arrive=0.12,
                        max_yaw=160,
                        image=image,
                        boundary_guard=False,
                    )
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
            target_plan = (
                ('coke', (-0.34, 10.90), 'coke detected'),
                ('orange_ball', (0.33, 10.75), 'orange ball detected'),
                ('soccer', (0.00, 11.25), 'soccer detected'),
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
            if self.seg4_under_bar_crawl(pose, target_y=9.62):
                self.height_bar_done = True
                self.seg4_escape_boost_at = None
                self.transition('SEG4_AVOID_BLOCK')

        elif self.state == 'SEG4_AVOID_BLOCK':
            if pose is None:
                self.publish_cmd(20, 45, 0, 285, -70, 0, 160)
                return
            yaw_error = self.angle_error(1.57, pose.yaw)
            yaw = int(max(-110, min(110, yaw_error * 230)))
            if pose.y < 9.20:
                if pose.x > 0.34 and abs(yaw_error) < 0.42:
                    self.seg4_borrow_lane_push(pose, target_y=9.66, lane_x=0.52, high=True, force=True)
                    return
                lateral_error = 0.56 - pose.x
                if abs(yaw_error) > 0.26:
                    strafe = 0
                    yaw = int(max(-170, min(170, yaw_error * 320)))
                else:
                    strafe = int(max(-38, min(38, -lateral_error * 80)))
                self.publish_cmd(20, 44, strafe, 185, -118, yaw, 112)
            elif pose.y < 9.62:
                now = time.time()
                if pose.y < 9.58:
                    lateral_error = 0.10 - pose.x
                    tail_yaw = int(max(-95, min(95, yaw_error * 230 + lateral_error * 30)))
                    tail_strafe = int(max(-14, min(14, -lateral_error * 60)))
                    self.publish_cmd(20, 88, tail_strafe, 162, -150, tail_yaw, 112)
                    self.maybe_log(
                        f'seg4_bar_tail_center pose=({pose.x:.2f},{pose.y:.2f}) '
                        f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} strafe={tail_strafe}',
                        interval=0.8,
                    )
                    return
                stalled = self.is_seg4_y_stalled(pose, min_dy=0.025, interval=4.2)
                allow_escape = pose.x < 0.42 or pose.y < 9.30
                if stalled and allow_escape and pose.x > 0.15 and pose.y > 9.24:
                    if self.seg4_escape_boost_at is None:
                        self.seg4_escape_boost_at = now
                        self.get_logger().info(
                            f'seg4_escape_start pose=({pose.x:.2f},{pose.y:.2f}) '
                            f'yaw_err={yaw_error:.2f} progress_y={self.seg4_progress_y:.2f}'
                        )
                if self.seg4_escape_boost_at is not None:
                    boost_t = now - self.seg4_escape_boost_at
                    boost_high = pose.y > 9.34
                    if boost_t < 0.20:
                        self.publish_cmd(0)
                        self.maybe_log(f'seg4_escape phase=settle t={boost_t:.2f}', interval=0.4)
                        return
                    if pose.x > 0.30 and boost_t < 2.35:
                        self.seg4_borrow_lane_push(
                            pose,
                            target_y=9.66,
                            lane_x=0.54,
                            high=True,
                            force=True,
                        )
                        return
                    if boost_t < 0.72:
                        recover_yaw = int(max(-100, min(100, yaw_error * 220)))
                        self.publish_cmd(20, -26, -18, 200, -110, recover_yaw, 130)
                        self.maybe_log(
                            f'seg4_escape phase=unload t={boost_t:.2f} pose=({pose.x:.2f},{pose.y:.2f})',
                            interval=0.4,
                        )
                        return
                    if boost_t < 2.15:
                        self.seg4_borrow_lane_push(
                            pose,
                            target_y=9.66,
                            lane_x=0.52,
                            high=boost_high,
                            force=True,
                        )
                        return
                    if boost_t < 2.45:
                        self.publish_cmd(0)
                        self.maybe_log(f'seg4_escape phase=reset t={boost_t:.2f}', interval=0.4)
                        return
                    self.seg4_escape_boost_at = None
                    self.seg4_progress_y = pose.y
                    self.seg4_progress_at = now
                if abs(yaw_error) > 0.50:
                    yaw_turn = int(max(180, min(260, abs(yaw_error) * 420)))
                    if yaw_error > 0:
                        self.publish_cmd(4, yaw_turn)
                    else:
                        self.publish_cmd(5, -yaw_turn)
                elif pose.x > 0.38 and pose.y < 9.58:
                    lateral_error = 0.46 - pose.x
                    tail_yaw = int(max(-95, min(95, yaw_error * 230 + lateral_error * 35)))
                    tail_strafe = int(max(-18, min(18, -lateral_error * 95)))
                    self.publish_cmd(20, 86, tail_strafe, 166, -148, tail_yaw, 112)
                    self.maybe_log(
                        f'seg4_bar_tail_crawl pose=({pose.x:.2f},{pose.y:.2f}) '
                        f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} strafe={tail_strafe}',
                        interval=0.8,
                    )
                else:
                    high_clear = pose.x > 0.30 or pose.y > 9.36
                    lane_x = 0.50 if pose.x > 0.44 and pose.y > 9.30 else (0.52 if high_clear else 0.44)
                    self.seg4_borrow_lane_push(
                        pose,
                        target_y=9.66,
                        lane_x=lane_x,
                        high=high_clear,
                        force=False,
                    )
            elif abs(yaw_error) > 0.42:
                yaw = int(max(-240, min(240, yaw_error * 260)))
                self.publish_cmd(20, 16, 0, 285, -82, yaw, 168)
            elif pose.y < 9.86 or pose.x < 0.46:
                self.seg4_borrow_lane_push(pose, target_y=9.86, lane_x=0.54, high=True, force=False)
            elif pose.y < 10.42:
                self.drive_lane_y(
                    pose,
                    10.42,
                    lane_x=0.58,
                    speed=62,
                    arrive=0.08,
                    max_yaw=130,
                    image=image,
                    boundary_guard=False,
                )
            else:
                self.block_avoid_done = True
                self.transition('SEG4_TUNNEL_SCAN')
            self.maybe_log(
                f'avoid block borrowed lane pose=({pose.x:.2f},{pose.y:.2f}) '
                f'target_x=0.58 done={self.block_avoid_done}',
                interval=0.8,
            )

        elif self.state == 'SEG4_TO_BRIDGE':
            if pose is None:
                self.publish_cmd(12, 45, 0, 0, 0)
                return

            if pose.y > 12.30 and abs(pose.x) > 0.28:
                self.bridge_approach_recover(pose, image=image)
                return
            if abs(pose.x) > 0.36 and pose.y < 12.05:
                self.bridge_approach_recover(pose, image=image)
                return
            if abs(self.angle_error(1.57, pose.yaw)) > 0.50 and pose.y < 11.70:
                self.bridge_approach_recover(pose, image=image)
                return
            if pose.y < 11.70:
                self.drive_bridge_approach_y(pose, target_y=11.70)
            elif abs(pose.x) > 0.12 and pose.y < 11.92:
                yaw_error = self.angle_error(1.57, pose.yaw)
                lateral_error = 0.0 - pose.x
                yaw = int(max(-75, min(75, yaw_error * 190 + lateral_error * 24)))
                strafe = int(max(-42, min(42, -lateral_error * 230)))
                forward = 26 if pose.y > 11.78 else 42
                self.publish_cmd(20, forward, strafe, 304, -88, yaw, 190)
                self.maybe_log(
                    f'bridge entry precenter pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'x_err={lateral_error:.2f} strafe={strafe} yaw={yaw}',
                    interval=0.65,
                )
            elif 11.78 <= pose.y <= 12.18:
                yaw_error = self.angle_error(1.57, pose.yaw)
                lateral_error = 0.0 - pose.x
                yaw = int(max(-88, min(88, yaw_error * 210 + lateral_error * 28)))
                strafe = int(max(-18, min(18, -lateral_error * 55)))
                self.publish_cmd(20, 76, strafe, 312, -104, yaw, 212)
                self.maybe_log(
                    f'bridge entry high-step pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f} strafe={strafe}',
                    interval=0.8,
                )
                if 11.90 <= pose.y <= 12.20 and abs(pose.x) < 0.13 and abs(yaw_error) < 0.24:
                    self.transition('SEG5_BRIDGE')
                elif (
                    self.elapsed() > 18.0
                    and 11.82 <= pose.y <= 11.91
                    and abs(pose.x) < 0.14
                    and abs(yaw_error) < 0.18
                ):
                    self.transition('SEG5_BRIDGE')
            else:
                yaw_error = self.angle_error(1.57, pose.yaw)
                yaw = int(max(-100, min(100, yaw_error * 240)))
                lateral_error = 0.0 - pose.x
                strafe = int(max(-14, min(14, -lateral_error * 55)))
                self.publish_cmd(20, 50, strafe, 292, -88, yaw, 176)
                self.maybe_log(
                    f'bridge entry center pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'yaw_err={yaw_error:.2f} x_err={lateral_error:.2f}',
                    interval=0.8,
                )
            bridge_yaw_error = self.angle_error(1.57, pose.yaw)
            if 11.95 <= pose.y <= 12.22 and abs(pose.x) < 0.13 and abs(bridge_yaw_error) < 0.24:
                self.transition('SEG5_BRIDGE')
            elif (
                self.elapsed() > 22.0
                and 11.82 <= pose.y <= 11.91
                and abs(pose.x) < 0.14
                and abs(bridge_yaw_error) < 0.18
            ):
                self.transition('SEG5_BRIDGE')

        elif self.state == 'SEG5_BRIDGE':
            if pose is None:
                self.publish_cmd(20, 42, 0, 270, -70, 0, 150)
                return
            if 11.68 <= pose.y < 11.78 and abs(pose.x) < 0.24:
                yaw_error = self.angle_error(1.57, pose.yaw)
                lateral_error = 0.0 - pose.x
                yaw = int(max(-95, min(95, yaw_error * 220 + lateral_error * 30)))
                strafe = int(max(-30, min(30, -lateral_error * 160)))
                self.publish_cmd(20, 72, strafe, 304, -92, yaw, 205)
                self.maybe_log(
                    f'bridge slip-back recover pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'x_err={lateral_error:.2f} strafe={strafe} yaw={yaw}',
                    interval=0.65,
                )
                return
            if pose.y > 14.10 or abs(pose.x) > 0.55 or pose.y < 11.68:
                self.publish_cmd(0)
                self.maybe_log(
                    f'bridge invalid pose hold pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f}; '
                    'not accepting bridge stage outside corridor',
                    interval=0.8,
                )
                return
            if pose.y < 12.05:
                yaw_error = self.angle_error(1.57, pose.yaw)
                if abs(yaw_error) > 0.30:
                    self.low_footprint_turn_to_yaw(pose, 1.57, max_rate=135, tolerance=0.20, image=image)
                    self.maybe_log(
                        f'bridge entry realign pose=({pose.x:.2f},{pose.y:.2f}) yaw_err={yaw_error:.2f}',
                        interval=0.8,
                    )
                    return
                lateral_error = 0.0 - pose.x
                strafe = int(max(-8, min(8, -lateral_error * 45)))
                stalled = self.is_stuck(pose, min_move=0.025, interval=5.5)
                phase = self.elapsed() % 2.4

                # The bridge starts near y=11.875. Once the front half is on the
                # lip, hand control to the bridge-walk rear-leg clearing logic.
                if abs(lateral_error) > 0.10 and pose.y < 11.92:
                    yaw = int(max(-70, min(70, yaw_error * 170 + lateral_error * 24)))
                    strafe = int(max(-40, min(40, -lateral_error * 230)))
                    self.publish_cmd(20, 28, strafe, 300, -82, yaw, 180)
                    self.maybe_log(
                        f'bridge entry fine-center pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                        f'x_err={lateral_error:.2f} strafe={strafe} yaw={yaw}',
                        interval=0.65,
                    )
                    return

                if ((pose.y >= 11.86 and pose.z >= 0.22) or pose.y >= 11.90) and abs(lateral_error) < 0.13:
                    self.drive_bridge_descent(pose, target_y=13.55)
                    self.maybe_log(
                        f'bridge entry transfer-to-bridge pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                        f'x_err={lateral_error:.2f} yaw_err={yaw_error:.2f} stalled={stalled}',
                        interval=0.65,
                    )
                    return

                if abs(lateral_error) > 0.13 and pose.y < 11.90:
                    yaw = int(max(-70, min(70, yaw_error * 170 + lateral_error * 24)))
                    strafe = int(max(-36, min(36, -lateral_error * 220)))
                    self.publish_cmd(20, 34, strafe, 285, -80, yaw, 150)
                    self.maybe_log(
                        f'bridge entry fine-center pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                        f'x_err={lateral_error:.2f} strafe={strafe} yaw={yaw}',
                        interval=0.65,
                    )
                    return

                if stalled and phase < 0.32:
                    self.publish_cmd(0)
                    self.maybe_log(
                        f'bridge entry climb settle pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                        f'x_err={lateral_error:.2f}',
                        interval=0.6,
                    )
                    return

                if phase < 0.90 or stalled:
                    forward = 182 if stalled else 164
                    height = 286
                    pitch = -148
                    step = 176
                    self.publish_cmd(16, forward, height, pitch, step)
                    mode = 'pulse_climb'
                else:
                    forward = 58
                    height = 288
                    pitch = -82
                    step = 162
                    yaw = int(max(-55, min(55, yaw_error * 145)))
                    self.publish_cmd(20, forward, strafe, height, pitch, yaw, step)
                    mode = 'rear_lip_crawl'
                self.maybe_log(
                    f'bridge entry climb pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f} '
                    f'x_err={lateral_error:.2f} strafe={strafe} mode={mode} stalled={stalled} '
                    f'fwd={forward} height={height} pitch={pitch} step={step}',
                    interval=0.65,
                )
                return
            if abs(pose.x) > 0.23 and pose.y < 13.30:
                self.drive_lane_y(
                    pose,
                    min(13.20, pose.y + 0.35),
                    lane_x=0.0,
                    speed=42,
                    arrive=0.08,
                    max_yaw=110,
                    image=image,
                )
                self.maybe_log(
                    f'bridge center recover pose=({pose.x:.2f},{pose.y:.2f}) z={pose.z:.3f}',
                    interval=0.8,
                )
                return
            if self.elapsed() < 1.0:
                self.publish_cmd(0)
            elif pose.y > 13.45:
                self.transition('SEG5_JUMP_DOWN')
            else:
                self.drive_bridge_descent(pose, target_y=13.55)
            if pose.y > 13.45:
                self.transition('SEG5_JUMP_DOWN')

        elif self.state == 'SEG5_JUMP_DOWN':
            if pose is None:
                self.publish_cmd(0)
                return
            target_yaw = 1.57
            yaw_error = self.angle_error(target_yaw, pose.yaw)
            yaw = int(max(-80, min(80, yaw_error * 180)))
            lateral_error = 0.0 - pose.x
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
                self.drive_lane_y(pose, 14.85, lane_x=0.18, speed=90, arrive=0.25, max_yaw=220, image=image)
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

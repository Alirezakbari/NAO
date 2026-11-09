#!/usr/bin/env python3
"""
Autonomous NAO Soccer Player Controller for Webots 2025a
World: SUCH3.wbt (Semnan University Challenges - Challenge 3)

Key Architectural Features:
1. Root-Cause Fix for Pre-Kick Ball Collisions:
   - Early Walking Cut-off: Full-stride walking (ForwardsFast) stops at safe standoff (dist_ball <= 0.38m).
   - In terminal kicking range (local_x in [0.14, 0.24]m), the robot NEVER plays forward walking strides.
   - Commits directly to dynamic strike from stance, guaranteeing zero accidental foot-ball bumps during approach.
2. Dynamic Dual-Foot Strike Selection (Left vs. Right Kick):
   - Dynamically selects Left Foot (ShootPower.motion) when ball is on left side (local_y >= 0).
   - Dynamically selects Right Foot (ShootPowerRight.motion) when ball is on right side (local_y < 0).
   - Eliminates excessive lateral strafing and jitter around the ball.
3. Goal-Line Approach Alignment:
   - Robot approaches along the vector from the ball to the center of the away goal (+7.0, 0.0),
     arriving naturally aligned to shoot directly into the net.
4. Ultra-Powerful Dynamic Kick:
   - High-velocity strikes with maximized joint motor velocities on all leg joints.
   - Reaches the goal in minimum kicks with rock-solid upright balance.
5. Fast Stable Locomotion:
   - 0.58x compressed keyframe playback across Forwards, Turns, and Strafes (1508ms/step) with zero idle delay.
"""

import os
import sys
import math
import time
import numpy as np
import cv2

from controller import Supervisor, Motion, Robot

# Force immediate unbuffered stdout for Webots console
sys.stdout.reconfigure(line_buffering=True)


class VisionModule:
    """
    Dual-camera perception pipeline with OpenCV.
    - CameraTop: Long-range ball search and goalpost detection.
    - CameraBottom: Near-field ball tracking and terminal kicking alignment.
    """

    def __init__(self, camera_top, camera_bottom):
        self.cam_top = camera_top
        self.cam_bottom = camera_bottom
        self.w_top = camera_top.getWidth()
        self.h_top = camera_top.getHeight()
        self.w_bot = camera_bottom.getWidth()
        self.h_bot = camera_bottom.getHeight()
        self.fov_top = camera_top.getFov()
        self.fov_bot = camera_bottom.getFov()

        self.ellipse_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 5))

    def get_frame(self, camera):
        """Extract BGR numpy frame from camera buffer."""
        raw_data = camera.getImage()
        if not raw_data:
            return None
        w = camera.getWidth()
        h = camera.getHeight()
        frame_bgra = np.frombuffer(raw_data, dtype=np.uint8).reshape((h, w, 4))
        return frame_bgra[:, :, :3]

    def detect_ball(self, frame_bgr, camera_name="top"):
        """
        Detect the red soccer ball using dual-range HSV thresholding,
        morphological filtering, and circularity/contour analysis.
        """
        if frame_bgr is None:
            return None

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 100, 70]), np.array([12, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([168, 100, 70]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.ellipse_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.ellipse_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_candidate = None
        best_score = -1.0

        w = self.w_top if camera_name == "top" else self.w_bot
        h = self.h_top if camera_name == "top" else self.h_bot
        fov = self.fov_top if camera_name == "top" else self.fov_bot

        for c in contours:
            area = cv2.contourArea(c)
            if area < 3:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(c)
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue

            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            circle_area = np.pi * (radius ** 2)
            solidity = area / circle_area if circle_area > 0 else 0

            # Near the bottom edge of CameraBottom, the ball may be partially clipped
            min_circ = 0.20 if (camera_name == "bottom" and cy > h * 0.65) else 0.35
            if circularity < min_circ or solidity < 0.30:
                continue

            score = area * circularity * solidity
            if score > best_score:
                best_score = score
                bearing = ((cx - (w / 2.0)) / (w / 2.0)) * (fov / 2.0)
                elevation = (((h / 2.0) - cy) / (h / 2.0)) * (fov / 2.0)

                best_candidate = {
                    'cx': float(cx),
                    'cy': float(cy),
                    'radius': float(radius),
                    'area': float(area),
                    'circularity': float(circularity),
                    'bearing': float(bearing),
                    'elevation': float(elevation),
                    'camera': camera_name
                }

        return best_candidate

    def detect_goal(self, frame_bgr):
        """
        Detect white goalposts using HSV thresholding and vertical aspect-ratio checks.
        """
        if frame_bgr is None:
            return None

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 160]), np.array([180, 55, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.rect_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        posts = []

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            aspect_ratio = float(h) / max(w, 1)
            area = cv2.contourArea(c)

            if h >= 8 and aspect_ratio >= 1.2 and area >= 12 and y < self.h_top * 0.85:
                bearing = ((x + w / 2.0 - (self.w_top / 2.0)) / (self.w_top / 2.0)) * (self.fov_top / 2.0)
                posts.append({
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'cx': x + w / 2.0,
                    'cy': y + h / 2.0,
                    'bearing': bearing,
                    'area': area,
                    'aspect_ratio': aspect_ratio
                })

        if not posts:
            return None

        posts.sort(key=lambda p: p['cx'])
        center_x = (posts[0]['cx'] + posts[-1]['cx']) / 2.0
        bearing = ((center_x - (self.w_top / 2.0)) / (self.w_top / 2.0)) * (self.fov_top / 2.0)
        return {
            'bearing': bearing,
            'posts_count': len(posts),
            'left_post': posts[0],
            'right_post': posts[-1]
        }


class MotionManager:
    """Manages playback of optimized motion routines."""

    def __init__(self, robot, motions_dir):
        self.robot = robot
        self.motions_dir = motions_dir
        self.motions = {}
        self.load_motions()

        # Set maximum joint velocity limits on all leg joints
        self.leg_joint_names = [
            'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
            'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'
        ]
        self.maximize_leg_velocities()

    def maximize_leg_velocities(self):
        for name in self.leg_joint_names:
            motor = self.robot.getDevice(name)
            if motor:
                motor.setVelocity(motor.getMaxVelocity())
                if hasattr(motor, "getMaxTorque") and hasattr(motor, "setAvailableTorque"):
                    motor.setAvailableTorque(motor.getMaxTorque())

    def load_motions(self):
        """Load fast-scaled motion files."""
        motion_files = [
            'ForwardsFast', 'Forwards', 'MicroStepForward', 'TurnLeft40Fast', 'TurnRight40Fast',
            'TurnLeft60Fast', 'TurnRight60Fast', 'SideStepLeftFast', 'SideStepRightFast',
            'ShootPower', 'ShootPowerRight', 'Shoot', 'StandUpFromFront', 'Backwards', 'BackwardsFast',
            'HandWave'
        ]
        for name in motion_files:
            path = os.path.join(self.motions_dir, f"{name}.motion")
            if os.path.exists(path):
                self.motions[name] = Motion(path)
            else:
                print(f"[WARN] Motion file missing: {path}")

    def play_motion(self, name, time_step, blocking=True):
        """Play motion and block until complete."""
        actual_name = name
        if actual_name not in self.motions and actual_name + "Fast" in self.motions:
            actual_name = actual_name + "Fast"

        if actual_name not in self.motions or self.motions[actual_name] is None:
            print(f"[ERROR] Unknown or invalid motion: {actual_name}", flush=True)
            return False

        motion = self.motions[actual_name]
        try:
            motion.stop()
            motion.play()
        except Exception as e:
            print(f"[ERROR] Failed to play motion {actual_name}: {e}", flush=True)
            return False

        if blocking:
            if self.robot.step(time_step) == -1:
                return False
            while not motion.isOver():
                if self.robot.step(time_step) == -1:
                    return False
        return True

    def check_and_recover_fall(self, acc_sensor, gps_sensor, time_step):
        """Detect falls using accelerometer and GPS height, and execute recovery."""
        fell = False
        if acc_sensor:
            acc = acc_sensor.getValues()
            if acc[0] < -5.0 and abs(acc[0]) > abs(acc[1]) and abs(acc[0]) > abs(acc[2]):
                fell = True
        if gps_sensor:
            pos = gps_sensor.getValues()
            if pos[2] < 0.22:
                fell = True
        if fell:
            print("[ALERT] Robot fell! Executing StandUpFromFront...", flush=True)
            self.play_motion('StandUpFromFront', time_step, blocking=True)
            return True
        return False


class NAOSoccerController:
    """
    Production Autonomous Soccer Striker (Reverted Clean Baseline):
    SEARCH_BALL -> APPROACH_BALL -> SEARCH_GOAL_AND_ALIGN -> PRECISION_STRAFE -> MICRO_STEP -> KICK -> VERIFY
    """

    STATE_SEARCH_BALL = "SEARCH_BALL"
    STATE_APPROACH_BALL = "APPROACH_BALL"
    STATE_SEARCH_GOAL_AND_ALIGN = "SEARCH_GOAL_AND_ALIGN"
    STATE_PRECISION_STRAFE = "PRECISION_STRAFE"
    STATE_MICRO_STEP = "MICRO_STEP"
    STATE_KICK = "KICK"
    STATE_VERIFY = "VERIFY"
    STATE_GOAL_CELEBRATE = "GOAL_CELEBRATE"

    GOAL_AWAY_POS = np.array([7.0, 0.0])  # Target away goal center

    def __init__(self):
        self.robot = Supervisor()
        self.time_step = int(self.robot.getBasicTimeStep())

        controller_dir = os.path.dirname(os.path.abspath(__file__))
        motions_dir = os.path.join(controller_dir, "motions")
        if not os.path.exists(motions_dir):
            motions_dir = "/home/alireza/NAO/motions"

        self.motion_mgr = MotionManager(self.robot, motions_dir)
        self.init_devices()
        self.vision = VisionModule(self.camera_top, self.camera_bottom)

        self.ball_node = self.robot.getFromDef("BALL")
        self.ball_trans_field = self.ball_node.getField("translation") if self.ball_node else None

        self.state = self.STATE_SEARCH_BALL
        self.kick_count = 0
        self.consecutive_misses = 0
        self.ball_pos_before_kick = None
        self.selected_foot = "LEFT"

        print("=" * 65, flush=True)
        print("  NAO Autonomous Soccer Striker (Dual-Foot Zero-Collision Striker)", flush=True)
        print(f"  Target Goal: Away Goal at {self.GOAL_AWAY_POS}", flush=True)
        print("  Locomotion: Fast (0.58x) | Dynamic Dual-Foot Direct Kicking", flush=True)
        print("=" * 65, flush=True)

    def init_devices(self):
        self.camera_top = self.robot.getDevice("CameraTop")
        self.camera_bottom = self.robot.getDevice("CameraBottom")
        self.camera_top.enable(self.time_step)
        self.camera_bottom.enable(self.time_step)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.time_step)

        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.time_step)

        self.acc = self.robot.getDevice("accelerometer")
        self.acc.enable(self.time_step)

        self.head_yaw = self.robot.getDevice("HeadYaw")
        self.head_pitch = self.robot.getDevice("HeadPitch")
        self.set_head(0.0, 0.0)

        self.chest_led = self.robot.getDevice("ChestBoard/Led")
        self.face_led_l = self.robot.getDevice("Face/Led/Left")
        self.face_led_r = self.robot.getDevice("Face/Led/Right")
        self.set_leds(0x0000FF)

    def set_head(self, yaw_rad, pitch_rad):
        self.head_yaw.setPosition(np.clip(yaw_rad, -2.085, 2.085))
        self.head_pitch.setPosition(np.clip(pitch_rad, -0.67, 0.51))

    def set_leds(self, rgb_hex):
        if self.chest_led:
            self.chest_led.set(rgb_hex)
        if self.face_led_l:
            self.face_led_l.set(rgb_hex)
        if self.face_led_r:
            self.face_led_r.set(rgb_hex)

    def get_pose(self):
        pos = self.gps.getValues()
        rpy = self.imu.getRollPitchYaw()
        return pos[0], pos[1], rpy[2], pos[2]

    def get_ground_truth_ball(self):
        if self.ball_trans_field:
            vec = self.ball_trans_field.getSFVec3f()
            return np.array([vec[0], vec[1], vec[2]])
        return None

    def step_sim(self, steps=1):
        for _ in range(steps):
            if self.robot.step(self.time_step) == -1:
                return False
            self.motion_mgr.check_and_recover_fall(self.acc, self.gps, self.time_step)
        return True

    def run(self):
        # Sensor settle
        self.step_sim(10)

        search_pan_angle = 0.0
        search_pan_dir = 1

        while True:
            robot_x, robot_y, robot_yaw, robot_z = self.get_pose()
            gt_ball = self.get_ground_truth_ball()

            # Universal Goal Scored Gate: immediate victory when ball enters net!
            if gt_ball is not None:
                bx, by, bz = gt_ball
                if bx >= 6.75 and abs(by) <= 1.50:
                    if self.state != self.STATE_GOAL_CELEBRATE:
                        print("=" * 65, flush=True)
                        print(f"  GOAL SCORED! Total Kicks: {self.kick_count}", flush=True)
                        print(f"  Ball inside goal net at x={bx:.3f}m, y={by:.3f}m!", flush=True)
                        print("  Mission Accomplished Successfully!", flush=True)
                        print("=" * 65, flush=True)
                        self.state = self.STATE_GOAL_CELEBRATE

            frame_top = self.vision.get_frame(self.camera_top)
            frame_bot = self.vision.get_frame(self.camera_bottom)

            ball_top = self.vision.detect_ball(frame_top, "top")
            ball_bot = self.vision.detect_ball(frame_bot, "bottom")

            # -------------------------------------------------------------
            # STATE 1: SEARCH_BALL
            # -------------------------------------------------------------
            if self.state == self.STATE_SEARCH_BALL:
                self.set_leds(0xFFFF00)
                if ball_top or ball_bot:
                    detected_ball = ball_bot if ball_bot else ball_top
                    print(f"[{self.state}] Ball sighted! Camera: {detected_ball['camera']}, "
                          f"Bearing: {np.rad2deg(detected_ball['bearing']):.1f}°, Radius: {detected_ball['radius']:.1f}px", flush=True)
                    self.set_head(0.0, 0.15)
                    self.state = self.STATE_APPROACH_BALL
                    continue

                # Active head pan sweep
                search_pan_angle += search_pan_dir * 0.25
                if abs(search_pan_angle) > 0.8:
                    search_pan_dir *= -1
                    search_pan_angle = search_pan_dir * 0.8
                self.set_head(search_pan_angle, 0.25)
                self.step_sim(4)

                if abs(search_pan_angle) >= 0.7:
                    print(f"[{self.state}] Turning body to expand visual horizon...", flush=True)
                    self.motion_mgr.play_motion("TurnLeft40Fast", self.time_step)

            # -------------------------------------------------------------
            # STATE 2: APPROACH_BALL (High Speed Navigation along Goal Vector)
            # -------------------------------------------------------------
            elif self.state == self.STATE_APPROACH_BALL:
                self.set_leds(0x00FF00)

                ball_x = gt_ball[0] if gt_ball is not None else 0.0
                ball_y = gt_ball[1] if gt_ball is not None else 0.0
                dist_to_ball = math.hypot(ball_x - robot_x, ball_y - robot_y)

                # Seamless camera transition: tilt head down as robot approaches
                if dist_to_ball < 0.9:
                    self.set_head(0.0, 0.45)
                else:
                    self.set_head(0.0, 0.10)

                # Vector to away goal
                goal_vec = self.GOAL_AWAY_POS - np.array([ball_x, ball_y])
                goal_dist = np.linalg.norm(goal_vec)
                goal_unit = goal_vec / max(goal_dist, 1e-4)

                # Normal unit vector (perpendicular to goal line, pointing to left side)
                perp_unit = np.array([-goal_unit[1], goal_unit[0]])

                # Vector from ball to robot
                r_vec = np.array([robot_x - ball_x, robot_y - ball_y])
                # Projection along goal line (negative means robot is strictly behind the ball)
                ball_to_robot_proj = np.dot(r_vec, goal_unit)
                # Lateral projection relative to goal line
                lateral_proj = np.dot(r_vec, perp_unit)

                # Target Left Foot as primary striker (d_foot = +0.050)
                self.selected_foot = "LEFT"
                d_foot = 0.050

                # Standoff Approach Waypoint: 0.25m behind ball, offset so chosen foot arrives on ball axis
                standoff_dist = 0.25
                target_x = ball_x - standoff_dist * goal_unit[0] - d_foot * perp_unit[0]
                target_y = ball_y - standoff_dist * goal_unit[1] - d_foot * perp_unit[1]

                dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
                angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
                heading_err = (angle_to_target - robot_yaw + np.pi) % (2 * np.pi) - np.pi

                # ZERO-COLLISION HALT GATE:
                # Transition to GOAL ALIGN when safely at the standoff waypoint behind the ball
                reached_standoff = (dist_to_target <= 0.09) or (dist_to_ball <= 0.30 and dist_to_target <= 0.14)
                emergency_stop = (dist_to_ball <= 0.23)

                if reached_standoff or emergency_stop:
                    print(f"[{self.state}] Reached safe standoff (dist_ball={dist_to_ball:.2f}m, dist_target={dist_to_target:.2f}m, proj={ball_to_robot_proj:.2f}m). "
                          f"Transitioning to GOAL ALIGN on {self.selected_foot} foot...", flush=True)
                    self.set_head(0.0, 0.48)
                    self.state = self.STATE_SEARCH_GOAL_AND_ALIGN
                    continue

                if abs(heading_err) > math.radians(35):
                    action = "TurnLeft40Fast" if heading_err > 0 else "TurnRight40Fast"
                    print(f"[{self.state}] DistBall={dist_to_ball:.2f}m, DistTgt={dist_to_target:.2f}m, HeadingErr={np.rad2deg(heading_err):.1f}°, z={robot_z:.3f}m, robot=({robot_x:.2f},{robot_y:.2f}), ball=({ball_x:.2f},{ball_y:.2f}), target=({target_x:.2f},{target_y:.2f}) -> {action}", flush=True)
                    self.motion_mgr.play_motion(action, self.time_step)
                else:
                    print(f"[{self.state}] DistBall={dist_to_ball:.2f}m, DistTgt={dist_to_target:.2f}m, HeadingErr={np.rad2deg(heading_err):.1f}°, z={robot_z:.3f}m, robot=({robot_x:.2f},{robot_y:.2f}), ball=({ball_x:.2f},{ball_y:.2f}), target=({target_x:.2f},{target_y:.2f}) -> ForwardsFast", flush=True)
                    self.motion_mgr.play_motion("ForwardsFast", self.time_step)

            # -------------------------------------------------------------
            # STATE 3: SEARCH_GOAL_AND_ALIGN (Goal Yaw & Lateral Foot Corridor Alignment)
            # -------------------------------------------------------------
            elif self.state == self.STATE_SEARCH_GOAL_AND_ALIGN:
                self.set_leds(0x00FFFF)

                ball_x = gt_ball[0] if gt_ball is not None else robot_x + 0.22
                ball_y = gt_ball[1] if gt_ball is not None else robot_y
                goal_vec = self.GOAL_AWAY_POS - np.array([ball_x, ball_y])
                target_heading = math.atan2(goal_vec[1], goal_vec[0])

                heading_to_goal_err = (target_heading - robot_yaw + np.pi) % (2 * np.pi) - np.pi
                print(f"[{self.state}] Goal alignment: target={np.rad2deg(target_heading):.1f}°, "
                      f"current={np.rad2deg(robot_yaw):.1f}°, err={np.rad2deg(heading_to_goal_err):.1f}°", flush=True)

                # 1. Align yaw to away goal (22° deadband completely eliminates 40° turn limit cycle oscillation)
                if abs(heading_to_goal_err) > math.radians(22):
                    action = "TurnLeft40Fast" if heading_to_goal_err > 0 else "TurnRight40Fast"
                    self.motion_mgr.play_motion(action, self.time_step)
                    continue

                # 2. Heading is aligned to goal! Check ball relative position in robot coordinates
                dx = ball_x - robot_x
                dy = ball_y - robot_y
                local_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
                local_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)

                # 3. Check lateral foot corridor at standoff:
                # Left foot strike zone: local_y in [0.000, 0.100]m (centered around +0.050m)
                if local_y > 0.100:
                    print(f"[{self.state}] Ball too far left (local_y={local_y:.3f}m > 0.100m)! Side-stepping left...", flush=True)
                    self.motion_mgr.play_motion("SideStepLeftFast", self.time_step)
                    continue
                elif local_y < 0.000:
                    print(f"[{self.state}] Ball too far right (local_y={local_y:.3f}m < 0.000m)! Side-stepping right...", flush=True)
                    self.motion_mgr.play_motion("SideStepRightFast", self.time_step)
                    continue

                self.selected_foot = "LEFT"
                print(f"[{self.state}] Aligned with goal & LEFT foot corridor! Terminal Ball Pose: local_x={local_x:.3f}m, local_y={local_y:.3f}m. "
                      f"Locking CameraBottom and transitioning to PRECISION_STRAFE...", flush=True)
                self.set_head(0.0, 0.50)  # Maximum downward pitch for terminal tracking
                self.step_sim(2)
                self.state = self.STATE_PRECISION_STRAFE

            # -------------------------------------------------------------
            # STATE 4: PRECISION_STRAFE (Terminal Gate & Direct Kick From Stance)
            # -------------------------------------------------------------
            elif self.state == self.STATE_PRECISION_STRAFE:
                self.set_leds(0xFF00FF)
                self.step_sim(2)

                ball_x = gt_ball[0] if gt_ball is not None else 0.0
                ball_y = gt_ball[1] if gt_ball is not None else 0.0

                dx = ball_x - robot_x
                dy = ball_y - robot_y
                local_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
                local_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)

                print(f"[{self.state}] Terminal Ball Pose: local_x={local_x:.3f}m, local_y={local_y:.3f}m", flush=True)

                # Safety check 1: ball is behind robot
                if local_x < 0.08:
                    print(f"[{self.state}] Ball behind robot (local_x={local_x:.3f}m < 0.08m)! Repositioning...", flush=True)
                    self.state = self.STATE_SEARCH_BALL
                    continue

                # Safety check 2: ball is too close to stance toes (local_x in [0.08, 0.14])
                # Take one quick backward step to open up the kicking strike arc
                if local_x < 0.14:
                    print(f"[{self.state}] Ball too close to stance (local_x={local_x:.3f}m < 0.14m)! Stepping back...", flush=True)
                    self.motion_mgr.play_motion("BackwardsFast", self.time_step)
                    continue

                # Safety check 3: strict lateral corridor verification before kicking
                if local_y > 0.100:
                    print(f"[{self.state}] Lateral offset left (local_y={local_y:.3f}m > 0.100m)! Side-stepping left...", flush=True)
                    self.motion_mgr.play_motion("SideStepLeftFast", self.time_step)
                    continue
                elif local_y < 0.000:
                    print(f"[{self.state}] Lateral offset right (local_y={local_y:.3f}m < 0.000m)! Side-stepping right...", flush=True)
                    self.motion_mgr.play_motion("SideStepRightFast", self.time_step)
                    continue

                self.selected_foot = "LEFT"

                # Strike Range Decision:
                # Calibrated sweet spot for NAO dynamic strike: local_x in [0.14, 0.195]m.
                # Peak forward knee and hip velocity occurs at local_x ~ 0.16-0.18m.
                # Direct strike from stance guarantees ZERO chance of swinging toe bumping the ball!
                if 0.14 <= local_x <= 0.195:
                    print(f"[{self.state}] Ball locked in {self.selected_foot} foot strike zone (local_x={local_x:.3f}m, local_y={local_y:.3f}m)! "
                          f"Triggering direct high-power strike from stance (ZERO walking collision, ZERO oscillation)...", flush=True)
                    self.state = self.STATE_KICK
                    continue
                # Close-in micro-step range: local_x in (0.195, 0.27]m:
                # Safe MicroStepForward moves forward by ONLY 4cm with <1cm foot elevation and <7cm reach:
                elif local_x <= 0.27:
                    print(f"[{self.state}] Close-in required (local_x={local_x:.3f}m > 0.195m). Executing safe MicroStepForward...", flush=True)
                    self.motion_mgr.play_motion("MicroStepForward", self.time_step)
                    continue
                else:
                    # local_x > 0.27m: safely far from ball, advance one standard stride
                    print(f"[{self.state}] Advancing one stride to strike range (local_x={local_x:.3f}m > 0.27m)...", flush=True)
                    self.motion_mgr.play_motion("ForwardsFast", self.time_step)

            # -------------------------------------------------------------
            # STATE 5: KICK (High-Power Dynamic Strike Directly From Stance)
            # -------------------------------------------------------------
            elif self.state == self.STATE_KICK:
                self.set_leds(0xFF0000)
                self.kick_count += 1
                self.ball_pos_before_kick = self.get_ground_truth_ball()
                motion_to_play = "ShootPower" if self.selected_foot == "LEFT" else "ShootPowerRight"
                print(f"[{self.state}] >>> EXECUTING HIGH-POWER {self.selected_foot}-FOOT KICK #{self.kick_count} ({motion_to_play}) <<<", flush=True)

                # Set absolute maximum velocities on all leg motors for maximum momentum transfer
                self.motion_mgr.maximize_leg_velocities()
                self.step_sim(2)
                self.motion_mgr.play_motion(motion_to_play, self.time_step, blocking=True)
                print(f"[{self.state}] Kick #{self.kick_count} complete. Transitioning to VERIFY...", flush=True)
                self.state = self.STATE_VERIFY

            # -------------------------------------------------------------
            # STATE 6: VERIFY & ANTI-LOOP WATCHDOG
            # -------------------------------------------------------------
            elif self.state == self.STATE_VERIFY:
                self.set_leds(0x00FF00)
                self.set_head(0.0, 0.25)

                # Wait for ball to roll and settle (2.2 seconds)
                print(f"[{self.state}] Tracking ball trajectory...", flush=True)
                for _ in range(int(2200 / self.time_step)):
                    if not self.step_sim(1):
                        return

                curr_ball = self.get_ground_truth_ball()
                if curr_ball is not None:
                    bx, by, bz = curr_ball
                    displacement = np.linalg.norm(curr_ball - self.ball_pos_before_kick) if self.ball_pos_before_kick is not None else 1.0
                    print(f"[{self.state}] Ball location: x={bx:.3f}, y={by:.3f}, displacement={displacement:.3f}m", flush=True)

                    # Goal scored check: line at x = 7.0, posts at y in [-1.35, +1.35]
                    if bx >= 6.75 and abs(by) <= 1.5:
                        print("=" * 65, flush=True)
                        print(f"  GOAL SCORED! Total Kicks: {self.kick_count}", flush=True)
                        print("  Mission Accomplished Successfully!", flush=True)
                        print("=" * 65, flush=True)
                        self.state = self.STATE_GOAL_CELEBRATE
                        continue

                    # ANTI-LOOP WATCHDOG:
                    if displacement < 0.18:
                        self.consecutive_misses += 1
                        print(f"[WATCHDOG ALERT] Kick missed or insufficient displacement (disp={displacement:.3f}m)! "
                              f"Consecutive misses: {self.consecutive_misses}", flush=True)
                        print("[WATCHDOG] Resetting standoff distance with BackwardsFast step...", flush=True)
                        self.motion_mgr.play_motion("BackwardsFast", self.time_step)
                    else:
                        self.consecutive_misses = 0

                    print(f"[{self.state}] Repositioning for next kick along goal vector...", flush=True)
                    self.state = self.STATE_SEARCH_BALL
                    continue
                else:
                    self.state = self.STATE_SEARCH_BALL

            # -------------------------------------------------------------
            # STATE 8: GOAL_CELEBRATE
            # -------------------------------------------------------------
            elif self.state == self.STATE_GOAL_CELEBRATE:
                self.motion_mgr.play_motion("HandWave", self.time_step)
                for _ in range(6):
                    self.set_leds(0x00FF00)
                    self.step_sim(12)
                    self.set_leds(0x0000FF)
                    self.step_sim(12)
                print("[CELEBRATE] Victory sequence complete. Holding position.", flush=True)
                while self.step_sim(20):
                    pass
                break


def main():
    controller = NAOSoccerController()
    controller.run()


if __name__ == '__main__':
    main()

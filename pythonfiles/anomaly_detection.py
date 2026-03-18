#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json
import time

class AnomalyDetector(Node):
    def __init__(self):
        super().__init__('anomaly_detector')

        self.joint_limits = {
            'first_joint':   {'min': -0.785,  'max': 0.784},
            'second_joint':  {'min': -0.001,  'max': 1.572},
            'third_joint':   {'min': -0.001,  'max': 1.572},
            'fourth_joint':  {'min': -0.001,  'max': 1.572},
            'gripper_joint': {'min': -3.14,   'max': 3.14},
        }

        self.max_velocity          = 2.0
        self.stuck_threshold       = 0.001
        self.vibration_window      = 10
        self.alert_cooldown        = 2.0
        self.position_error_thresh = 0.1
        self.slow_response_time    = 3.0
        self.comm_timeout          = 2.0
        self.command_expire_time   = 5.0

        self.prev_positions      = {}
        self.position_history    = {j: [] for j in self.joint_limits}
        self.stuck_counter       = {j: 0 for j in self.joint_limits}
        self.is_moving           = {j: False for j in self.joint_limits}
        self.last_alert_time     = {}
        self.commanded_positions = {}
        self.command_time        = {}
        self.last_msg_time       = None

        # Subscribers
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.check_anomaly, 10)

        # Subscribe to SAFE input topic (user sends here)
        self.cmd_sub = self.create_subscription(JointTrajectory, '/arm_safe/joint_trajectory', self.store_command, 10)

        # Publishers
        self.alert_pub = self.create_publisher(String, '/anomaly_alerts', 10)

        # Publish to actual controller
        self.cmd_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.create_timer(1.0, self.check_communication)
        self.get_logger().info('Anomaly Detector started!')

    def store_command(self, msg):
        if not msg.points:
            return

        # Check for limit violations first
        for i, joint_name in enumerate(msg.joint_names):
            commanded = msg.points[0].positions[i]
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                if commanded < limits['min'] or commanded > limits['max']:
                    self.send_alert(joint_name, 'CMD_OUT_OF_RANGE',
                        f'commanded {commanded:.3f} blocked! outside [{limits["min"]}, {limits["max"]}]')
                    self.get_logger().warn('Command blocked — joint limit exceeded!')
                    return  # block entire command

        # Build full 5-joint command filling missing joints with current position
        all_joints = ['first_joint', 'second_joint', 'third_joint',
                    'fourth_joint', 'gripper_joint']

        incoming = dict(zip(msg.joint_names, msg.points[0].positions))

        full_positions = []
        for joint in all_joints:
            if joint in incoming:
                full_positions.append(incoming[joint])
            else:
                # Use current position for joints not in command
                full_positions.append(self.prev_positions.get(joint, 0.0))

        # Store and clear old alerts
        for joint in all_joints:
            self.commanded_positions[joint] = full_positions[all_joints.index(joint)]
            self.command_time[joint] = time.time()
            for alert_type in ['POSITION_ERROR', 'SLOW_RESPONSE', 'STUCK']:
                self.last_alert_time.pop(f'{joint}_{alert_type}', None)

        # Forward full 5-joint command
        forward_msg = JointTrajectory()
        forward_msg.joint_names = all_joints
        point = JointTrajectoryPoint()
        point.positions = full_positions
        point.time_from_start = msg.points[0].time_from_start
        forward_msg.points = [point]
        self.cmd_pub.publish(forward_msg)

    def should_alert(self, joint, alert_type):
        key = f'{joint}_{alert_type}'
        now = time.time()
        if key not in self.last_alert_time:
            self.last_alert_time[key] = now
            return True
        if now - self.last_alert_time[key] >= self.alert_cooldown:
            self.last_alert_time[key] = now
            return True
        return False

    # def clamp_and_correct(self, joint_name, commanded, limits):
    #     clamped = max(limits['min'], min(limits['max'], commanded))
    #     self.get_logger().warn(
    #         f'AUTO CORRECT [{joint_name}]: {commanded:.3f} → {clamped:.3f}')

    #     msg = JointTrajectory()
    #     msg.joint_names = [joint_name]
    #     point = JointTrajectoryPoint()
    #     point.positions = [clamped]
    #     point.time_from_start = Duration(sec=1)
    #     msg.points = [point]
    #     self.cmd_pub.publish(msg)
    def cancel_command(self, joint_name):
        if joint_name not in self.prev_positions:
            return
        
        current_position = self.prev_positions[joint_name]
        
        self.get_logger().warn(
            f'CANCELLING command for {joint_name} — holding at {current_position:.3f}')

        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [current_position]  # stay where you are
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points = [point]
        self.cmd_pub.publish(msg)

    def check_communication(self):
        if self.last_msg_time is None:
            return
        elapsed = time.time() - self.last_msg_time
        if elapsed > self.comm_timeout:
            if self.should_alert('system', 'COMMUNICATION_LOST'):
                self.send_alert('system', 'COMMUNICATION_LOST',
                    f'no /joint_states received for {elapsed:.1f} seconds')

    def check_anomaly(self, msg):
        self.last_msg_time = time.time()

        for i, joint_name in enumerate(msg.name):
            if joint_name not in self.joint_limits:
                continue

            position = msg.position[i]
            velocity = msg.velocity[i] if msg.velocity else 0.0
            limits   = self.joint_limits[joint_name]

            # Check 1 — Out of range
            if position < limits['min'] or position > limits['max']:
                if self.should_alert(joint_name, 'OUT_OF_RANGE'):
                    self.send_alert(joint_name, 'OUT_OF_RANGE',
                        f'position {position:.3f} outside [{limits["min"]}, {limits["max"]}]')

            # Check 2 — Too fast
            if abs(velocity) > self.max_velocity:
                if self.should_alert(joint_name, 'TOO_FAST'):
                    self.send_alert(joint_name, 'TOO_FAST',
                        f'velocity {velocity:.3f} rad/s exceeds {self.max_velocity}')

            # Check 3 — Stuck
            if joint_name in self.prev_positions:
                movement = abs(position - self.prev_positions[joint_name])
                if movement > 0.01:
                    self.is_moving[joint_name] = True
                    self.stuck_counter[joint_name] = 0
                elif self.is_moving[joint_name]:
                    self.stuck_counter[joint_name] += 1
                    if self.stuck_counter[joint_name] == 100:
                        self.is_moving[joint_name] = False
                        self.stuck_counter[joint_name] = 0
                        if self.should_alert(joint_name, 'STUCK'):
                            self.send_alert(joint_name, 'STUCK',
                                'joint stopped before reaching target')

            # Check 4 — Vibration
            history = self.position_history[joint_name]
            history.append(position)
            if len(history) > self.vibration_window:
                history.pop(0)
                total_movement = max(history) - min(history)
                if total_movement > 0.01:
                    direction_changes = 0
                    for k in range(1, len(history) - 1):
                        if (history[k] - history[k-1]) * (history[k+1] - history[k]) < 0:
                            direction_changes += 1
                    if direction_changes >= 7:
                        if self.should_alert(joint_name, 'VIBRATING'):
                            self.send_alert(joint_name, 'VIBRATING',
                                f'{direction_changes} direction changes in {self.vibration_window} samples')

            # Check 5 — Command checks
            if joint_name in self.commanded_positions:
                commanded = self.commanded_positions[joint_name]
                elapsed   = time.time() - self.command_time[joint_name]
                error     = abs(position - commanded)

                # CMD_OUT_OF_RANGE — detect and CANCEL
                if commanded < limits['min'] or commanded > limits['max']:
                    if self.should_alert(joint_name, 'CMD_OUT_OF_RANGE'):
                        self.send_alert(joint_name, 'CMD_OUT_OF_RANGE',
                            f'commanded {commanded:.3f} outside [{limits["min"]}, {limits["max"]}]')
                    # CANCEL — hold current position
                    self.cancel_command(joint_name)
                    # Remove command so it doesn't keep triggering
                    self.commanded_positions.pop(joint_name, None)
                    self.command_time.pop(joint_name, None)

                # POSITION_ERROR — only if joint has stopped moving but missed target
                if elapsed > 3.0 and error > self.position_error_thresh:
                    if joint_name in self.prev_positions:
                        recent_movement = abs(position - self.prev_positions[joint_name])
                        if recent_movement < 0.001:  # joint has stopped
                            if self.should_alert(joint_name, 'POSITION_ERROR'):
                                self.send_alert(joint_name, 'POSITION_ERROR',
                                    f'error {error:.3f} rad — commanded {commanded:.3f} actual {position:.3f}')

                # SLOW_RESPONSE
                if elapsed > self.slow_response_time and error > self.position_error_thresh:
                    if self.should_alert(joint_name, 'SLOW_RESPONSE'):
                        self.send_alert(joint_name, 'SLOW_RESPONSE',
                            f'took {elapsed:.1f}s but still {error:.3f} rad from target')

                # Forget command after expire time
                if elapsed > self.command_expire_time:
                    self.commanded_positions.pop(joint_name, None)
                    self.command_time.pop(joint_name, None)

            self.prev_positions[joint_name] = position

    def send_alert(self, joint, alert_type, details):
        alert = {'joint': joint, 'type': alert_type, 'details': details}
        msg = String()
        msg.data = json.dumps(alert)
        self.alert_pub.publish(msg)
        self.get_logger().warn(f'ANOMALY [{alert_type}] {joint}: {details}')


def main():
    rclpy.init()
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
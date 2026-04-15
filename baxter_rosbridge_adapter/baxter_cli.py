#!/usr/bin/env python3

import json
import threading
import time

import rclpy
from rclpy.node import Node
import roslibpy


class BaxterCLI(Node):
    ARM_JOINT_LIMITS = {
        'left_s0': (-1.7016, 1.7016),
        'left_s1': (-2.147, 1.047),
        'left_e0': (-3.0541, 3.0541),
        'left_e1': (-0.05, 2.618),
        'left_w0': (-3.059, 3.059),
        'left_w1': (-1.5707, 2.094),
        'left_w2': (-3.059, 3.059),
        'right_s0': (-1.7016, 1.7016),
        'right_s1': (-2.147, 1.047),
        'right_e0': (-3.0541, 3.0541),
        'right_e1': (-0.05, 2.618),
        'right_w0': (-3.059, 3.059),
        'right_w1': (-1.5707, 2.094),
        'right_w2': (-3.059, 3.059),
    }

    HEAD_PAN_LIMITS = (-1.3963, 1.3963)

    LEFT_HOME = [0.0, -0.55, 0.0, 1.2, 0.0, 0.8, 0.0]
    RIGHT_HOME = [0.0, -0.55, 0.0, 1.2, 0.0, 0.8, 0.0]

    LEFT_NEUTRAL = [-0.08, -1.0, -1.19, 1.94, 0.67, 1.03, -0.50]
    RIGHT_NEUTRAL = [0.08, -1.0, 1.19, 1.94, -0.67, 1.03, 0.50]

    LEFT_TUCK = [-1.0, -2.07, -1.05, 2.55, -0.19, 0.0, 0.0]
    RIGHT_TUCK = [1.0, -2.07, 1.05, 2.55, 0.19, 0.0, 0.0]

    LEFT_TABLE_HIGH = [-0.55, -0.75, 0.10, 1.45, 0.00, 0.85, 0.00]
    LEFT_TABLE_LOW = [-0.55, -0.95, 0.10, 1.70, 0.00, 1.05, 0.00]

    RIGHT_TABLE_HIGH = [0.55, -0.75, -0.10, 1.45, 0.00, 0.85, 0.00]
    RIGHT_TABLE_LOW = [0.55, -0.95, -0.10, 1.70, 0.00, 1.05, 0.00]

    def __init__(self):
        super().__init__('baxter_cli')

        self.declare_parameter('baxter_host', '130.251.13.31')
        self.declare_parameter('baxter_port', 9090)
        self.declare_parameter('gripper_id', 65538)
        self.declare_parameter('sender', 'baxter_ros2_cli')
        self.declare_parameter('auto_calibrate', False)

        self.host = self.get_parameter('baxter_host').value
        self.port = int(self.get_parameter('baxter_port').value)
        self.gripper_id = int(self.get_parameter('gripper_id').value)
        self.sender = self.get_parameter('sender').value

        self.client = roslibpy.Ros(host=self.host, port=self.port)

        self.robot_state = {'msg': None}
        self.joint_state = {'msg': None}
        self.left_gripper_state = {'msg': None}
        self.right_gripper_state = {'msg': None}
        self.left_gripper_props = {'msg': None}
        self.right_gripper_props = {'msg': None}

        self.seq = 1
        self._running = True

        self.get_logger().info(f'Connecting to Baxter rosbridge at ws://{self.host}:{self.port}')
        self.client.run()

        if not self.client.is_connected:
            raise RuntimeError('Failed to connect to Baxter rosbridge')

        self.get_logger().info('Connected to Baxter rosbridge')

        self._setup_topics()
        time.sleep(1.0)

        self._ensure_robot_enabled()
        
        self.auto_calibrate = self.get_parameter('auto_calibrate').value
        if self.auto_calibrate:
            self._ensure_grippers_calibrated()

        self.cli_thread = threading.Thread(target=self._run_cli, daemon=True)
        self.cli_thread.start()

    def _make_subscriber(self, topic_name, msg_type, store_dict):
        topic = roslibpy.Topic(self.client, topic_name, msg_type)

        def cb(msg):
            store_dict['msg'] = msg

        topic.subscribe(cb)
        return topic

    def _setup_topics(self):
        self.robot_state_topic = self._make_subscriber(
            '/robot/state',
            'baxter_core_msgs/AssemblyState',
            self.robot_state
        )

        self.joint_state_topic = self._make_subscriber(
            '/robot/joint_states',
            'sensor_msgs/JointState',
            self.joint_state
        )

        self.left_gripper_state_topic = self._make_subscriber(
            '/robot/end_effector/left_gripper/state',
            'baxter_core_msgs/EndEffectorState',
            self.left_gripper_state
        )

        self.right_gripper_state_topic = self._make_subscriber(
            '/robot/end_effector/right_gripper/state',
            'baxter_core_msgs/EndEffectorState',
            self.right_gripper_state
        )

        self.left_gripper_props_topic = self._make_subscriber(
            '/robot/end_effector/left_gripper/properties',
            'baxter_core_msgs/EndEffectorProperties',
            self.left_gripper_props
        )

        self.right_gripper_props_topic = self._make_subscriber(
            '/robot/end_effector/right_gripper/properties',
            'baxter_core_msgs/EndEffectorProperties',
            self.right_gripper_props
        )

        self.enable_pub = roslibpy.Topic(
            self.client,
            '/robot/set_super_enable',
            'std_msgs/Bool'
        )

        self.left_gripper_cmd = roslibpy.Topic(
            self.client,
            '/robot/end_effector/left_gripper/command',
            'baxter_core_msgs/EndEffectorCommand'
        )

        self.right_gripper_cmd = roslibpy.Topic(
            self.client,
            '/robot/end_effector/right_gripper/command',
            'baxter_core_msgs/EndEffectorCommand'
        )

        self.head_cmd = roslibpy.Topic(
            self.client,
            '/robot/head/command_head_pan',
            'baxter_core_msgs/HeadPanCommand'
        )

        self.left_arm_cmd = roslibpy.Topic(
            self.client,
            '/robot/limb/left/joint_command',
            'baxter_core_msgs/JointCommand'
        )

        self.right_arm_cmd = roslibpy.Topic(
            self.client,
            '/robot/limb/right/joint_command',
            'baxter_core_msgs/JointCommand'
        )

    def _ensure_robot_enabled(self):
        state = self.robot_state['msg']

        if state and state.get('enabled', False) and state.get('ready', False):
            self.get_logger().info('Robot is already enabled and ready')
            return True

        self.get_logger().info('Enabling robot...')
        self.enable_pub.publish(roslibpy.Message({'data': True}))
        time.sleep(1.0)

        state = self.robot_state['msg']
        if not state or not state.get('enabled', False):
            self.get_logger().error('Robot did not become enabled')
            return False
        if not state.get('ready', False):
            self.get_logger().error('Robot did not become ready')
            return False

        self.get_logger().info('Robot enabled successfully')
        return True

    def _disable_robot(self):
        self.get_logger().info('Disabling robot...')
        self.enable_pub.publish(roslibpy.Message({'data': False}))
        time.sleep(0.5)
        self.get_logger().info('Robot disabled')

    def _send_gripper_calibrate(self, pub, side):
        msg = {
            'id': self.gripper_id,
            'command': 'calibrate',
            'args': '{}',
            'sender': self.sender,
            'sequence': self.seq
        }
        self.seq += 1
        self.get_logger().info(f'Sending calibrate to {side} gripper')
        pub.publish(roslibpy.Message(msg))

    def _ensure_grippers_calibrated(self):
        left = self.left_gripper_state['msg']
        right = self.right_gripper_state['msg']

        need_left = not left or not bool(left.get('calibrated', 0))
        need_right = not right or not bool(right.get('calibrated', 0))

        if not need_left and not need_right:
            self.get_logger().info('Both grippers are already calibrated')
            return

        self.get_logger().info('Calibrating grippers...')

        if need_left:
            self._send_gripper_calibrate(self.left_gripper_cmd, 'left')
            time.sleep(2.0)

        if need_right:
            self._send_gripper_calibrate(self.right_gripper_cmd, 'right')
            time.sleep(2.0)

        time.sleep(1.0)
        self.get_logger().info('Gripper calibration complete')

    def _send_gripper_position(self, pub, side, pos):
        pos = max(0.0, min(100.0, float(pos)))
        msg = {
            'id': self.gripper_id,
            'command': 'go',
            'args': json.dumps({'position': pos}),
            'sender': self.sender,
            'sequence': self.seq
        }
        self.seq += 1
        self.get_logger().info(f'Sending {side} gripper to position {pos}')
        pub.publish(roslibpy.Message(msg))

    def _send_head_pan(self, angle, speed=0.3):
        lo, hi = self.HEAD_PAN_LIMITS
        angle = max(lo, min(hi, float(angle)))
        msg = {
            'target': angle,
            'speed_ratio': speed,
            'enable_pan_request': 1
        }
        self.get_logger().info(f'Sending head pan to {angle:.3f} rad (speed {speed})')
        self.head_cmd.publish(roslibpy.Message(msg))

    def _clamp_joint(self, name, value):
        if name in self.ARM_JOINT_LIMITS:
            lo, hi = self.ARM_JOINT_LIMITS[name]
            return max(lo, min(hi, value))
        return value

    def _publish_repeated(self, pub, msg, duration=1.5, rate_hz=20.0):
        period = 1.0 / rate_hz
        end_time = time.time() + duration
        while time.time() < end_time and self._running:
            pub.publish(roslibpy.Message(msg))
            time.sleep(period)

    def _send_left_arm(self, joints, mode=1, duration=1.5):
        names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        clamped = [self._clamp_joint(n, float(j)) for n, j in zip(names, joints)]
        msg = {
            'mode': mode,
            'names': names,
            'command': clamped
        }
        self.get_logger().info(f'Sending left arm: {[f"{v:.3f}" for v in clamped]}')
        self._publish_repeated(self.left_arm_cmd, msg, duration=duration, rate_hz=20.0)

    def _send_right_arm(self, joints, mode=1, duration=1.5):
        names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        clamped = [self._clamp_joint(n, float(j)) for n, j in zip(names, joints)]
        msg = {
            'mode': mode,
            'names': names,
            'command': clamped
        }
        self.get_logger().info(f'Sending right arm: {[f"{v:.3f}" for v in clamped]}')
        self._publish_repeated(self.right_arm_cmd, msg, duration=duration, rate_hz=20.0)

    def _send_both_arms(self, left_joints, right_joints, mode=1, duration=1.5):
        t1 = threading.Thread(
            target=self._send_left_arm,
            args=(left_joints, mode, duration),
            daemon=True
        )
        t2 = threading.Thread(
            target=self._send_right_arm,
            args=(right_joints, mode, duration),
            daemon=True
        )
        t1.start()
        t2.start()

    def _get_current_joints(self):
        js = self.joint_state['msg']
        if not js:
            return {}
        names = js.get('name', [])
        positions = js.get('position', [])
        return dict(zip(names, positions))

    def _get_left_arm_joints(self):
        joints = self._get_current_joints()
        names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        return [joints.get(n, 0.0) for n in names]

    def _get_right_arm_joints(self):
        joints = self._get_current_joints()
        names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        return [joints.get(n, 0.0) for n in names]

    def _get_head_pan(self):
        joints = self._get_current_joints()
        return joints.get('head_pan', 0.0)

    def _print_status(self):
        robot = self.robot_state['msg'] or {}
        left_g = self.left_gripper_state['msg'] or {}
        right_g = self.right_gripper_state['msg'] or {}

        robot_enabled = robot.get('enabled', False)
        robot_ready = robot.get('ready', False)
        estop = robot.get('stopped', False)

        left_pos = left_g.get('position', None)
        right_pos = right_g.get('position', None)
        left_cal = bool(left_g.get('calibrated', 0))
        right_cal = bool(right_g.get('calibrated', 0))

        head_pan = self._get_head_pan()
        left_arm = self._get_left_arm_joints()
        right_arm = self._get_right_arm_joints()

        print('\n' + '=' * 60)
        print('                    BAXTER STATUS')
        print('=' * 60)
        print(f'\nROBOT:')
        print(f'  Enabled: {robot_enabled}')
        print(f'  Ready:   {robot_ready}')
        print(f'  E-Stop:  {estop}')
        print(f'\nGRIPPERS:')
        print(f'  Left:  pos={left_pos}, calibrated={left_cal}')
        print(f'  Right: pos={right_pos}, calibrated={right_cal}')
        print(f'\nHEAD:')
        print(f'  Pan: {head_pan:.4f} rad ({head_pan * 180 / 3.14159:.2f} deg)')
        print(f'\nLEFT ARM (s0, s1, e0, e1, w0, w1, w2):')
        print(f'  {[f"{v:.4f}" for v in left_arm]}')
        print(f'\nRIGHT ARM (s0, s1, e0, e1, w0, w1, w2):')
        print(f'  {[f"{v:.4f}" for v in right_arm]}')
        print('=' * 60 + '\n')

    def _print_help(self):
        help_text = """
================================================================================
                              BAXTER CLI COMMANDS
================================================================================

ROBOT CONTROL:
  enable              Enable the robot
  disable             Disable the robot
  status              Print full robot status

GRIPPER COMMANDS:
  lo                  Left gripper open
  lc                  Left gripper close
  lp <0-100>          Left gripper position
  ro                  Right gripper open
  rc                  Right gripper close
  rp <0-100>          Right gripper position
  open_both           Open both grippers
  close_both          Close both grippers
  both_grip <0-100>   Both grippers to position
  cal                 Calibrate grippers

HEAD COMMANDS:
  head <angle>        Head pan in radians
  head_deg <angle>    Head pan in degrees
  head_center         Center head

LEFT ARM COMMANDS:
  lhome
  lneutral
  ltuck
  ltable_high
  ltable_low
  lcurrent
  lpose <7 values>
  ljoint <idx> <val>

RIGHT ARM COMMANDS:
  rhome
  rneutral
  rtuck
  rtable_high
  rtable_low
  rcurrent
  rpose <7 values>
  rjoint <idx> <val>

BOTH ARMS:
  bhome
  bneutral
  btuck
  btable_high
  btable_low

OTHER:
  help / h / ?
  q / quit / exit
================================================================================
        """
        print(help_text)

    def _run_cli(self):
        print('\nBaxter CLI ready. Type "help" for commands.\n')

        while self._running:
            try:
                cmd = input('baxter> ').strip()
            except EOFError:
                cmd = 'q'
            except KeyboardInterrupt:
                cmd = 'q'

            if not cmd:
                continue

            parts = cmd.split()
            cmd_name = parts[0].lower()
            args = parts[1:]

            try:
                self._handle_command(cmd_name, args)
            except Exception as e:
                print(f'Error: {e}')

            time.sleep(0.05)

    def _handle_command(self, cmd, args):
        if cmd in ('q', 'quit', 'exit'):
            self.get_logger().info('Quit requested')
            self._running = False
            rclpy.shutdown()
            return

        if cmd in ('help', 'h', '?'):
            self._print_help()
            return

        if cmd == 'enable':
            self._ensure_robot_enabled()
            return

        if cmd == 'disable':
            self._disable_robot()
            return

        if cmd == 'status':
            self._print_status()
            return

        if cmd == 'lo':
            self._send_gripper_position(self.left_gripper_cmd, 'left', 100.0)
            return

        if cmd == 'lc':
            self._send_gripper_position(self.left_gripper_cmd, 'left', 0.0)
            return

        if cmd == 'ro':
            self._send_gripper_position(self.right_gripper_cmd, 'right', 100.0)
            return

        if cmd == 'rc':
            self._send_gripper_position(self.right_gripper_cmd, 'right', 0.0)
            return

        if cmd == 'lp':
            if len(args) != 1:
                print('Usage: lp <0-100>')
                return
            self._send_gripper_position(self.left_gripper_cmd, 'left', float(args[0]))
            return

        if cmd == 'rp':
            if len(args) != 1:
                print('Usage: rp <0-100>')
                return
            self._send_gripper_position(self.right_gripper_cmd, 'right', float(args[0]))
            return

        if cmd == 'open_both':
            self._send_gripper_position(self.left_gripper_cmd, 'left', 100.0)
            self._send_gripper_position(self.right_gripper_cmd, 'right', 100.0)
            return

        if cmd == 'close_both':
            self._send_gripper_position(self.left_gripper_cmd, 'left', 0.0)
            self._send_gripper_position(self.right_gripper_cmd, 'right', 0.0)
            return

        if cmd == 'both_grip':
            if len(args) != 1:
                print('Usage: both_grip <0-100>')
                return
            val = float(args[0])
            self._send_gripper_position(self.left_gripper_cmd, 'left', val)
            self._send_gripper_position(self.right_gripper_cmd, 'right', val)
            return

        if cmd == 'cal':
            self._ensure_grippers_calibrated()
            return

        if cmd == 'head':
            if len(args) != 1:
                print('Usage: head <angle_rad>')
                return
            self._send_head_pan(float(args[0]))
            return

        if cmd == 'head_deg':
            if len(args) != 1:
                print('Usage: head_deg <angle_deg>')
                return
            angle_rad = float(args[0]) * 3.14159265 / 180.0
            self._send_head_pan(angle_rad)
            return

        if cmd == 'head_center':
            self._send_head_pan(0.0)
            return

        if cmd == 'lhome':
            self._send_left_arm(self.LEFT_HOME)
            return

        if cmd == 'lneutral':
            self._send_left_arm(self.LEFT_NEUTRAL)
            return

        if cmd == 'ltuck':
            self._send_left_arm(self.LEFT_TUCK)
            return

        if cmd == 'ltable_high':
            self._send_left_arm(self.LEFT_TABLE_HIGH)
            return

        if cmd == 'ltable_low':
            self._send_left_arm(self.LEFT_TABLE_LOW)
            return

        if cmd == 'lcurrent':
            joints = self._get_left_arm_joints()
            names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
            print('\nLeft arm current joints:')
            for n, v in zip(names, joints):
                print(f'  {n}: {v:.4f}')
            print(f'\nAs list: {[round(v, 4) for v in joints]}\n')
            return

        if cmd == 'lpose':
            if len(args) != 7:
                print('Usage: lpose j0 j1 j2 j3 j4 j5 j6')
                return
            joints = [float(x) for x in args]
            self._send_left_arm(joints)
            return

        if cmd == 'ljoint':
            if len(args) != 2:
                print('Usage: ljoint <idx 0-6> <value>')
                return
            idx = int(args[0])
            val = float(args[1])
            if idx < 0 or idx > 6:
                print('Joint index must be 0-6')
                return
            joints = self._get_left_arm_joints()
            joints[idx] = val
            self._send_left_arm(joints)
            return

        if cmd == 'rhome':
            self._send_right_arm(self.RIGHT_HOME)
            return

        if cmd == 'rneutral':
            self._send_right_arm(self.RIGHT_NEUTRAL)
            return

        if cmd == 'rtuck':
            self._send_right_arm(self.RIGHT_TUCK)
            return

        if cmd == 'rtable_high':
            self._send_right_arm(self.RIGHT_TABLE_HIGH)
            return

        if cmd == 'rtable_low':
            self._send_right_arm(self.RIGHT_TABLE_LOW)
            return

        if cmd == 'rcurrent':
            joints = self._get_right_arm_joints()
            names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
            print('\nRight arm current joints:')
            for n, v in zip(names, joints):
                print(f'  {n}: {v:.4f}')
            print(f'\nAs list: {[round(v, 4) for v in joints]}\n')
            return

        if cmd == 'rpose':
            if len(args) != 7:
                print('Usage: rpose j0 j1 j2 j3 j4 j5 j6')
                return
            joints = [float(x) for x in args]
            self._send_right_arm(joints)
            return

        if cmd == 'rjoint':
            if len(args) != 2:
                print('Usage: rjoint <idx 0-6> <value>')
                return
            idx = int(args[0])
            val = float(args[1])
            if idx < 0 or idx > 6:
                print('Joint index must be 0-6')
                return
            joints = self._get_right_arm_joints()
            joints[idx] = val
            self._send_right_arm(joints)
            return

        if cmd == 'bhome':
            self._send_both_arms(self.LEFT_HOME, self.RIGHT_HOME)
            return

        if cmd == 'bneutral':
            self._send_both_arms(self.LEFT_NEUTRAL, self.RIGHT_NEUTRAL)
            return

        if cmd == 'btuck':
            self._send_both_arms(self.LEFT_TUCK, self.RIGHT_TUCK)
            return

        if cmd == 'btable_high':
            self._send_both_arms(self.LEFT_TABLE_HIGH, self.RIGHT_TABLE_HIGH)
            return

        if cmd == 'btable_low':
            self._send_both_arms(self.LEFT_TABLE_LOW, self.RIGHT_TABLE_LOW)
            return

        print(f'Unknown command: {cmd}')
        print('Type "help" for available commands.')

    def destroy_node(self):
        self._running = False

        for attr in [
            'robot_state_topic',
            'joint_state_topic',
            'left_gripper_state_topic',
            'right_gripper_state_topic',
            'left_gripper_props_topic',
            'right_gripper_props_topic',
        ]:
            topic = getattr(self, attr, None)
            if topic is not None:
                try:
                    topic.unsubscribe()
                except Exception:
                    pass

        for attr in [
            'enable_pub',
            'left_gripper_cmd',
            'right_gripper_cmd',
            'head_cmd',
            'left_arm_cmd',
            'right_arm_cmd',
        ]:
            topic = getattr(self, attr, None)
            if topic is not None:
                try:
                    topic.unadvertise()
                except Exception:
                    pass

        try:
            self.client.terminate()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BaxterCLI()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

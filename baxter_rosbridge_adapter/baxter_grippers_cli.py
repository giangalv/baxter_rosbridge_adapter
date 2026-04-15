import json
import threading
import time

import rclpy
from rclpy.node import Node
import roslibpy


class BaxterGrippersCLI(Node):
    def __init__(self):
        super().__init__('baxter_grippers_cli')

        self.declare_parameter('baxter_host', '130.251.13.31')
        self.declare_parameter('baxter_port', 9090)
        self.declare_parameter('gripper_id', 65538)
        self.declare_parameter('sender', 'baxter_ros2_gripper_cli')
        self.declare_parameter('auto_calibrate', False)

        self.host = self.get_parameter('baxter_host').value
        self.port = int(self.get_parameter('baxter_port').value)
        self.gripper_id = int(self.get_parameter('gripper_id').value)
        self.sender = self.get_parameter('sender').value

        self.client = roslibpy.Ros(host=self.host, port=self.port)

        self.robot_state = {'msg': None}
        self.left_state = {'msg': None}
        self.right_state = {'msg': None}
        self.left_props = {'msg': None}
        self.right_props = {'msg': None}

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

        self.left_state_topic = self._make_subscriber(
            '/robot/end_effector/left_gripper/state',
            'baxter_core_msgs/EndEffectorState',
            self.left_state
        )

        self.right_state_topic = self._make_subscriber(
            '/robot/end_effector/right_gripper/state',
            'baxter_core_msgs/EndEffectorState',
            self.right_state
        )

        self.left_props_topic = self._make_subscriber(
            '/robot/end_effector/left_gripper/properties',
            'baxter_core_msgs/EndEffectorProperties',
            self.left_props
        )

        self.right_props_topic = self._make_subscriber(
            '/robot/end_effector/right_gripper/properties',
            'baxter_core_msgs/EndEffectorProperties',
            self.right_props
        )

        self.enable_topic = roslibpy.Topic(
            self.client,
            '/robot/set_super_enable',
            'std_msgs/Bool'
        )

        self.left_cmd = roslibpy.Topic(
            self.client,
            '/robot/end_effector/left_gripper/command',
            'baxter_core_msgs/EndEffectorCommand'
        )

        self.right_cmd = roslibpy.Topic(
            self.client,
            '/robot/end_effector/right_gripper/command',
            'baxter_core_msgs/EndEffectorCommand'
        )

    def _pretty(self, obj):
        return json.dumps(obj, indent=2)
    
    def _print_status(self):
        robot = self.robot_state['msg'] or {}
        left = self.left_state['msg'] or {}
        right = self.right_state['msg'] or {}

        robot_enabled = robot.get('enabled', False)
        robot_ready = robot.get('ready', False)

        left_pos = left.get('position', None)
        right_pos = right.get('position', None)

        left_cal = bool(left.get('calibrated', 0))
        right_cal = bool(right.get('calibrated', 0))

        left_ready = bool(left.get('ready', 0))
        right_ready = bool(right.get('ready', 0))

        print('\n=== STATUS ===')
        print(f'Robot    -> enabled: {robot_enabled}, ready: {robot_ready}')
        print(f'Left     -> position: {left_pos}, calibrated: {left_cal}, ready: {left_ready}')
        print(f'Right    -> position: {right_pos}, calibrated: {right_cal}, ready: {right_ready}')
        print()

    def _ensure_robot_enabled(self):
        state = self.robot_state['msg']
        self.get_logger().info('Robot state before enable:\n' + self._pretty(state))

        if state and state.get('enabled', False) and state.get('ready', False):
            self.get_logger().info('Robot is already enabled and ready')
            return

        self.get_logger().info('Enabling robot...')
        self.enable_topic.publish(roslibpy.Message({'data': True}))
        time.sleep(1.0)

        state = self.robot_state['msg']
        self.get_logger().info('Robot state after enable:\n' + self._pretty(state))

        if not state or not state.get('enabled', False):
            raise RuntimeError('Robot did not become enabled')
        if not state.get('ready', False):
            raise RuntimeError('Robot did not become ready')

    def _send_calibrate(self, pub, side):
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
        left = self.left_state['msg']
        right = self.right_state['msg']

        self.get_logger().info('Left gripper state:\n' + self._pretty(left))
        self.get_logger().info('Right gripper state:\n' + self._pretty(right))

        need_left = not left or not bool(left.get('calibrated', 0))
        need_right = not right or not bool(right.get('calibrated', 0))

        if not need_left and not need_right:
            self.get_logger().info('Both grippers are already calibrated')
            return

        self.get_logger().info('Calibrating grippers...')

        if need_left:
            self._send_calibrate(self.left_cmd, 'left')
            time.sleep(2.0)

        if need_right:
            self._send_calibrate(self.right_cmd, 'right')
            time.sleep(2.0)

        time.sleep(1.0)

        self.get_logger().info('Left gripper state after calibration:\n' + self._pretty(self.left_state['msg']))
        self.get_logger().info('Right gripper state after calibration:\n' + self._pretty(self.right_state['msg']))

        if not bool(self.left_state['msg'].get('calibrated', 0)):
            raise RuntimeError('Left gripper calibration failed')
        if not bool(self.right_state['msg'].get('calibrated', 0)):
            raise RuntimeError('Right gripper calibration failed')

    def _send_position(self, pub, side, pos):
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

    def _run_cli(self):
        print('\nCommands:')
        print('  lo = left open')
        print('  lc = left close')
        print('  ro = right open')
        print('  rc = right close')
        print('  lp X = left position 0..100')
        print('  rp X = right position 0..100')
        print('  both X = both grippers to position 0..100')
        print('  open_both = both grippers open')
        print('  close_both = both grippers close')
        print('  status = print robot/gripper status')
        print('  enable = enable robot')
        print('  cal = calibrate grippers')
        print('  q = quit')

        while self._running:
            try:
                cmd = input('> ').strip().lower()
            except EOFError:
                cmd = 'q'
            except KeyboardInterrupt:
                cmd = 'q'

            if cmd == 'q':
                self.get_logger().info('Quit requested')
                self._running = False
                rclpy.shutdown()
                break

            elif cmd == 'lo':
                self._send_position(self.left_cmd, 'left', 100.0)

            elif cmd == 'lc':
                self._send_position(self.left_cmd, 'left', 0.0)

            elif cmd == 'ro':
                self._send_position(self.right_cmd, 'right', 100.0)

            elif cmd == 'rc':
                self._send_position(self.right_cmd, 'right', 0.0)

            elif cmd == 'open_both':
                self._send_position(self.left_cmd, 'left', 100.0)
                self._send_position(self.right_cmd, 'right', 100.0)

            elif cmd == 'close_both':
                self._send_position(self.left_cmd, 'left', 0.0)
                self._send_position(self.right_cmd, 'right', 0.0)

            elif cmd == 'status':
                self._print_status()

            elif cmd == 'enable':
                self._ensure_robot_enabled()

            elif cmd == 'cal':
                self._ensure_grippers_calibrated()

            elif cmd.startswith('lp '):
                try:
                    val = float(cmd.split()[1])
                    self._send_position(self.left_cmd, 'left', val)
                except Exception:
                    print('Usage: lp X')

            elif cmd.startswith('rp '):
                try:
                    val = float(cmd.split()[1])
                    self._send_position(self.right_cmd, 'right', val)
                except Exception:
                    print('Usage: rp X')

            elif cmd.startswith('both '):
                try:
                    val = float(cmd.split()[1])
                    self._send_position(self.left_cmd, 'left', val)
                    self._send_position(self.right_cmd, 'right', val)
                except Exception:
                    print('Usage: both X')

            else:
                print('Unknown command')

            time.sleep(0.2)

    def destroy_node(self):
        self._running = False

        for topic_name in [
            'robot_state_topic',
            'left_state_topic',
            'right_state_topic',
            'left_props_topic',
            'right_props_topic',
        ]:
            topic = getattr(self, topic_name, None)
            if topic is not None:
                try:
                    topic.unsubscribe()
                except Exception:
                    pass

        for topic_name in ['enable_topic', 'left_cmd', 'right_cmd']:
            topic = getattr(self, topic_name, None)
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
        node = BaxterGrippersCLI()
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

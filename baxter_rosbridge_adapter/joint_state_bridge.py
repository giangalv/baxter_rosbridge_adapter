import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import roslibpy


class BaxterJointStateBridge(Node):
    def __init__(self):
        super().__init__('baxter_joint_state_bridge')

        self.declare_parameter('baxter_host', '130.251.13.31')
        self.declare_parameter('baxter_port', 9090)
        self.declare_parameter('ros1_topic', '/robot/joint_states')
        self.declare_parameter('ros2_topic', '/joint_states')

        host = self.get_parameter('baxter_host').value
        port = int(self.get_parameter('baxter_port').value)
        ros1_topic = self.get_parameter('ros1_topic').value
        ros2_topic = self.get_parameter('ros2_topic').value

        self.pub = self.create_publisher(JointState, ros2_topic, 10)

        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()

        if self.client.is_connected:
            self.get_logger().info(f'Connected to Baxter rosbridge at ws://{host}:{port}')
        else:
            self.get_logger().error(f'Failed to connect to Baxter rosbridge at ws://{host}:{port}')
            raise RuntimeError('Cannot connect to Baxter rosbridge')

        # cache for complete robot state
        self.joint_cache = {}
        self.last_header_frame_id = ''
        self.last_stamp_sec = 0
        self.last_stamp_nanosec = 0

        self.sub = roslibpy.Topic(self.client, ros1_topic, 'sensor_msgs/JointState')
        self.sub.subscribe(self.callback)
        self.get_logger().info(f'Subscribed to {ros1_topic}, publishing merged state to {ros2_topic}')

    def callback(self, message):
        header = message.get('header', {})
        stamp = header.get('stamp', {})

        sec = int(stamp.get('secs', 0))
        nanosec = int(stamp.get('nsecs', 0))
        frame_id = header.get('frame_id', '')

        self.last_stamp_sec = sec
        self.last_stamp_nanosec = nanosec
        self.last_header_frame_id = frame_id

        names = list(message.get('name', []))
        positions = list(message.get('position', []))
        velocities = list(message.get('velocity', []))
        efforts = list(message.get('effort', []))

        for i, name in enumerate(names):
            pos = positions[i] if i < len(positions) else 0.0
            vel = velocities[i] if i < len(velocities) else 0.0
            eff = efforts[i] if i < len(efforts) else 0.0

            self.joint_cache[name] = {
                'position': pos,
                'velocity': vel,
                'effort': eff
            }

        out = JointState()
        out.header.stamp.sec = self.last_stamp_sec
        out.header.stamp.nanosec = self.last_stamp_nanosec
        out.header.frame_id = self.last_header_frame_id

        joint_names = sorted(self.joint_cache.keys())

        out.name = joint_names
        out.position = [self.joint_cache[n]['position'] for n in joint_names]
        out.velocity = [self.joint_cache[n]['velocity'] for n in joint_names]
        out.effort = [self.joint_cache[n]['effort'] for n in joint_names]

        self.pub.publish(out)

    def destroy_node(self):
        try:
            self.sub.unsubscribe()
        except Exception:
            pass
        try:
            self.client.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaxterJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

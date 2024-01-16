import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from lbr_fri_msgs.msg import LBRState, LBRPositionCommand


class JointStateSplitter(Node):

    def __init__(self):
        super().__init__('joint_state_splitter')
        self.command_publisher = [self.create_publisher(Float64, f'a{i + 1}_command', 1) for i in range(7)]
        self.measure_publisher = [self.create_publisher(Float64, f'a{i + 1}_measure', 1) for i in range(7)]

        self.state_listener = self.create_subscription(
            LBRState,
            '/lbr/state',
            self.state_listener_callback,
            10)
        self.state_listener  # prevent unused variable warning

        self.command_listener = self.create_subscription(
            LBRPositionCommand,
            '/lbr/command/joint_position',
            self.command_listener_callback,
            10)
        self.command_listener  # prevent unused variable warning
    

    def state_listener_callback(self, msg):
        
        self.get_logger().info('State: "%f"' % msg.measured_joint_position[4])
        # Publish state position
        for index, publisher in enumerate(self.measure_publisher):
            measure_msg = Float64()
            measure_msg.data = msg.measured_joint_position[index]
            publisher.publish(measure_msg)
    
    def command_listener_callback(self, msg):
        
        self.get_logger().info('I heard: "%f"' % msg.joint_position[4])
        # Publish commanded position
        for index, publisher in enumerate(self.command_publisher):
            command_msg = Float64()
            command_msg.data = msg.joint_position[index]
            publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)

    joint_state_splitter = JointStateSplitter()

    rclpy.spin(joint_state_splitter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_state_splitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
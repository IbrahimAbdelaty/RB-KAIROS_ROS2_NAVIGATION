#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRepublisher(Node):
    """
    A ROS 2 node that republishes Twist messages from /cmd_vel to a custom topic.
    """
    def __init__(self):
        """
        Initializes the CmdVelRepublisher node.
        Sets up the publisher and subscriber.
        """
        # Initialize the ROS 2 node with a unique name
        super().__init__('cmd_vel_republisher')
        self.get_logger().info("CmdVelRepublisher node started.")

        # Define the output topic name
        self.output_topic = '/robot/robotnik_base_controller/cmd_vel_unstamped'
        # Define the input topic name
        self.input_topic = '/cmd_vel'

        # Create a publisher to the custom topic
        # The message type is Twist from geometry_msgs
        # queue_size is replaced by qos_profile.depth in ROS 2
        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.get_logger().info(f"Publisher initialized for topic: {self.output_topic}")

        # Create a subscriber to the /cmd_vel topic
        # When a message is received, the 'cmd_vel_callback' method will be called
        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self.cmd_vel_callback,
            10  # QoS depth
        )
        # Prevent unused variable warning, although in this simple case it's not strictly necessary
        self.subscription
        self.get_logger().info(f"Subscriber initialized for topic: {self.input_topic}")


    def cmd_vel_callback(self, msg):
        """
        Callback function for the /cmd_vel subscriber.
        This function is called every time a Twist message is received on /cmd_vel.
        It then republishes the same message to the output topic.

        Args:
            msg (geometry_msgs.msg.Twist): The received Twist message.
        """
        # Log the received message (optional, for debugging)
        # self.get_logger().info(f"Received Twist message on {self.input_topic}: Linear.x={msg.linear.x}, Angular.z={msg.angular.z}"
        # msg.angular.z= -msg.angular.z
        # Publish the received message to the output topic
        self.publisher.publish(msg)
        # self.get_logger().info(f"Republished Twist message to {self.output_topic}")

def main(args=None):
    """
    Main function to run the CmdVelRepublisher node.
    """
    rclpy.init(args=args) # Initialize ROS 2 communication

    republisher_node = CmdVelRepublisher()

    try:
        rclpy.spin(republisher_node) # Keep the node running until it's shut down
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    finally:
        republisher_node.destroy_node() # Clean up node resources
        rclpy.shutdown() # Shut down ROS 2 communication

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TriggerControlNode(Node):
    def __init__(self):
        super().__init__('trigger_control_node')
        
        # Create a publisher for /cmd_vel2
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel2', 10)
        
        # Create a subscriber for /joy topic (joystick input)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Initialize Twist message
        self.twist = Twist()

    def joy_callback(self, msg):
        # Map the right trigger (R2) to forward speed (0 to 1 range)
        right_trigger = msg.axes[5]  # Right trigger (usually on axes[5] in many controllers)
        
        # Map right trigger from 0 to 1 (no need to invert)
        rt_value = right_trigger  # Map directly to 0 to 1
        
        # Set the forward speed based on the right trigger value
        self.twist.linear.x = rt_value  # Forward speed (0 to 1)
        
        # Map the left trigger (L2) to reverse speed (0 to -1 range)
        left_trigger = msg.axes[4]  # Left trigger (usually on axes[4] in many controllers)
        
        # Map left trigger from 0 to -1
        lt_value = -left_trigger  # Map directly to 0 to -1
        
        # Set reverse speed based on the left trigger value
        self.twist.linear.x += lt_value  # Reverse speed (0 to -1)
        
        # Use the left joystick (horizontal axis) for angular movements (turning)
        self.twist.angular.z = msg.axes[0]  # Left joystick horizontal axis
        
        # Publish the message to /cmd_vel2
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = TriggerControlNode()
    rclpy.spin(node)

    # Destroy the node after stopping
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

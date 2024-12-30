## Part of this code is contributed by Group 11 member: Jonas Land
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class VelocityTranslatorVode(Node):

    def __init__(self):
        super().__init__('velocity_translator_node')

        self.get_logger().info(f"Started velocity_translator_node node")

        # subscriber for cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # publisher to send out vl data
        self.vl_publisher = self.create_publisher(Float64, '/vl', 10)

        # publisher to send out vr data
        self.vr_publisher = self.create_publisher(Float64, '/vr', 10)

        # Declare parameters for wheel properties
        self.declare_parameter('distance', 0.0)
        self.declare_parameter('radius', 0.0)

        # Get parameter values
        self.radius = self.get_parameter('radius').value
        self.distance = self.get_parameter('distance').value

    def cmd_vel_callback(self, msg):
        # Translate linear and angular components into wheel velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate wheel velocities based on differential drive equations
        vr = (2 * linear_vel + angular_vel * self.distance) / (2 * self.radius) / 10
        vl = (2 * linear_vel - angular_vel * self.distance) / (2 * self.radius) / 10

        # Create Float64 messages for wheel velocities
        vr_msg = Float64()
        vr_msg.data = vr
        vl_msg = Float64()
        vl_msg.data = vl

        # Publish the calculated wheel velocities
        self.vr_publisher.publish(vr_msg)
        self.vl_publisher.publish(vl_msg)
        
def main(args=None):
    rclpy.init(args=args)
    velocity_translator_node = VelocityTranslatorVode()
    rclpy.spin(velocity_translator_node)
    velocity_translator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

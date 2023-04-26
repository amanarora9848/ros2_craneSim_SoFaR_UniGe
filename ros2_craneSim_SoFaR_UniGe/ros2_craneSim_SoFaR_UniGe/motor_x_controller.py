import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Point
# from simple_pid import PID


class MotorXController(Node):
    def __init__(self):
        super().__init__("motor_x_controller")
        self.get_logger().info("Motor X Controller Started")
        # Publish motor_x value to topic /motor_x
        self.motor_x_pub = self.create_publisher(Float64, "/motor_x", 10)
        # Publish acknowledgement message to topic /ack_x when the controller is idle
        self.ack_x_pub = self.create_publisher(Bool, "/ack_x", 10)
        # Subscribe to topic /controller_setpoint to get the target position
        self.target_sub = self.create_subscription(Point, "/controller_setpoint", self.target_callback, 10)
        # Initialize motor_x and target
        self.motor_x = 0
        self.target = 0

    def target_callback(self, msg: Point):
        # Self-implemented Controller loop
        # Set the target to the x value of the Point message
        self.target = msg.x
        # Publish a message to topic /ack_x with data False
        self.ack_x_pub.publish(Bool(data=False))
        while True:
            # Calculate the error
            error = self.target - self.motor_x
            self.get_logger().info("Error: " + str(error))
            # Update the motor_x value
            self.motor_x += error * 0.003
            self.motor_x_pub.publish(Float64(data=self.motor_x))
            # If the error is less than 0.1, publish a message to topic /ack_x with data True and break
            if abs(error) < 0.1:
                self.ack_x_pub.publish(Bool(data=True))
                break


def main():
    rclpy.init(args=None)

    # Create the node
    motor_x_controller = MotorXController()
    rclpy.spin(motor_x_controller)

    # On shutdown, destroy the node explicitly
    motor_x_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Point
# from simple_pid import PID


class MotorYController(Node):
    def __init__(self):
        super().__init__("motor_y_controller")
        self.get_logger().info("Motor Y Controller Started")
        # Publish motor_y value to topic /motor_y (std_msgs/Float64)
        self.motor_y_pub = self.create_publisher(Float64, "/motor_y", 10)
        # Publish acknowledgement message to topic /ack_y (std_msgs/Bool) when the controller is idle
        self.ack_y_pub = self.create_publisher(Bool, "/ack_y", 10)
        # Subscribe to topic /controller_setpoint (geometry_msgs/Point) to get the target position
        self.target_sub = self.create_subscription(Point, "/controller_setpoint", self.target_callback, 10)
        # Initialize motor_y and target
        self.motor_y = 0
        self.target = 0

    def target_callback(self, msg: Point):
        # Self-implemented Controller loop
        # Set the target to the y value of the Point message
        self.target = msg.y
        # Publish a message to topic /ack_y with data False
        self.ack_y_pub.publish(Bool(data=False))
        while True:
            # Calculate the error
            error = self.target - self.motor_y
            self.get_logger().info("Error: " + str(error))
            # Update the motor_y value
            self.motor_y += error * 0.003
            self.motor_y_pub.publish(Float64(data=self.motor_y))
            # If the error is less than 0.1, publish a message to topic /ack_y with data True and break
            if abs(error) < 0.1:
                self.ack_y_pub.publish(Bool(data=True))
                break


def main():
    rclpy.init(args=None)

    # Create the node
    motor_y_controller = MotorYController()
    rclpy.spin(motor_y_controller)

    # On shutdown, destroy the node explicitly
    motor_y_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

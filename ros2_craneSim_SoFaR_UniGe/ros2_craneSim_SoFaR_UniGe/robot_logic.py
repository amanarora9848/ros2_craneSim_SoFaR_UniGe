import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Bool

class RobotLogic(Node):
    def __init__(self):
        super().__init__("robot_logic")
        self.stage_pub = self.create_publisher(Int64, "/next_stage", 10)
        # subscribe to /ack_x and /ack_y topics to check if both controllers are idle
        self.ack_x_sub = self.create_subscription(Bool, "/ack_x", self.ack_x_callback, 10)
        self.ack_y_sub = self.create_subscription(Bool, "/ack_y", self.ack_y_callback, 10)

        # Initialize stage and timer
        self.stage = 1
        self.timer = self.create_timer(1, self.timer_callback)

        # Initialize ack_x and ack_y
        self.ack_x = True
        self.ack_y = True

        # Initialize score
        self.score = 0

    def ack_x_callback(self, msg):
        # Update ack_x and log it
        self.ack_x = msg.data
        self.get_logger().info("ack_x: {0}".format(self.ack_x))

    def ack_y_callback(self, msg):
        # Update ack_y and log it
        self.ack_y = msg.data
        self.get_logger().info("ack_y: {0}".format(self.ack_y))

    def timer_callback(self):
        if self.ack_x and self.ack_y:
            # publish next stage
            if self.stage == 6:
                self.score += 1
                self.get_logger().info("Pick and place completed. Score: {0}".format(self.score))
                self.stage = 1
            self.stage_pub.publish(Int64(data=self.stage))
            self.get_logger().info("Published next stage: {0}".format(self.stage))
            self.stage += 1


def main(args=None):
    # Initialize rclpy and node
    rclpy.init(args=args)
    robot_logic = RobotLogic()
    rclpy.spin(robot_logic)

    # Destroy node and shutdown rclpy
    robot_logic.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

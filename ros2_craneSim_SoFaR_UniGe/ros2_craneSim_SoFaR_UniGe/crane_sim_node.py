import arcade
import rclpy 
import numpy as np
from enum import Enum
from threading import Thread
from rclpy.node import Node
from .lib.crane_sim import CraneSimulation

from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Point

# Enum for pick/place phases
class Stage(Enum):
    PICK = 1
    LIFT = 2
    MOVE = 3
    PLACE = 4
    DROP = 5

# Thread to run simulation in background
class CraneSimThread(Thread):
   def __init__(self):
      Thread.__init__(self)
   
   def run(self):
      arcade.run()

# Simulation node
class CraneSimNode(Node):

    def __init__(self):
        super().__init__("crane_simulation_node")
        
        # Set up simulation environment
        self.sim = CraneSimulation()
        self.sim.setup()
        
        # Get container locations variables
        self.container_loc = None
        self.deposit_location = self.sim.get_container_deposit_location()
        # Get center of workspace location
        self.center_loc = self.sim.get_center_location()

        # Run simulation in separate thread
        self.thread = CraneSimThread()
        self.thread.start()

        # Subscribers for motors' positions
        self.sub_motor_x = self.create_subscription(Float64, "/motor_x", self.callback_motor_x, 10)
        self.sub_motor_y = self.create_subscription(Float64, "/motor_y", self.callback_motor_y, 10)
        # Subscriber for next pick/place stage
        self.stage_sub = self.create_subscription(Int64, "/next_stage", self.stage_callback, 10)

        # Publishers for next controller setpoint based on current pick/place stage
        self.target_pub = self.create_publisher(Point, "/controller_setpoint", 10)

        # Utilities
        self.lift_offset = 75

    # Callback for setting position of motor x 
    def callback_motor_x(self, msg: Float64):
        self.sim.set_end_effector_position(x=msg.data)

    # Callback for setting position of motor y
    def callback_motor_y(self, msg: Float64):
        self.sim.set_end_effector_position(y=msg.data)

    # Callback for next pick/place stage received
    def stage_callback(self, msg: Int64):
        next_stage = Stage(msg.data)
        self.get_logger().info("Received next stage message: {0}".format(next_stage))
        # Instantiate next target msg
        next_target_msg = Point()
        # If next stage is PICK, publish container's location
        if next_stage == Stage.PICK:
            self.container_loc = self.sim.get_container_location()
            next_target_msg.x = float(self.container_loc[0])
            next_target_msg.y = float(self.container_loc[1])
        # If next stage is LIFT, hook container to end-effector, then lift it
        elif next_stage == Stage.LIFT:
            self.sim.hook_container()
            next_target_msg.x = float(self.container_loc[0])
            next_target_msg.y = float(self.container_loc[1] - self.lift_offset)
        # If next stage is MOVE, move container above deposit location
        elif next_stage == Stage.MOVE:
            next_target_msg.x = float(self.deposit_location[0])
            next_target_msg.y = float(self.deposit_location[1] - self.lift_offset) 
        # If next stage is PLACE, descend to container deposit location
        elif next_stage == Stage.PLACE:
            next_target_msg.x = float(self.deposit_location[0])
            next_target_msg.y = float(self.deposit_location[1]) 
        # If next stage is DROP, release container and move back to center 
        elif next_stage == Stage.DROP:
            self.sim.drop_container()
            next_target_msg.x = float(self.center_loc[0])
            next_target_msg.y = float(self.center_loc[1])
        # In all cases, publish next target...
        self.target_pub.publish(next_target_msg)
            

# Main function
def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation and spin indefinitely..
    crane_sim_node = CraneSimNode()
    rclpy.spin(crane_sim_node)

    # On shutdown...
    crane_sim_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()
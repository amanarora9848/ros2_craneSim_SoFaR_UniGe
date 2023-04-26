import arcade
import random
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory

# Constants
SCREEN_WIDTH = 1025
SCREEN_HEIGHT = 600
SCREEN_TITLE = "Crane Sim"

# Constants defining offset for crane's origin
X_OFFSET = 220
Y_OFFSET = 430

# Constants defining max dimensions for crane workspace
CRANE_MAX_X = 590
CRANE_MAX_Y = 350

# Constants used to scale our sprites from their original size
BASE_SCALING = 0.5
CRANE_SCALING = 0.25
END_EFFECTOR_SCALING = 0.06

# Constant for distance threshold
DIST_THRESHOLD = 20

class CraneSimulation(arcade.Window):

    def __init__(self):
        super().__init__(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
        # Variables for keeping track of sprites
        self.terrain_list = None
        self.crane_structure_list = None
        self.end_effector = None
        self.chain_list = None
        self.container = None
        # Internal score variable
        self.score = 0
        # Variable to control interaction with containers
        self.is_container_set = False
        self.container_hooked = False
        self.vertical_hooking_distance = 0
        # Variable for container deposit location
        self.container_deposit_location = [X_OFFSET + 45, Y_OFFSET - (CRANE_MAX_Y + 15)]
        self.container_deposit_size = 80

        self.base_share_directory = get_package_share_directory("ros2_craneSim_SoFaR_UniGe")
        arcade.set_background_color(arcade.csscolor.WHITE)

    # Method to create base layer for the 2D world
    def make_terrain(self):
        self.terrain_list = arcade.SpriteList(use_spatial_hash=True)
        for x in range(0, SCREEN_WIDTH, 64):
            terrain = arcade.Sprite(":resources:images/tiles/stoneMid.png", BASE_SCALING)
            terrain.center_x = x
            terrain.center_y = 32
            self.terrain_list.append(terrain)
        
    # Method to create static crane structure 
    def make_crane_structure(self):
        self.crane_structure_list = arcade.SpriteList(use_spatial_hash=True)
        # Left walls - vertical
        for y in range(5):
            v_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_vertical.png", BASE_SCALING)
            v_wall.position = [200, 100 + 70*y]
            self.crane_structure_list.append(v_wall)
        # Top-left wall
        tl_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_top_left.png", CRANE_SCALING)
        tl_wall.position = [215, 88.5 + 70*(y+1)]
        self.crane_structure_list.append(tl_wall)
        # Top walls - horizontal
        for x in range(8):
            h_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_horizontal.png", BASE_SCALING)
            h_wall.position = [270 + 70*x, 104 + 70*(y+1)]
            self.crane_structure_list.append(h_wall)
        # Top-right wall
        tr_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_top_right.png", CRANE_SCALING)
        tr_wall.position = [270 + 70*(x) + 57.5, 88.5 + 70*(y+1)]
        self.crane_structure_list.append(tr_wall)
        # Right walls - vertical
        for y in range(5):
            v_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_vertical.png", BASE_SCALING)
            v_wall.position = [270 + 70*(x+1) + 2.5, 100 + 70*y]
            self.crane_structure_list.append(v_wall)

    # Method to instantiate end-effector sprite and spawn it in crane's origin position
    def make_end_effector(self):
        self.end_effector = arcade.Sprite(self.base_share_directory + "/resource/hook.png", END_EFFECTOR_SCALING)
        self.end_effector.position = [X_OFFSET, Y_OFFSET]
        self.chain_list = arcade.SpriteList(use_spatial_hash=True)

    # Method to spawn container object within crane's limits
    def make_container_at_random_location(self):
        self.container = arcade.Sprite(self.base_share_directory + "/resource/container.png", CRANE_SCALING)
        self.container.center_x = X_OFFSET + random.randint(CRANE_MAX_X / 2, CRANE_MAX_X-50)
        self.container.center_y = 78
        self.is_container_set = True

    # Retrieve center location of workspace
    def get_center_location(self):
        return np.array([CRANE_MAX_X/2, CRANE_MAX_Y/2])

    # Retrieve container's position (if set)
    def get_container_location(self):
        if self.is_container_set:
            return np.array([self.container.center_x - X_OFFSET, Y_OFFSET - 15 - self.container.center_y])
        else:
            return None

    # Retrieve container deposit position for external scripts
    def get_container_deposit_location(self):
        return np.array([self.container_deposit_location[0] - X_OFFSET, Y_OFFSET - 30 - self.container_deposit_location[1]])

    # Method to hook end-effector to container
    def hook_container(self):
        self.vertical_hooking_distance = self.end_effector.center_y - self.container.center_y
        self.container_hooked = True
        time.sleep(1)

    # Method to drop container on deposit
    def drop_container(self):
        self.container_hooked = False
        self.deposit_container()

    # Method to increase score if container deposited corerctly
    def deposit_container(self):
        self.score += 1
        self.is_container_set = False
        self.container = None
        
    # Arcade standard method to initialize all graphical components - called once during initialization
    def setup(self):
        # Init sprites list for terrain, crane structure and end-effector
        self.make_terrain()
        self.make_crane_structure()
        self.make_end_effector()
        
    # Method to update end-effector position with values from motors
    def set_end_effector_position(self, x=None, y=None):
        if x is not None:
            self.end_effector.center_x = X_OFFSET + x 
        if y is not None:
            self.end_effector.center_y = Y_OFFSET - y
        # Then check if ee's position is still within graphical limits
        self.check_end_effector_in_limits()

    # Additional checks to make sure end-effector is drawn inside crane
    def check_end_effector_in_limits(self):
        # Checks for the X axis
        if self.end_effector.center_x > X_OFFSET + CRANE_MAX_X:
            self.end_effector.center_x = X_OFFSET + CRANE_MAX_X
        elif self.end_effector.center_x < X_OFFSET:
            self.end_effector.center_x = X_OFFSET
        # Checks for the Y axis
        if self.end_effector.center_y > Y_OFFSET:
            self.end_effector.center_y = Y_OFFSET 
        elif abs(self.end_effector.center_y - Y_OFFSET) > CRANE_MAX_Y:
            self.end_effector.center_y = Y_OFFSET - CRANE_MAX_Y

    # Retrieve coordinates of end-effector
    def get_end_effector_position(self):
        return np.array([self.end_effector.center_x - X_OFFSET, Y_OFFSET - self.end_effector.center_y])

    # Draw deposit position
    def draw_container_deposit_location(self):
        arcade.draw_line(
            self.container_deposit_location[0] - self.container_deposit_size/2, 
            self.container_deposit_location[1], 
            self.container_deposit_location[0] + self.container_deposit_size/2, 
            self.container_deposit_location[1], 
            arcade.color.DARK_ORANGE, 
            5)

    # Method to draw score message on top right corner
    def draw_score_message(self):
        arcade.draw_text("SCORE: {0}".format(self.score),
            SCREEN_WIDTH - 300,
            SCREEN_HEIGHT - 50,
            arcade.color.BLACK,
            35)

    # Arcade standard method that redraws the canvas - called at every frame
    def on_draw(self):
        # Clear the screen to the background color
        self.clear()
        # Draw terrain
        self.terrain_list.draw()
        # Draw reference for container deposit
        self.draw_container_deposit_location()
        # If container spawned, draw it
        if self.is_container_set:
            self.container.draw()
        # Draw crane structure
        self.crane_structure_list.draw()
        # Draw end-effector and chain
        self.end_effector.draw()
        self.chain_list.draw()
        # Draw score 
        self.draw_score_message()

    # Method to draw chain once end-effector is descending
    def update_end_effector_chain(self):
        self.chain_list.clear()
        for v in range(Y_OFFSET, int(self.end_effector.center_y) + 15, -10):
            chain = arcade.Sprite(self.base_share_directory + "/resource/chain.png", BASE_SCALING)
            chain.center_x = self.end_effector.center_x
            chain.center_y = v
            self.chain_list.append(chain)

    # Arcade standard method for updating simulation - called at every frame
    def on_update(self, delta_time: float):
        self.update_end_effector_chain()
        if self.container is None:
            self.make_container_at_random_location()
        if self.container_hooked:
            self.container.center_x = self.end_effector.center_x
            self.container.center_y = self.end_effector.center_y - self.vertical_hooking_distance
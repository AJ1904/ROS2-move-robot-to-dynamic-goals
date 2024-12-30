## Part of this code is contributed by Group 11 member: Jonas Land
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import os
import tempfile
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import math
import random
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
import yaml
import xml.etree.ElementTree as ET
import json
import xmltodict 
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy
from project4d.disc_robot import load_disc_robot, disc_robot_urdf
import yaml
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from collections import defaultdict
from queue import PriorityQueue
from geometry_msgs.msg import Twist


class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator_node')
        # Declare parameters for the robot
        self.declare_parameter('radius', 0.0)
        self.declare_parameter('distance', 0.0)
        self.declare_parameter('height', 0.0)
        self.declare_parameter('error_variance_left', 0.0)
        self.declare_parameter('error_variance_right', 0.0)
        self.declare_parameter('error_update_rate', 0.0)
        
	# Get parameter values
        self.radius = self.get_parameter('radius').value
        self.distance = self.get_parameter('distance').value
        self.height = self.get_parameter('height').value
        self.error_variance_left = self.get_parameter('error_variance_left').value
        self.error_variance_right = self.get_parameter('error_variance_right').value
        self.error_update_rate = self.get_parameter('error_update_rate').value

        self.get_logger().info(f"Started simulator_node")
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        # MAP MAKER FUNCTIONALITY
        # publisher to send out vr data
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Declare parameters for wheel properties
        self.declare_parameter('world_file', "")

        # Get parameter values
        self.map_file = self.get_parameter('world_file').value

        # Initialize the OccupancyGrid message
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = "world"
        # Set the height of the map
        self.map_msg.info.origin = Pose()
        self.map_msg.info.origin.position = Point()
        self.map_msg.info.origin.orientation = Quaternion()
        
        self.two_d_map = None
        self.get_map_data()
                
        # Initialize graph and other variables
        self.graph = defaultdict(list)
        self.create_graph()
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

	# Set the timer for the callback function
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscriber for vl
        self.vl_subscription = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vl_subscription

        # Subscriber for vr
        self.vr_subscription = self.create_subscription(Float64, '/vr', self.vr_callback, 10)
        self.vr_subscription
        
        # subscriber for goal pose
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        
        # current goal for robot
        self.current_goal = None
        self.path = []
        self.reached_goal = False
        self.angle_corrected = False
        
        		
        # Set up the initial transform from world to base_link
        self.base_link_transform = TransformStamped()
        self.base_link_transform.header.frame_id = 'world'
        self.base_link_transform.child_frame_id = 'base_link'
        
        # Initialize TF2 Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # initial velocities
        self.vl = 0.0
        self.vr = 0.0
        
        # Initialize error variables
        self.error_left = 0.0  # Initialize with no error
        self.error_right = 0.0  # Initialize with no error
        
        # Initialize timestamp for velocity commands and error update
        self.last_velocity_update_time = self.get_clock().now()
        self.last_error_update_time = self.get_clock().now()
                    
    #GETS MAP DATA FROM THE FILE
    def get_map_data(self):
        self.get_logger().info(f"getting map data from the file")
        with open(self.map_file) as f:
            world = yaml.safe_load(f)
        resolution = world['resolution']
        self.map_msg.info.resolution = resolution  # Set the resolution of the map      
        initial_pose = world['initial_pose']
        self.pose = {'x': initial_pose[0], 'y': initial_pose[1], 'theta': initial_pose[2]}
        map_layout = world['map']
        current_row = []
        current_width = 0 # used to dynamically get the width
        current_height = 0 # used to dynamically get the height
        first_row = True
        self.current_map = []
        # A 2d map for ease of use in other functions
        self.two_d_map = []
        current_row = []
        for row in map_layout:          
            if first_row and (str(row) != "\n"):
                current_width += 1
            if str(row) == "#":
                # found obstacle
               current_row.append(int(100))
            elif str(row) == ".":
                # found free space
                current_row.append(int(0))
            elif str(row) == "\n":
                # end of line
                current_height += 1
                first_row = False
                # current_row.reverse()
                self.two_d_map.append(current_row)
                current_row = []

        # The map would be inverted before this point so this flips it
        self.two_d_map.reverse()
        for row in self.two_d_map:
            for col in row:
                self.current_map.append(col)  
                            
        self.map_msg.info.width = current_width
        self.map_msg.info.height = current_height
        self.get_logger().info(f"current_width: {current_width}, current_height: {current_height}")
        self.get_logger().info(f"map 2d: {self.two_d_map}")
        self.get_logger().info(f"map 1d: {self.current_map}")
        self.publish_map()
        
        # This timer calls publish for the map every 2 seconds
        self.world_timer = self.create_timer(2, self.publish_map)

    def create_graph(self):
        resolution = self.map_msg.info.resolution
        # Calculate the number of steps based on the resolution
        x_steps = int(len(self.two_d_map[0]) / (resolution))
        y_steps = int(len(self.two_d_map) / (resolution))
                
        for step_x in range(x_steps):
            for step_y in range(y_steps):
                x = step_x * resolution #+ self.radius / resolution
                y = step_y * resolution #+ self.radius / resolution
                #self.get_logger().info(f"x: {x}, y: {y}")
                if not self.is_collision(x,y) and round(x)<len(self.two_d_map[0]) and round(y)<len(self.two_d_map) and self.two_d_map[round(y)][round(x)] == 0:  # Consider empty cells as nodes
                    node_id = (round(x,2), round(y,2))  # Use (x, y) as unique identifiers for nodes
                
                    # Find neighboring empty cells with adjusted positions based on resolution
                    # self.get_logger().info(f"checking neighbors for {node_id}")
                    neighbors = self.find_neighbors(x, y)  
                               
                    self.graph[node_id] = neighbors  # Add edges between nodes
                # else: self.get_logger().info(f"either collision or out of bounds")
        # self.print_graph()
                    
    def find_neighbors(self, x, y):
        # Function to get neighboring empty cells for a given cell
        neighbors = []
        resolution = self.map_msg.info.resolution
        directions = [(resolution, 0), (0, resolution), (-resolution, 0), (0, -resolution)]  # Right, Down, Left, Up

        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy # x + dx, y + dy
            # map_x_index = int(round(new_x / resolution))
            map_x_index = round(new_x)
            map_y_index = round(new_y)
            #self.get_logger().info(f"new_x:{new_x}, new_y:{new_y}, map_x_index:{map_x_index}, map_y_index:{map_y_index}")
            if not self.is_collision(new_x,new_y) and 0 <= map_x_index < len(self.two_d_map[0]) and 0 <= map_y_index < len(self.two_d_map):
                if self.two_d_map[map_y_index][map_x_index] == 0:  # Check if neighbor cell is empty
                    neighbor_id = (round(new_x,2), round(new_y,2))  # Unique identifier for the neighbor node
                
                    # Check if the path between nodes passes through an obstacle
                    if not self.path_passes_through_obstacle(x, y, new_x, new_y):
                        neighbors.append(neighbor_id)  # Add neighbor to the list of neighbors
                    # else:
                    #    self.get_logger().info(f"path passes through obstacle")
               # else:
               #     self.get_logger().info(f"neighbor cell is not empty")
           # else:
           #      self.get_logger().info(f"out of bounds or collision neighbor")
        # self.get_logger().info(f"neighbors: {neighbors}")
        return neighbors
    
    def path_passes_through_obstacle(self, x1, y1, x2, y2):
        step_size = 0.1  # Define the step size for path interpolation
        resolution = self.map_msg.info.resolution
        num_steps = int(max(abs(x2 - x1), abs(y2 - y1)) / step_size)
        for i in range(num_steps + 1):
            interp_x = x1 + (x2 - x1) * (i / num_steps)
            interp_y = y1 + (y2 - y1) * (i / num_steps)

            for angle in range(360):
                current_rad = angle * (math.pi / 180)
                check_x = interp_x + self.radius * math.cos(current_rad)
                check_y = interp_y + self.radius * math.sin(current_rad)
                map_x_index = round(check_x / resolution)
                map_y_index = round(check_y / resolution)

                if (0 <= map_x_index < len(self.two_d_map[0]) and 0 <= map_y_index < len(self.two_d_map) and self.two_d_map[map_y_index][map_x_index] != 0):
                    return True  # Path intersects with an obstacle
        return False  # Path doesn't intersect with any obstacles

    # function to print graph
    def print_graph(self):
        # Function to print the generated graph
        self.get_logger().info("Printing Graph:")
        for node_id, neighbors in self.graph.items():
            self.get_logger().info(f"Node: {node_id} --> Neighbors: {neighbors}")
    
    # callback for goal pose
    def goal_pose_callback(self, msg):
        self.get_logger().info(f"Goal pose detected: {msg}")
        
        if self.is_collision(msg.pose.position.x, msg.pose.position.y):
            self.get_logger().info(f"Goal pose inside obstacle, discarding.")
            return
        
        self.goal_pose = msg    
        self.path = []
        self.angle_corrected = False
        self.reached_goal = False
        # temporary place holder for goal pose
        goal_pose_dict = {'x':self.goal_pose.pose.position.x, 'y':self.goal_pose.pose.position.y}

        # Check if there is a straight obstacle free path between current pose and goal pose
        if not self.path_passes_through_obstacle(goal_pose_dict['x'], goal_pose_dict['y'], self.pose['x'], self.pose['y']):
            self.path = []
        else:
            # Find the nearest node in the graph based on the current pose
            nearest_start_node = self.find_nearest_node(self.pose)
            self.get_logger().info(f"nearest start node: {nearest_start_node}")
        
            # Find the nearest node in the graph based on the goal pose
            nearest_goal_node = self.find_nearest_node(goal_pose_dict)
            self.get_logger().info(f"nearest goal node: {nearest_goal_node}")
            
            # generate the path using dijkstra
            self.path = self.generate_path(nearest_start_node, nearest_goal_node)

        # path is empty if straight line movement possible between current pose and goal pose, so just append the goal
        if not self.path:
            self.path = [goal_pose_dict]
        # else append the goal to the generated path
        else: 
            self.path.append(goal_pose_dict)
        self.get_logger().info(f"New path to follow: {self.path}")
    
    def correct_angle(self):
        current_theta = self.pose['theta']
        angle_tolerance = 0.01
        goal_theta = 2*math.asin(self.goal_pose.pose.orientation.z)
        angle_error = (goal_theta - current_theta + math.pi) % (2 * math.pi) - math.pi
        turn_direction = 1 if angle_error > 0 else -1  # 1 for clockwise, -1 for counterclockwise
        twist_msg = Twist()
        # self.get_logger().info("correcting angle ")
        if abs(angle_error) > angle_tolerance:
            twist_msg.angular.z = min(abs(angle_error)*turn_direction, 1.0*turn_direction) * turn_direction
            self.cmd_vel_publisher.publish(twist_msg)
        else:
            self.angle_corrected = True
            self.get_logger().info("angle correction complete")
            
            
    def follow_path(self):
        distance_tolerance = 0.01
        angle_tolerance = 0.01
        if self.path:
            twist_msg = Twist()
            current_pose = self.pose
            current_goal = self.path[0]
            # self.get_logger().info(f"current pose: {current_pose}")
            # self.get_logger().info(f"current goal: {current_goal}")
            if len(self.path)==1: # this is the actual goal node
                distance = math.sqrt((current_goal['x'] - current_pose['x'])**2  + (current_goal['y'] - current_pose['y'])**2)
                angle = math.atan2(current_goal['y'] - current_pose['y'] , current_goal['x'] - current_pose['x'])
                # angle_error = angle - self.pose['theta']
                angle_error = (angle - self.pose['theta'] + math.pi) % (2 * math.pi) - math.pi
                turn_direction = 1 if angle_error > 0 else -1  # 1 for clockwise, -1 for counterclockwise
                if abs(angle_error) > angle_tolerance:
                    twist_msg.angular.z = min(abs(angle_error)*turn_direction, 1.0*turn_direction)                
                else:
                    if distance >= distance_tolerance:
                        twist_msg.linear.x = min(distance, 2.0)                    
                    else:
                        twist_msg.linear.x = 0.0
                        del self.path[0]
                        self.get_logger().info(f"Path remaining: {self.path}")
                        self.reached_goal = True               
            else:
                distance = math.sqrt((current_goal[0] - current_pose['x'])**2  + (current_goal[1] - current_pose['y'])**2)
                angle = math.atan2(current_goal[1] - current_pose['y'] , current_goal[0] - current_pose['x'])
                # angle_error = angle - self.pose['theta']
                angle_error = (angle - self.pose['theta'] + math.pi) % (2 * math.pi) - math.pi
                turn_direction = 1 if angle_error > 0 else -1  # 1 for clockwise, -1 for counterclockwise
                if abs(angle_error) > angle_tolerance:
                    twist_msg.angular.z = min(abs(angle_error)*turn_direction, 1.0*turn_direction)          
                
                else:
                    if distance >= distance_tolerance:
                        twist_msg.linear.x = min(distance, 2.0)
                    else:
                        twist_msg.linear.x = 0.0
                        del self.path[0]
                        self.get_logger().info(f"Path remaining: {self.path}")
            self.cmd_vel_publisher.publish(twist_msg)
                                        
    def find_nearest_node(self, current_pose):
        # Function to find the nearest node in the graph based on the current pose
        min_distance = float('inf')  # Initialize minimum distance to infinity
        nearest_node = None

        for node_id in self.graph:
            # Calculate distance between the current robot pose and each node in the graph           
            node_pose = node_id  # node_id represents (x, y) coordinates
            distance = self.calculate_distance(current_pose, node_pose)
            # Update nearest_node and min_distance if a closer node is found
            if distance < min_distance:
                min_distance = distance
                nearest_node = node_id
        self.get_logger().info(f"{nearest_node}")
        return nearest_node
    
    def calculate_distance(self, pose1, pose2):
        # Function to calculate the Euclidean distance between two poses
        return ((pose2[0] - pose1['x']) ** 2 + (pose2[1] - pose1['y']) ** 2) ** 0.5
    
    def minDistance(self, dist, sptSet):
        # Initialize minimum distance for next node
        minn = float('inf')
        min_index = None
        # Search not nearest vertex not in the shortest path tree
        for v in self.graph:
            # self.get_logger().info(f"checking v: {v}: {dist[v]}")
            if dist[v] < minn and v not in sptSet:
                minn = dist[v]
                min_index = v
        return min_index
        
    def generate_path(self, nearest_start_node, nearest_goal_node):
        # Dijkstra's algorithm to find the path from nearest_start_node to nearest_goal_node
        # Initialize distances with infinity except for the nearest_start_node (distance 0)
        distances = {node: float('inf') for node in self.graph}
        # self.get_logger().info(f"{self.graph}")
        distances[nearest_start_node] = 0
        # self.get_logger().info(f"distances: {distances}")
        sptSet = set()
        
        predecessors = {}  # To store predecessors for constructing the path
        while len(sptSet) < len(self.graph):
            # Pick the minimum distance vertex from the set of vertices not yet processed. u is always equal to src in first iteration
            u = self.minDistance(distances, sptSet)            
            if u is None:
                # self.get_logger().info("cannot find a path")
                break  # No more reachable nodes
                
            sptSet.add(u)
            # if u == nearest_goal_node:
            if u == nearest_goal_node:
                break  # Found the goal node, no need to explore further
            for v in self.graph[u]:
                if v not in sptSet:
                    d = ((u[0]-v[0])**2 + (u[1]-v[1])**2)**0.5
                    if distances[v] > distances[u] + d:
                        predecessors[v] = u  # Update predecessor
                        distances[v] = d
        # construct the shortest path
        path = []
        node = nearest_goal_node
        while node is not None:
            path.insert(0, node)
            node = predecessors.get(node)
            self.get_logger().info(f"{path}")
        # self.get_logger().info(f"PATH: {path}")
        return path if path[0] == nearest_start_node else []
        
    def publish_map(self):
        self.map_msg.data = self.current_map
        # Update the timestamp in the header
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        # Publish the map
        self.map_publisher.publish(self.map_msg)
        
    #callback for when vl is received
    def vl_callback(self, msg):
        # self.get_logger().info(f"Current vl: {msg.data}")
        self.vl = msg.data
        self.last_velocity_update_time = self.get_clock().now()
        if self.error_left != 0.0:
            self.vl = self.vl * self.error_left

    #callback for when vl is received
    def vr_callback(self, msg):
        # self.get_logger().info(f"Current vr: {msg.data}")
        self.vr = msg.data
        self.last_velocity_update_time = self.get_clock().now()
        if self.error_right != 0.0:
            self.vr = self.vr * self.error_right

    # Timer callback function        
    def timer_callback(self):
         if self.path:
             self.follow_path()
         if self.reached_goal and not self.angle_corrected:
             self.correct_angle()
         
         # Calculate elapsed time since last velocity update
         current_time = self.get_clock().now()
         elapsed_time_velocity = (current_time - self.last_velocity_update_time).nanoseconds / 1e9
         elapsed_time_error = (current_time - self.last_error_update_time).nanoseconds / 1e9
         
         # If no velocity command received for 1 second, stop the robot
         if elapsed_time_velocity > 1.0:
             self.vl = 0.0
             self.vr = 0.0
             
         # Update the errors if the error_update_rate is reached
         if elapsed_time_error >= self.error_update_rate:
             self.update_errors()
         self.update_robot_pose(elapsed_time_velocity)
             
        # Broadcast the transform
         self.broadcast_transform()
         
    def update_errors(self):
        # Update the errors for left and right wheels with random values
        self.error_left = random.gauss(1.0, self.error_variance_left)
        self.error_right = random.gauss(1.0, self.error_variance_right)
        self.last_error_update_time = self.get_clock().now()
        # self.get_logger().info(f'error_left = {self.error_left}, error_right = {self.error_right}')
         
    def update_robot_pose(self, time):
        # Update robot pose based on velocities and errors
        if(self.vl != 0.0 or self.vr != 0.0):
            # Update robot pose only if it is moving
            omega = (self.vr - self.vl) / self.distance
            omega_t = omega * 0.1
            if self.vl == self.vr:
                R = 10000000000.0 # icc at infinity
            elif self.vl == -self.vr: 
                R = 0
            else:
                R = (self.distance / 2) * (self.vr + self.vl) / (self.vr - self.vl)

            c = [self.pose['x'] - R * math.sin(self.pose['theta']), self.pose['y'] + R * math.cos(self.pose['theta'])]
            cos_omega_t = math.cos(omega_t)
            sin_omega_t = math.sin(omega_t)

            updated_x, updated_y, updated_theta = self.pose['x'], self.pose['y'], self.pose['theta']

            if self.vl == self.vr:
                # If the velocities are equal, then this is the equation we use
                updated_x =  self.pose['x'] + self.vl * math.cos(self.pose['theta'])*0.1
                updated_y =  self.pose['y'] + self.vl * math.sin(self.pose['theta'])*0.1
            else:
                # If the velocities are not equal
                updated_x = cos_omega_t * (self.pose['x'] - c[0]) - sin_omega_t * (self.pose['y'] - c[1]) + c[0]
                updated_y = sin_omega_t * (self.pose['x'] - c[0]) + cos_omega_t * (self.pose['y'] - c[1]) + c[1]
                updated_theta = self.pose['theta'] + omega_t
            if not self.is_collision(updated_x, updated_y):
                self.pose['x'] = updated_x
                self.pose['y'] = updated_y
                self.pose['theta'] = updated_theta
                # self.broadcast_transform()
            else:
                self.get_logger().info("collision detected. not updating pose")
                self.vl = 0.0
                self.vr = 0.0
            # self.get_logger().info(f"x : {self.pose['x']}, y : {self.pose['y']}, theta : {self.pose['theta']}")
       
    def broadcast_transform(self):
        # Broadcast the transform from world to base_link
	    # Get the current time and set it as the stamp for the transform
        self.base_link_transform.header.stamp = self.get_clock().now().to_msg()
	    
	# Set the translation values of the transform
        self.base_link_transform.transform.translation.x = self.pose['x']
        self.base_link_transform.transform.translation.y = self.pose['y']

	# Calculate quaternion values for the rotation from the robot's theta angle
        quat_tf = [0.0, 0.0, math.sin(self.pose['theta']/2), math.cos(self.pose['theta']/2)]
        quaternion = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])

	# Set the rotation of the transform using the calculated quaternion
        self.base_link_transform.transform.rotation = quaternion

	# Send the transform using the TF2 Broadcaster
        self.tf_broadcaster.sendTransform(self.base_link_transform)

    def is_collision(self, x, y):
        # This returns true if the new x and y would yield a collision
        # self.get_logger().info("Checking for collision")
        # This looks at all the points around the robot and checks if they would collide with an obstacle
        for angle in range(360):
            new_y = y
            new_x = x
            current_rad = angle * (math.pi /180)
            new_y += math.sin(current_rad) * self.radius
            new_x += math.cos(current_rad) * self.radius
            resolution = self.map_msg.info.resolution
            map_x_index = int(math.floor(new_x / resolution))
            map_y_index = int(math.floor(new_y / resolution))
            if 0 <= map_x_index < self.map_msg.info.width and 0 <= map_y_index < self.map_msg.info.height:
                map_cell =  self.two_d_map[map_y_index][map_x_index]
            else:
                map_cell = 100
            # Check for obstacles
            if map_cell == 100:
                return True
        return False
            
def main(args=None):
    rclpy.init(args=args)
    simulator_node = SimulatorNode()
    rclpy.spin(simulator_node)
    simulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

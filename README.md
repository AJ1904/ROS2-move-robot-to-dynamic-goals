# ROS2-move-robot-to-dynamic-goals
Software to navigate robot to multiple goal positions on the go.



https://github.com/user-attachments/assets/74956f01-d0f9-42f4-bc09-1ab837f51a4d



https://github.com/user-attachments/assets/400f711b-ed0b-401d-bff7-5c4888894e89


## How the Program Works
The launch file is used to launch the nodes that allow the project to run. It takes in the command line arguments:  
- **`robot_file`**: Determines the robot file needed to get the information about the current robot.  
- **`world_file`**: Determines the world description file to be used.  

It runs the built-in ROS2 node `robot_state_publisher` as well as custom nodes:  
- `simulator_node`  
- `velocity_translator_node`

### Process Overview:
1. The program starts by reading the world description file to create a 2D map of the world.
2. Using the 2D map, a graph is created where empty cells act as nodes. Nearby nodes are identified, and edges are created between them if they don’t intersect any obstacles.
3. When a goal pose is posted using 2D goal pose in `rviz2`, the `goal_pose_callback` function in `simulator_node` is triggered:
   - If the goal pose is inside an obstacle, it is discarded.
   - If the goal pose is directly reachable from the current location, the robot follows a straight-line path (no Dijkstra algorithm performed, path is empty).
   - Otherwise:
     - **Nearest Start Node**: The nearest node to the robot's current location is identified.
     - **Nearest Goal Node**: The nearest node to the desired goal location is identified.
     - Dijkstra's algorithm is executed to find a path between the nearest start node and the nearest goal node.
     - The goal pose is appended to the path.

4. The `simulator_node` publishes `Twist` messages on `/cmd_vel` to:
   - Move the robot to the nearest start location.
   - Follow the generated path.
   - Move from the nearest goal node to the actual goal location.
   - Perform angle correction to align with the goal pose.
5. The robot stops after reaching the goal. If a new goal pose is published while the robot is moving, the current path is discarded, and a new path is generated.

### Sample Command to Run the Code:
```bash
ros2 launch project4d start.launch.py world_file:=src/project4d/world_files/maze.world robot_file:=src/project4d/robot_files/normal.robot
```

---

## 1. `start.launch.py`

This file launches the nodes required to create the simulator. The robot's description is loaded from a specified robot file using the `load_disc_robot` module.

### Key Components:
- **Launch Arguments**:
  - `robot_file`: Declared as a launch argument for use within the simulator node and `robot_state_publisher`.
  - `world_file`: Declared as a launch argument.
  - Both arguments are accessed using `sys.argv` or `DeclareLaunchArgument`.

- **`load_disc_robot`**:
  - Retrieves the robot's information from the robot file and passes it to the other nodes.

- **Robot State Publisher Node**:
  - Publishes the robot's state to the ROS2 ecosystem using its URDF description (unmodified built-in node).

- **Simulator Node**:
  - Simulates the robot’s behavior based on parameters from the robot file (e.g., radius, height, wheel distance, and error characteristics).
  - Subscribes to `/vl` and `/vr` (left and right wheel velocities) and broadcasts the robot's transform based on these values.
  - Publishes the environment map on `/map`.
  - Stops the robot if it moves into an obstacle.

- **Velocity Translator Node**:
  - Translates `Twist` commands into the velocities of the left and right wheels of the differential drive robot.

---

## 2. `velocity_translator_node.py`

This node subscribes to `Twist` messages on `/cmd_vel` and calculates the velocities of the left and right wheels. It publishes the calculated values as `Float64` messages to `/vl` and `/vr`.

### Key Components:
- Declares parameters:
  - `distance`: Distance between the robot's wheels.
  - `radius`: Radius of the wheels.
- Subscribes to `/cmd_vel` for `Twist` messages.
- The `cmd_vel_callback` function:
  - Retrieves linear and angular velocities.
  - Calculates `vr` (right wheel velocity) and `vl` (left wheel velocity).
  - Publishes the velocities to `/vr` and `/vl`.

---

## 3. `simulator_node.py`

This node simulates the robot's movement, generates paths to the goal, and enables the robot to follow the paths.

### Key Components:
- **Subscriptions**:
  - `/vl` and `/vr`: Updates the robot's velocities.
  - Timer (0.1s interval) triggers `timer_callback` to handle all other functions.
- **Functions**:
  - **`update_errors`**: Calculates error multipliers using random Gaussian functions.
  - **`update_robot_pose`**: Determines pose changes using differential drive equations. Calls `is_collision` to ensure no collisions.
  - **`broadcast_transform`**: Updates the robot's position using a tf2 broadcaster.
  - **`get_map_data`**: Reads the world file, initializes the pose, and creates the occupancy grid.
  - **`publish_map`**: Publishes the map as an occupancy grid every 2 seconds.
  - **`is_collision`**: Checks for collisions at a given pose.
  - **Graph Construction**:
    - **`create_graph`**: Creates a graph of empty cells as nodes, connecting neighbors with collision-free edges.
    - **`find_neighbors`**: Identifies valid neighboring cells for a given node.
    - **`path_passes_through_obstacle`**: Ensures paths do not intersect obstacles.
  - **Goal Handling**:
    - **`goal_pose_callback`**: Processes new goal poses, generates paths, and handles Dijkstra's algorithm.
    - **`follow_path`**: Controls movement along the path.
    - **`correct_angle`**: Adjusts the robot's orientation to align with the goal pose.
  - **Pathfinding**:
    - **`find_nearest_node`**: Finds the closest graph node to a given pose.
    - **`generate_path`**: Implements Dijkstra's algorithm to find the shortest path between nodes.

---

## Did the Results Meet Expectations?

The results met expectations on average. Challenges included:
- Determining correct velocities for publishing.
- Generating collision-free edges.
- Avoiding unnecessary paths (caused by a basic pathfinding algorithm).  

However, the robot:
- Reached the goal pose successfully.
- Discarded goal poses that were in obstacles or unreachable.

---

## Credit

Part of this document was based on previous submissions by Group 11 (Jonas Land and Ayushri Jain).

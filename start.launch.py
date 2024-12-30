## Part of this code is contributed by Group 11 member: Jonas Land
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable, LaunchConfiguration, LocalSubstitution, PythonExpression)
import sys
import json
from launch_ros.parameter_descriptions import ParameterValue
from project4d.disc_robot import load_disc_robot, disc_robot_urdf
import yaml
        
def generate_launch_description():
    # Extracting robot_file from command line arguments
    for arg in sys.argv:
        if arg.startswith("robot_file:="):
            robot_file = str(arg.split(":=")[1])
            break
        else:
            robot_file = 'project4d/robot_files/normal.robot'
            
    # Load robot parameters from the specified file
    robot = load_disc_robot(robot_file)
    robot_desc = robot['urdf']
    radius = robot['body']['radius']
    height = robot['body']['height']
    distance = robot['wheels']['distance']
    error_variance_left = robot['wheels']['error_variance_left']
    error_variance_right = robot['wheels']['error_variance_right']
    error_update_rate = robot['wheels']['error_update_rate']
    
    # Parameters for laser
    laser_rate = robot['laser']['rate']
    laser_count = robot['laser']['count']
    laser_angle_min = robot['laser']['angle_min']
    laser_angle_max = robot['laser']['angle_max']
    laser_range_min = robot['laser']['range_min']
    laser_range_max = robot['laser']['range_max']
    laser_error_variance = robot['laser']['error_variance']
    laser_fail_probability  = robot['laser']['fail_probability']
    
    # LaunchDescription object
    ld = LaunchDescription(
        [
        

        DeclareLaunchArgument(
            'world_file',
            default_value='project4d/world_files/brick.world',
            description='World file'
        ),
        # declares the robot_file parameter as a command line argument
        DeclareLaunchArgument(
            'robot_file',
            default_value='project4d/robot_files/normal.robot',
            description='Input robot file'
        ),
        ]
    )
  
    # Simulator Node
    simulator_node = Node(
        package='project4d',
        executable='simulator_node',
        parameters=[{
            'world_file': LaunchConfiguration('world_file'),
            'robot_description': robot_desc, 
        	'radius': radius,
        	'height': height,
        	'distance': distance,
        	'error_variance_left': error_variance_left,
        	'error_variance_right': error_variance_right,
        	'error_update_rate': error_update_rate
        	
        	}]
        )

    # # Velocity Translator Node
    velocity_translator_node = Node(
         package='project4d',
         executable='velocity_translator_node',
         parameters=[{'robot_description': robot_desc,
         	'radius': radius,
        	'distance': distance}]
         )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
     	package='robot_state_publisher',
     	executable='robot_state_publisher',
     	name='robot_state_publisher',
     	namespace='',
     	output='screen',
     	parameters=[{'robot_description': robot_desc}]
     	)

    
    
    # Add actions to the LaunchDescription
    
    ld.add_action(simulator_node)
    ld.add_action(velocity_translator_node)
    ld.add_action(robot_state_publisher_node)

    return ld

    

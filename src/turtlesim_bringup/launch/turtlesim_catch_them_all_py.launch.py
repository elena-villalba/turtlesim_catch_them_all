from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Path to the parameter configuration file (YAML)
    param_config = os.path.join(
        get_package_share_directory("turtlesim_bringup"),
        "config", 
        "turtlesim_catch_them_all.yaml")    
    
    # Turtlesim graphical simulation node (graphic window)
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    
    # Node that controls the turtle to catch others
    turtle_controller = Node(
        package="turtlesim_catch_them_all_py",
        executable="turtle_controller",
        # parameters=[
        #     {"catch_closest_turtle_first": False}
        # ]
        parameters=[param_config]
    )
    
    # Node that spawns turtles at random positions
    turtle_spawner = Node(
        package="turtlesim_catch_them_all_py",
        executable="turtle_spawner",
        # parameters=[
        #     {"spawn_frequency": 2.0}
        # ]
        parameters=[param_config]
    )

    # Return all nodes directly in the LaunchDescription
    return LaunchDescription([
        turtlesim_node,
        turtle_controller,
        turtle_spawner
    ])
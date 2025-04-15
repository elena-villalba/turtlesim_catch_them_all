# turtlesim_catch_them_all
This project implements a ROS 2 simulation using the `turtlesim` package, where a turtle (`turtle1`) "catches" other turtles that are randomly spawned by moving toward them and removing them from the screen once caught.

<!-- ![demo](path/to/your/demo.gif) Replace with the actual path to your GIF -->

## Requirements
- Ubuntu 24.04 LTS
- ROS 2 Jazzy 

## Installation

Clone the repository:

```bash
git clone git@github.com:elena-villalba/turtlesim_catch_them_all.git
```

Build the workspace:

```bash
cd turtlesim_catch_them_all
colcon build
```
Source the workspace:

```bash
source install/setup.bash
```

Launch the simulation using python or xml launch:

```bash
ros2 launch turtlesim_bringup turtlesim_catch_them_all_py.launch.py
```
```bash
ros2 launch turtlesim_bringup turtlesim_catch_them_all_py.launch.xml
```

## Modifying Parameters in the YAML File
For more information about configurable options, refer to the [Parameters](#parameters) section.
1. Open the `turtlesim_catch_them_all.yaml` file located in the `turtlesim/config/` folder.
2. Modify the parameters as needed. 
3. Save the file.
4. Rebuild the workspace with `colcon build`.
5. Source the environment (`source /install/setup.bash`).
6. Launch the simulation again to apply the new parameters.

## Feauters

### 🧠 Nodes

The project includes three nodes:

1. **`turtlesim_node`:** 
    - **Description:** This node is from the `turtlesim` package and provides the graphical simulation where the turtles are displayed. It is responsible for spawning the initial `turtle1` and handling the core simulation environment.
    - **Server Services:** 
        - `/spawn`: Used to create new turtles at specific positions.
        - `/kill`: Used to remove turtles from the screen.
    - **Topics:**
        - `/turtle1/pose`: Publishes the current position and orientation of `turtle1`. 
        - `/turtle1/cmd_vel`: Subscribed to by `turtlesim_node` to receive velocity commands for `turtle1`. 

2. **`turtle_spawner`:** 
    - **Description:** This custom node handles the spawning of new turtles at random positions and manages the "alive" turtles in the simulation. 
    - **Client Services:** 
        - `/spawn`: Used to create new turtles at specific positions.
        - `/kill`: Used to remove turtles from the screen.
    - **Server Services:** 
        - `/catch_turtle`: A custom service that is used to remove a turtle from the list of alive turtles and the screen, based on requests from the `turtle_controller` node.
    - **Topic:**
        - `/alive_turtles`: Publishes a list of all currently alive turtles, including their names and coordinates.

3. **`turtle_controller`:** 
    - **Description:** This custom node controls the movement of the "master" turtle (`turtle1`). It runs a simplified proportional (P) controller to move towards and catch other turtles by reaching their positions. The node selects the next turtle to catch from the list of alive turtles, based on predefined strategies.
    - **Client Service:** 
        - `/catch_turtle`: Called to remove a turtle once it has been caught by `turtle1`. 
    - **Server Services:** 
        - `/catch_turtle`: A custom service that is used to remove a turtle from the list of alive turtles and the screen, based on requests from the `turtle_controller` node.
    - **Topics:**
        - `/turtle1/pose`: Subscribed to in order to track the current position and orientation of `turtle1`.
        - `/turtle1/cmd_vel`**: Publishes velocity commands to control the motion of `turtle1`.
        - `/alive_turtles`: Subscribed to in order to retrieve the list of turtles currently on the screen.

### 💬 Custom interfaces

The project uses custom messages and services for communication between nodes:

1. `Turtle.msg`:
    - This message format is used to represent a turtle's data (name, x and y coordinates, and orientation).
   
2. `TurtleArray.msg`:
    - This message format holds an array of `Turtle` messages, representing all the currently alive turtles with their positions. It is published by the `turtle_spawner` node on the `/alive_turtles` topic.

3. `CatchTurtle.srv`:
    - This custom service allows the `turtle_controller` node to request the removal of a turtle from the simulation. It accepts the name of the turtle to be removed and responds with a success status.

### ⚙️ Parameters:

Parameters for `turtle_spawner` (node spawning turtles):
- **`spawn_frequency`:**
    - **Type**: `float`
    - **Default**: `1.0`
    - **Description**: Defines the frequency at which new turtles are spawned, in Hz. A higher value will result in more turtles being spawned per second.

Parameters for `turtle_controller` (node controlling `turtle1`):
- **`catch_closest_turtle_first`:**
    - **Type**: `bool`
    - **Default**: `True`
    - **Description**: When set to `True`, the controller will prioritize catching the closest alive turtle. If set to `False`, the controller will catch the turtle that was first spawn. 

## Future Work
- Add a C++ package with similar functionality to the current Python implementation.
- Improve the control loop in turtle_controller.py for more advanced turtle behavior.

## License
This project is licensed under the Apache 2.0 License.
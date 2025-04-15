#!/usr/bin/env python3

import random
import math
from functools import partial
import rclpy 
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim_interfaces.msg import Turtle
from turtlesim_interfaces.msg import TurtleArray
from turtlesim_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        
        # Declare and retrieve parameters
        self.declare_parameter("spawn_frequency",1.0)
        spawn_frequency = self.get_parameter("spawn_frequency").value 
        
        # Internal global variable
        self.alive_turtles_ = {}  # Alive turtles dictionary: name → [x, y, theta]

        # Create clients, publisher and server
        self.spawn_client_ = self.create_client(Spawn, "spawn")
        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray,"alive_turtles",10)
        self.catch_turtle_server_ = self.create_service(
            CatchTurtle,"catch_turtle",self.callback_catch_turtle)
        self.kill_client_ = self.create_client(
            Kill, "kill")
        
        # Timer to spawn a new turtle periodically
        self.spawn_turtle_timer_ = self.create_timer(
            (1.0 / spawn_frequency), self.spawn_new_turtle)
        
        self.get_logger().info("Turtle Spawner Node has started with a spawn frequency of" + str(spawn_frequency) + "Hz.")

    def spawn_new_turtle(self):
        """Generate random position and orientation, then request to spawn a new turtle."""
        x = random.uniform(0.0, 11.0)  
        y = random.uniform(0.0, 11.0)  
        theta = random.uniform(0.0, 2 * math.pi) 
        
        self.call_spawn_service(x, y, theta)

    def call_spawn_service(self, x, y, theta):
        """Send request to the Spawn service to create a new turtle."""
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn server to become available...")

        # Create and send spawn request
        request = Spawn.Request()
        request.x = x 
        request.y = y 
        request.theta = theta

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_spawn_service, request=request))
    
    def callback_call_add_spawn_service(self, future, request: Spawn.Request):
        """Handle the response from the spawn service after attempting to create a turtle."""
        response: Spawn.Response = future.result()
        
        if response.name != "": # if it is an error, the name is not provide
            # Turtle successfully spawned
            name = response.name
            x = request.x
            y = request.y
            theta = request.theta
            self.alive_turtles_[name] = [x, y, theta] # Add new alive turtle at the end
            self.publish_alive_turtles()
        else:
            # Failed to spawn turtle
            self.get_logger().error("Failed to spawn a new turtle.") 

    def publish_alive_turtles(self):
        """Publish the current list of alive turtles to a custom message topic."""
        msg = TurtleArray()
        for name, (x, y, theta) in self.alive_turtles_.items():
            turtle = Turtle()
            turtle.name = name
            turtle.x = x
            turtle.y = y
            turtle.theta = theta
            msg.turtles.append(turtle)

        self.alive_turtles_publisher_.publish(msg)
   
    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        """Callback for the CatchTurtle service. Attempts to kill the requested turtle."""
        turtle_name = request.name
        
        if turtle_name not in self.alive_turtles_:
            self.get_logger().warn(f"Received request to catch non-existent turtle '{turtle_name}'")
            response.success = False
            return response

        self.call_kill_service(turtle_name)
        response.success = True        
        return response
    
    def call_kill_service(self, turtle_name):
        """Send request to Kill service to remove the turtle."""
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Kill server to become available...")

        request = Kill.Request()
        request.name = turtle_name
        
        future = self.kill_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_service, turtle_name=turtle_name))
    
    def callback_call_kill_service(self, future, turtle_name):
        """Callback for when the kill service completes. Removes the turtle from the list."""
        if turtle_name in self.alive_turtles_:
            self.alive_turtles_.pop(turtle_name)
            self.publish_alive_turtles()
        else:
            self.get_logger().warn(f"Attempted to remove unknown turtle '{turtle_name}'.")

def main(args=None):
    rclpy.init(args=args) 
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown() 

# Entry point for the script
if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import math
from functools import partial
import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_interfaces.msg import Turtle
from turtlesim_interfaces.msg import TurtleArray
from turtlesim_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller") 
        
        # Declare parameters
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value
        
        # Internal global variables
        self.turtle_current_pose_: Pose = None # x, y, theta
        self.alive_turtles_ = {}
        self.turtle_to_catch_: Turtle = None
         
        # Create subscriptions, publisher and service client
        self.turtle_pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtles_sub_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles,10)
        self.turtle_cmd_vel_pub_ = self.create_publisher(
            Twist,"/turtle1/cmd_vel", 10)
        self.catch_turtle_client_ = self.create_client(
            CatchTurtle, "catch_turtle")
        
        # Timer for control loop
        self.control_loop_timer_ = self.create_timer(
            0.01, self.control_loop)

        self.get_logger().info("Turtle Controller Node has started.")
    
    def callback_turtle_pose(self, msg:Pose): 
        """Callback to update the current position of turtle1."""
        self.turtle_current_pose_ = msg
    
    def callback_alive_turtles(self, msg: TurtleArray):
        """Callback to update the list of alive turtles and choose which one to catch."""
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                # Find the closest turtle
                closest_turtle = None
                min_distance = float('inf')

                for turtle in msg.turtles:
                    distance = math.sqrt(
                        (turtle.x - self.turtle_current_pose_.x)**2 +
                        (turtle.y - self.turtle_current_pose_.y)**2
                    )
                    if distance < min_distance:
                        min_distance = distance
                        closest_turtle = turtle
                self.turtle_to_catch_ = closest_turtle
                
            else:
                # Select the first turtle in the list
                self.turtle_to_catch_ = msg.turtles[0]
        else:
            self.get_logger().warn("No alive turtles at the moment.")
            self.turtle_to_catch_ = None 
    
    def control_loop(self):
        """Main control loop for moving turtle1 toward the target turtle."""
        if ((self.turtle_current_pose_ == None) or (self.turtle_to_catch_ == None)):
            return

        # Compute distance and angle to target
        dist_x = self.turtle_to_catch_.x - self.turtle_current_pose_.x
        dist_y = self.turtle_to_catch_.y - self.turtle_current_pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        cmd = Twist()

        if distance > 0.5:
            # Move towards the turtle
            cmd.linear.x = 2*distance

            # Rotate towards the turtle
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.turtle_current_pose_.theta
            
            # Normalize angle
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            cmd.angular.z = 6*diff
        else:
            # Close enough to catch
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
            
        self.turtle_cmd_vel_pub_.publish(cmd)
    
    def call_catch_turtle(self, turtle_name):
        """Call the catch_turtle service to remove the targeted turtle."""
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Catch Turtle server to become available...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_call_catch_turtle, turtle_name=turtle_name))
    
    def callback_call_call_catch_turtle(self, future, turtle_name):
        """Callback after attempting to call the CatchTurtle service."""
        response: CatchTurtle.Response = future.result()
        if not response.success:
            self.get_logger().error("Turtle " + turtle_name + "could no be remove")

def main(args=None):
    rclpy.init(args=args) 
    node = TurtleControllerNode() 
    rclpy.spin(node) 
    rclpy.shutdown() 

# Entry point of the script
if __name__ == "__main__":
    main()
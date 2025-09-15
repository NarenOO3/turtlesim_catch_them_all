import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import numpy as np
import math
from my_pkg_interfaces.msg import TurtleArray 
from my_pkg_interfaces.srv import CatchTurtle
from functools import partial # too add extra args to callback fn or fn is general


class TurtleController(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closet_turtle_first", True)
        self.catch_closest_turtle_first = self.get_parameter("catch_closet_turtle_first").value
        
        self.pose_: Pose = None
        self.twist_publisher_ = self.create_publisher(Twist, "autobot/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "autobot/pose", self.callback_pose_subscriber,10)
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles_subscriber,10)
        self.timer_ = self.create_timer(0.01, self.publish_turtle_controller)
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
        self.get_logger().info("turtle_controller has been Started...")
        self.turtle_to_catch_ = None

    def publish_turtle_controller(self):

        if self.pose_ == None:
            return       
         
        if self.turtle_to_catch_ == None:
            return   

        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)
        cmd = Twist()

        if distance >0.5:
            #linear

            cmd.linear.x = 2*distance
            
            #angular
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < - math.pi:
                diff += 2*math.pi
            cmd.angular.z = 6*diff

            
        else:
            # goal_reached

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            request = CatchTurtle.Request()
            request.name = self.turtle_to_catch_.name
            future =self.catch_turtle_client_.call_async(request)
            future.add_done_callback(partial(self.call_back_turtle_caught, request = request))
            self.turtle_to_catch_ = None
            

        self.twist_publisher_.publish(cmd)
        # self.get_logger().info("turtle_controller has been Started...")

    def call_back_turtle_caught(self, future, request):
        pass

    def callback_pose_subscriber(self, pose:Pose):
        self.pose_ = pose
        
    def callback_alive_turtles_subscriber(self, turtle_array: TurtleArray):
        if len(turtle_array.turtles) > 0:
            if self.catch_closest_turtle_first == True:
                closet_turtle = None
                closet_turtle_distance = None
                for turtle in turtle_array.turtles:
                        dist_x = turtle.x - self.pose_.x
                        dist_y = turtle.y - self.pose_.y  
                        dist = math.sqrt(dist_x**2 + dist_y**2)
                        if closet_turtle == None or  dist < closet_turtle_distance:
                            closet_turtle_distance = dist
                            closet_turtle = turtle
                            self.turtle_to_catch_ = closet_turtle
            else:           
                self.turtle_to_catch_ = turtle_array.turtles[0]

def main(args = None):
    
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
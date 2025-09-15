import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
from my_pkg_interfaces.msg import TurtleArray 
from my_pkg_interfaces.msg import Turtle
from turtlesim.srv import Kill
from my_pkg_interfaces.srv import CatchTurtle
from functools import partial # too add extra args to callback fn or fn is general
from turtlesim.msg import Pose
import math

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        
        self.declare_parameter("turtle_name_prefix", "turtle")
        self.declare_parameter("spawn_frequency", 1.0)
        
        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        self.spawn_client_ = self.create_client(Spawn, "spawn")
        self.pose_subscriber_ = self.create_subscription(Pose, "autobot/pose", self.callback_pose_subscriber,10)
        self.kill_client_ = self.create_client(Kill, "kill")
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
        self.timer_ = self.create_timer((1.0/self.spawn_frequency_), self.call_turtle_spawner)
        self.get_logger().info("turtle_spawner has been Started...")
        self.create_autobot()
    
    def create_autobot(self):
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Kill Service...")      
            
        request = Kill.Request()
        request.name = "turtle1"
        future =self.kill_client_.call_async(request)  
        future.add_done_callback(self.callback_call_autobot_spawn)
        
    def callback_call_autobot_spawn(self, future):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for callback_call_autobot_spawn...")
        request = Spawn.Request()
        request.name = "autobot"
        request.x = 5.544445
        request.y = 5.544445
        request.theta = 0.0
        future = self.spawn_client_.call_async(request)
        
    def callback_pose_subscriber(self, pose:Pose):
        self.pose_ = pose
        
    def callback_catch_turtle(self, request:CatchTurtle.Request, response:CatchTurtle.Response):
        #call kill service
        print("Hi"+request.name)
        self.call_kill_service(request.name)
        response.success = True
        return response
        
    def call_turtle_spawner(self):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for call_turtle_spawner...")
        
        self.turtle_counter_ +=1    
        request = Spawn.Request()
        request.name = self.turtle_name_prefix_+str(self.turtle_counter_)
        request.x = float(random.randint(1,10))
        request.y = float(random.randint(1,10))
        request.theta = float(random.randint(-4,4))
        future =self.spawn_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_turtle_spawner, request = request))
        
        
    def callback_turtle_spawner(self, future, request):
        response = future.result()
        if response.name != "":              
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            msg = TurtleArray()
            msg.turtles = self.alive_turtles_  
            self.alive_turtles_publisher_.publish(msg)
            self.min_dist_ = 20
            
            
    def call_kill_service(self, turtle_name):
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Kill Service...")      
            
        request = Kill.Request()
        request.name = turtle_name
        future =self.kill_client_.call_async(request)  
        future.add_done_callback(
            partial(self.callback_call_kill_service, request = request))
        
    def callback_call_kill_service(self, future, request: Kill.Request):
        for (i, turtle) in enumerate(self.alive_turtles_):

            if turtle.name == request.name:
                del self.alive_turtles_[i]
                msg = TurtleArray()
                msg.turtles = self.alive_turtles_
                self.alive_turtles_publisher_.publish(msg)
                break
        
        
        
        
def main(args = None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
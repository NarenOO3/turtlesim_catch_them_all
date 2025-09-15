# turtlesim_catch_them_all

An Autobot (turtlesim turtle 🤖) will catch all Decepticons (Enemy turtlesim turtles 👾) one by one in order or catch the one nearest to it to save planet earth ...
| Catch the First Spawned Deception          | Catch the Neartest Spawned Deception               |
| ---------------------- | ---------------------- |
| ![arena hola](https://github.com/NarenOO3/HolaBot/assets/98276114/e36fe4da-5eea-429b-982b-1ab001aa1460) | ![Hola bot and aruco markers](https://github.com/NarenOO3/HolaBot/assets/98276114/bddef082-c0a0-4953-84fb-bef05edb1719) |

A small project to make my transition from ROS1 to ROS2. Good thing is that concepts remains the same just need to get to use to updated function and classes in ROS2

# Working of Turtlesim Catch them all

We have three nodes turtlesim node, turtle_controller node, turtle_spawner node. The turtle_spawner node will spawn the Decepticons randomly in the Turtlesim and the turtle_controller node will control (nothing special, basic proportional controller) the Autobot to drive itself to the Decepticons and catch them once caught the deceptions are neutralised (removed from the turtlesim). We have custom message and services to spawn and neutraulise the deceptions

# rqt graph





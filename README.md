# turtlesim_catch_them_all...



An Autobot (turtlesim turtle 🤖) will catch all Decepticons (Enemy turtlesim turtles 👾) one by one in order or catch the one nearest to it to save planet earth ...

| Catch the First Spawned Deception          |      Catch the Neartest Spawned Deception               |
| ---------------------- | ---------------------- |
| ![catch the first spawned turtle](https://github.com/user-attachments/assets/2cafad9b-09ce-4dec-af96-381f3cd0fbb0)| ![catch the nearest turtle](https://github.com/user-attachments/assets/0a5040f4-8662-48d0-ac73-e7f8d575e240) |







A small project to make my transition from ROS1 to ROS2. Good thing is that concepts remains the same just need to get to use to updated function and classes in ROS2

## Working of Turtlesim Catch them all

We have three nodes turtlesim node, turtle_controller node, turtle_spawner node. The turtle_spawner node will spawn the Decepticons randomly in the Turtlesim and the turtle_controller node will control (nothing special, basic proportional controller) the Autobot to drive itself to the Decepticons and catch them once caught the deceptions are neutralised (removed from the turtlesim). We have custom message and services to spawn and neutralize the deceptions and parameters to change the spawn rate and catching mode (first spawned turtle catching mode or nearest turtle catching mode)

### rqt graph

<img width="2172" height="1476" alt="rqt_graph" src="https://github.com/user-attachments/assets/f352a5ac-8914-45f2-945a-1129ec0350ed" />




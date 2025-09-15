# turtlesim_catch_them_all...



An Autobot (turtlesim turtle 🤖) will catch all Decepticons (Enemy turtlesim turtles 👾) one by one in an order or catch the one nearest to it to save planet Earth 🌍...

| Catch the First Spawned Deception          |      Catch the Neartest Spawned Deception               |
| ---------------------- | ---------------------- |
| ![catch the first spawned turtle](https://github.com/user-attachments/assets/2cafad9b-09ce-4dec-af96-381f3cd0fbb0)| ![catch the nearest turtle](https://github.com/user-attachments/assets/0a5040f4-8662-48d0-ac73-e7f8d575e240) |

A small project to help with my transition from **ROS1** to **ROS2**. The good thing is that the core concepts remain the same — I just need to get used to the updated functions and classes in ROS2.

## Working of Turtlesim Catch them all

We have three main nodes in this project: **turtlesim node**, **turtle_controller node**, and **turtle_spawner node**.

- The **turtle_spawner node** randomly spawns the Decepticons in Turtlesim.  
- The **turtle_controller node** acts as the Autobot, using a basic proportional controller to drive toward the Decepticons. Once a Decepticon is caught, it is neutralized (removed from Turtlesim).  

We also define **custom messages and services** to handle spawning and neutralizing Decepticons. Additionally, parameters are provided to:  

- Adjust the **spawn rate** of Decepticons.  
- Select the **catching mode**: either *first-spawned turtle mode* or *nearest-turtle mode*.  

The interaction between these nodes and topics can be visualized using **rqt_graph**, as shown below:

<img width="2172" height="1476" alt="rqt_graph" src="https://github.com/user-attachments/assets/f352a5ac-8914-45f2-945a-1129ec0350ed" />

You can get the complete code from the files in this repository.  

This project is a fun way to learn, practice, and strengthen my understanding of ROS2 while building on familiar ROS1 concepts🚀...


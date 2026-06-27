
## Objective 

Develop a simulation of a drone swarm capable of performing autonomous search and rescue operations using ROS 2 and Gazebo.  The system must allow dozens of drones to cooperate with one another without relying on a central controller, exploring the area, detecting victims, avoiding obstacles, and reorganizing the mission should a drone fail. 

## Scenario 

Imagine a city struck by an earthquake. 

A rescue team deploys 30 drones. 

The drones do not have a map of the area. 

They need to: 

- explore the environment; 

- avoid buildings; 

- avoid collisions; 

- search for victims; 

- share information; 

- decide which drone investigates each location; 

- return when their batteries run low. 

All of this without a central computer telling each drone exactly what to do. 

This is a classic problem in distributed robotics. 

## Technologies 

Python, ROS 2, Gazebo, RViz. Future use of C++ for critical modules. 

## Project structure 

```
SwarmSAR/

src/

    communication/

    navigation/

    formation/

    mapping/

    planning/

    search/

    battery/

    collision/

    simulator/

config/

launch/

worlds/

models/

results/

docs/

README.md
```


## Architecture 

Each drone has independent modules: Navigation, Communication, Planning, Mapping, Sensors, Battery Management, Collision Avoidance, and Search. 

## Sensors

Simulations:

GPS

IMU

LiDAR

Camera

Victim detector
## Communication 

Each drone transmits ID, position, battery level, local map, and found victims. There is no central controller. 

## Interface

```
Mission

Time

Number of drones

Active drones

Returning drones

Victims found

Area coverage

Map

Flight paths
```

## Algorithms 

Boids (collective behavior), A* (pathfinding), Frontier Exploration (exploration), Consensus (map sharing), and Auction Algorithm (task allocation). 

## Features 

Cooperative exploration, victim search, battery management, task redistribution, flight formation, fault detection, and metric generation. 

## Metrics 

Mission time, area coverage, energy consumption, collisions avoided, messages exchanged, and efficiency. 

## Organization in modules

```
Drone

│

├── Navigation

├── Communication

├── Battery

├── Planner

├── Mapping

├── Sensors

├── Collision

├── Formation

└── Search
```

## Extras 

Computer vision for victim detection, web dashboard, algorithm comparison, and future integration with physical drones. 

## Suggested Timeline

| Phase | Objective                                |
| ----- | ---------------------------------------- |
| 1     | Set up ROS 2, Gazebo, and a single drone |
| 2     | Multiple drones and communication        |
| 3     | Coordination and exploration             |
| 4     | Search, battery, and fault handling      |
| 5     | Dashboard, metrics, and documentation    |



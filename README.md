# UR5 Autonomous Pick-and-Place — ROS2

Autonomous pick-and-place system for a **UR5 robotic arm**, developed as part
of the Intelligent Robotics course. The robot identifies two AprilTag-marked
cubes (red and blue) in a Gazebo simulation and swaps their positions while
avoiding obstacles.

## Features
- AprilTag-based cube detection and pose estimation
- Autonomous pick-and-place with collision avoidance
- Position swapping of two target objects in simulation

## Technologies
- ROS2
- UR5 Robotic Arm
- MoveIt2
- AprilTag
- Gazebo (simulation environment provided by course instructor)

## Report
The full project report is available [here](docs/report.pdf)

## Demo
https://github.com/user-attachments/assets/b96614c1-4ce8-48b5-b87f-1d220578440b

[Watch high quality version on Google Drive](https://drive.google.com/file/d/1lhawb0vihj1RL_NqCUTKuvsUHr9nxyFX/view?usp=sharing)

## Setup

> This project requires a Gazebo simulation environment provided by the
> course instructor (not included in this repository).

1. Clone this repository inside your workspace:
```bash
   cd ~/ws_18_assignments/src
   git clone https://github.com/leopont36/ur5_pick_and_place.git
```
2. Build the packages:
```bash
   cd ..
   colcon build --packages-select group18_assignment_2
```
3. Run the launch file:
```bash
   source install/setup.bash
   ros2 launch group18_assignment_2 group18_assignment_2.launch.py
```

## Contributors
Developed in collaboration with
[@marcofacco2001](https://github.com/marcofacco2001) and
[@giaco-mas](https://github.com/giaco-mas)
as part of a university group assignment.

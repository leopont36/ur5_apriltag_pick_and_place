# Intelligent robotics assignment 2 - group n. 18
University project to implement a swapping system for the UR5 robot to exchange the positions of two cubes using MoveIt.
Participants:
- Marco Facco (@marcofacco2001) - ID number: 2140334
- Leonardo Pontello (@leopont36) - ID number: 2146898
- Giacomo Masiero (@giaco-mas) - ID number: 2130974

## Setup
1. Clone this repository inside the `/ws_18_assignments/src` folder:
  ```bash
  cd ~/ws_18_assignments/src
  git clone git@github.com:marcofacco2001/group18_assignment_2.git
  ```
2. Compile the packages:
  ```bash
  cd ..
  colcon build --packages-select group18_assignment_2
  ```
3. Run launch file:
  ```bash
  source install/setup.bash
  ros2 launch group18_assignment_2 group18_assignment_2.launch.py
  ```

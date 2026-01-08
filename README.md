# Intelligent robotics assignment 2 - group n. 18
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

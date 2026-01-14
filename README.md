# Intelligent robotics assignment 2 - group n. 18

**Autonomous Pick-and-Place Swapping System for UR5 Robot**

Solution for the second assignment of the Intelligent Robotics course (Group 18). The project implements an autonomous **Pick-and-Place** system for a **UR5 robotic arm**. The primary objective is to identify two Apriltag-marked cubes (red and blue) within a Gazebo simulation and robustly swap their positions while avoiding obstacles.

###  Project Demo
https://github.com/user-attachments/assets/b96614c1-4ce8-48b5-b87f-1d220578440b

_Video compressed for preview. [**Click here to watch the high quality version on Google Drive**](https://drive.google.com/file/d/1lhawb0vihj1RL_NqCUTKuvsUHr9nxyFX/view?usp=sharing)._

### Installation and setup
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

### Team members:
- **Marco Facco** (@marcofacco2001) - ID number: 2140334
- **Leonardo Pontello** (@leopont36) - ID number: 2146898
- **Giacomo Masiero** (@giaco-mas) - ID number: 2130974

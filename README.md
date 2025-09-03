# UR5_AMR_CL_1

## Procedure to Run

1. **Clone the repository**
   ```bash
   git clone git@github.com:vishalnaik37/UR5_AMR_CL_1.git

(or use HTTPS if you prefer)

2. Navigate to the workspace
   ```bash
   cd ~/UR5_AMR_CL_1

3. Build the workspace and source the setup file
   ```bash
   colcon build
   ```bash
   source install/setup.bash

4. Launch the UR5 controller in Gazebo

   ros2 launch robotic_arms_control controller_spawner_ur5.launch.py

5. Open another terminal, source the workspace again:

   cd ~/UR5_AMR_CL_1
   source install/setup.bash

6. Run the trajectory publisher node

   ros2 run robotic_arms_control jtc_multi_goals

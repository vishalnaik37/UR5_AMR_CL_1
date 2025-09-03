# UR5_AMR_CL_1

## Procedure to Run

1. **Clone the repository**
   ```bash
   git clone git@github.com:vishalnaik37/UR5_AMR_CL_1.git

(or use HTTPS if you prefer)

2. Navigate to the workspace
   ```bash
   cd ~/UR5_AMR_CL_1

3. Open the URDF file and edit the Gazebo plugin section:
    ```bash
   gedit /home/<user_name>/UR5_AMR_CL_1/src/robotic_arms_control/robotic_arms_control/urdf/ur5.urdf
Add/verify the following inside <gazebo> tags:
     ```bash
   <gazebo>   
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>
      /home/vishal/UR5_AMR_CL_1/src/robotic_arms_control/robotic_arms_control/config/ur5_itc_controller.yaml
    </parameters>
  </plugin>
</gazebo>

Important: Ensure that the path in <parameters> is an absolute path on your system.


4. Build the workspace and source the setup file
   ```bash
   colcon build
   source install/setup.bash

5. Launch the UR5 controller in Gazebo
    ```bash
   ros2 launch robotic_arms_control controller_spawner_ur5.launch.py

6. Open another terminal, source the workspace again:
    ```bash
   cd ~/UR5_AMR_CL_1
   source install/setup.bash

7. Run the trajectory publisher node
    ```bash
   ros2 run robotic_arms_control jtc_multi_goals

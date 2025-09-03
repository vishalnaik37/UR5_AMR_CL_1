import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from spatialmath.base import*
from spatialmath import SE3
# Load your custom URDF file
robotic_arm = ERobot.URDF(
    "/home/vishal/UR5_robot/src/robotic_arms_control/robotic_arms_control/urdf/ur5.urdf"
)

# Print robot details
print(robotic_arm)

# Forward Kin
fk=robotic_arm.fkine([0.1,0.2,0.4,0.3,0.4,0.1]) 
print(fk)


points=SE3(0.6642,0.2525,0.6993)
ik=robotic_arm.ikine_LM(points) 
print(ik)

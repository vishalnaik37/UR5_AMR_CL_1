import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vishal/UR5_AMR_CL_1/install/robotic_arms_control'

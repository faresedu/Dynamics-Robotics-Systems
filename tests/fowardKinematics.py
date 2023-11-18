# import roboticstoolbox as rtb
# import numpy as np
# from spatialmath import SE2
# import matplotlib.pyplot as plt
# from roboticstoolbox.backends.PyPlot import PyPlot

# a1 = 50; a2 = 60

# # the empty parentheses have been replaced with sequential joint variables q0 and q1
# # e = rtb.ET2.R() * rtb.ET2.tx(a1) * rtb.ET2.R() *  rtb.ET2.tx(a2)
# # print(e)

# # e.fkine(np.deg2rad([30, 45])).printline()

# # # the equivalent is to use the SE2 class
# # k = SE2.Rot(np.deg2rad(30)) * SE2.Tx(a1) * SE2.Rot(np.deg2rad(45)) * SE2.Tx(a2)
# # k.printline()

# # # number of joints 
# # print(e.n)

# # # type sctructure
# # print(e.structure)
# # e.plot(np.deg2rad([30, 45]))

# # Foward Kinematics

# # Figure
# env = PyPlot()
# env.launch()

# link1 = rtb.Link2(rtb.ET2.R(), name='link1')
# link2 = rtb.Link2(rtb.ET2.tx(a1)*rtb.ET2.R(), name='link2', parent=link1)
# link3 = rtb.Link2(rtb.ET2.tx(a2), name='link3', parent=link2)

# robot = rtb.ERobot2([link1, link2, link3], name='my robot')

# # Plot robot
# print(robot)

# Te = robot.fkine(robot)
# print(Te)


import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

# Define the robot parameters
L1 = 1  # length of the first link
L2 = 1  # length of the second link

# Define the robot's DH parameters
dh_parameters = [
    RevoluteDH(a=0, alpha=0, d=0, offset=0),  # Joint 1
    RevoluteDH(a=L1, alpha=0, d=0, offset=0),  # Joint 2
]

# Create a robot model
robot = DHRobot(dh_parameters, name='2R Robot')

# Display the robot
robot.plot([0, 0])  # Plot the robot at the home configuration

# Set the limits for joint movements
robot.qlim = [
    [0, 2 * np.pi],  # Joint 1 limits (0 to 2*pi radians)
    [0, 2 * np.pi],  # Joint 2 limits (0 to 2*pi radians)
]

# Test forward kinematics: compute end-effector position for a given joint configuration
q_test = [np.pi / 4, np.pi / 3]  # Example joint configuration
pose = robot.fkine(q_test)  # Forward kinematics
print('End-Effector Pose for Joint Configuration [pi/4, pi/3]:')
print(pose)

# Test inverse kinematics: find joint configurations for a given end-effector pose
# pose_test = SE3.TrotZ(np.pi / 3) * SE3.TrotX(np.pi / 4) * SE3.Transl([0.5, 0.5, 0.1])  # Example end-effector pose
# q_solutions = robot.ikine(pose_test)  # Inverse kinematics
# print('Joint Configurations for End-Effector Pose [0.5, 0.5, 0.1, pi/4, pi/3]:')
# print(q_solutions)
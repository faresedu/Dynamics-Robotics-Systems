import roboticstoolbox as rtb
from spatialmath import SE3, SE2
import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt
p = 5 # padding in degrees
getLimits = lambda x, p: np.deg2rad([-x+p, x-p])
limits = {
    'joint 1': np.deg2rad([0, 30]),
    'joint 2': np.deg2rad([180, 120]),
}

myRobot = rtb.DHRobot([
    rtb.RevoluteDH(d=0, a=50, alpha=0, offset=0, qlim=limits['joint 1']),
    rtb.RevoluteDH(d=0, a=60, alpha=0, offset=0, qlim=limits['joint 2'])], name='Mechanism')
print(myRobot)

# q = np.deg2rad([30, 60, 0])
q = np.deg2rad([30, 120])
fkine03 = myRobot.fkine(q)
print(myRobot.fkine(q))

l1 = rtb.DHRobot([rtb.RevoluteDH(d=0, a=50, alpha=0, offset=0, qlim=limits['joint 1'])], name='link1')
l2 = rtb.DHRobot([rtb.RevoluteDH(d=0, a=60, alpha=0, offset=0, qlim=limits['joint 2'])], name='link2')

fkine_01 = l1.fkine(q[0])
print(f'fkine 0D1:\n{fkine_01}')
fkine_12 = l2.fkine(q[1])
print(f'fkine 1D2:\n{fkine_12}')

qt = rtb.jtraj(np.deg2rad([0,180]), q, 50)
myRobot.plot(qt.q, backend='pyplot', movie='myrobot.gif')



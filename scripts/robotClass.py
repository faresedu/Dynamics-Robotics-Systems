import roboticstoolbox as rtb
import roboticstoolbox as rtb
from spatialmath import SE3, SE2
import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt

class myRobot():
    def __init__(self, name = '', joints = int, typeJoints = '', d=[], a=[], alpha=[], offset=0, qlim = None):
        self.name = name
        self.joints = joints
        self.transforms = {}
        self.fkineD = {}
        self.limits = {}
        self.typeJoints = typeJoints
        self.d = d
        self.a = a
        self.alpha = alpha
        self.offset = offset
        self.qlim = qlim

    def matrixDH(self):
        self.singleJoints = []
        for joint in range(self.joints):
                self.singleJoints.append(rtb.RevoluteDH(d=self.d[joint], a=self.a[joint], alpha=self.alpha[joint], offset=self.offset, qlim=self.qlim))
                self.transforms[f'T_{joint}{joint+1}'] = rtb.DHRobot([rtb.RevoluteDH(d=self.d[joint], a=self.a[joint], alpha=self.alpha[joint], offset=self.offset, qlim=self.qlim)], name=f'link {joint}')
                                                                                    
        self.transforms['DH'] = rtb.DHRobot(self.singleJoints, name=f'end effector')

    def fkine(self, q):
         for joint, keys in enumerate(self.transforms):
            if keys != 'DH':
                print(f'fkine {keys}:\n{self.transforms[keys].fkine(q[joint])}')
                self.fkineD[keys] = self.transforms[keys].fkine(q[joint])
            else: 
                print(f'fkine {keys}:\n{self.transforms[keys].fkine(q)}')
                self.fkineD[keys] = self.transforms[keys].fkine(q)
    
    def plot(self, q, transformation, movie):
        self.transforms[transformation].plot(q, backend='pyplot', movie=movie)
    
    def printTable(self):
         print(self.transforms['DH'])
    
    def jacobian(self, b0):
        jL = []; jA = []; lastRotation = np.eye(3)
        t = self.fkineD['DH'].t
        for joint, keys in enumerate(self.transforms):
            if keys != 'DH':
                if self.typeJoints[joint] == 'R':
                    if joint == 0: 
                        jA.append(b0)
                        lastPosition = np.array(t[:3] - np.zeros(3))
                        jL.append(np.cross(jA[joint], lastPosition))

                    elif joint>0 and keys != 'DH': 
                        lastRotation = lastRotation @ self.fkineD[keys].R
                        jA.append((lastRotation @ b0))
                        lastT = self.fkineD[keys].t
                        lastPosition = np.array(t[:3] - lastT[:3])
                        cross = np.cross(jA[joint], lastPosition)
                        jL.append(cross)
                    
                else:
                    if joint == 0: jL.append(b0)
                    
                    elif joint>0 and keys != 'DH': 
                        lastRotation = lastRotation @ self.fkineD[keys].R
                        jL.append(lastRotation @ b0)
                    
                    jA.append(np.zeros(3))
        
        

        return np.array(jL), np.array(jA)
         
    # def ikine(self, T, unit, verbose=True):
    #     assert unit in ['deg', 'rad'], "Unit must be 'deg' or 'rad'"
    #     q_out, success, _, _, _ = self.iiwa.ik_LM(SE3(T), tol=1e-10, ilimit=1000, q0=self.iiwa.q, joint_limits=True)

    #     # Print result
    #     if success: 
    #         if verbose:
    #             rospy.loginfo(f'Inverse Kinematics converged successfully! {np.around(np.rad2deg(q_out),1)}')
    #             rospy.loginfo('Error: {:.3} mm, {:.3} degrees'.format(*self.compute_error(T, q_out)))
    #         if unit == 'deg':
    #             return np.rad2deg(q_out), success
    #         else:
    #             return q_out, success
    #     else:
    #         if verbose:
    #             rospy.logerr(f'Inverse Kinematics failed to converge! {np.around(np.rad2deg(q_out),1)}')
    #             rospy.logerr('Error: {:.3} mm, {:.3} degrees'.format(*self.compute_error(T, q_out)))
    #         return q_out, success

         
         


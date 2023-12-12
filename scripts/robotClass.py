import roboticstoolbox as rtb
from spatialmath import SE3, SE2
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt

class myRobot():
    def __init__(self, name = '', joints = int, typeJoints = '', d=[], a=[], alpha=[], offset=0, qlim = None):
        self.name = name
        self.joints = joints
        self.transforms = {}
        self.fkineD = {}
        self.limits = {}
        self.jacobianD = {}
        self.typeJoints = typeJoints
        self.d = d
        self.a = a
        self.alpha = alpha
        self.offset = offset
        self.qlim = qlim
        self.q = None

    def matrixDH(self):
        self.singleJoints = []
        for joint in range(self.joints):
                if self.qlim == None:
                    self.singleJoints.append(rtb.RevoluteDH(d=self.d[joint], a=self.a[joint], alpha=self.alpha[joint], offset=self.offset, qlim=self.qlim))
                    self.transforms[f'T_{joint}{joint+1}'] = rtb.DHRobot([rtb.RevoluteDH(d=self.d[joint], a=self.a[joint], alpha=self.alpha[joint], offset=self.offset, qlim=self.qlim)], name=f'link {joint}')
                else: 
                    self.singleJoints.append(rtb.RevoluteDH(d=self.d[joint], a=self.a[joint], alpha=self.alpha[joint], offset=self.offset, qlim=self.qlim[joint]))
                    self.transforms[f'T_{joint}{joint+1}'] = rtb.DHRobot([rtb.RevoluteDH(d=self.d[joint], a=self.a[joint], alpha=self.alpha[joint], offset=self.offset, qlim=self.qlim[joint])], name=f'link {joint}')
                                                                                    
        self.transforms['DH'] = rtb.DHRobot(self.singleJoints, name=f'end effector')

    def fkine(self, q, verbose=True):
         self.q = q
         for joint, keys in enumerate(self.transforms):
            if keys != 'DH':
                # print(f'fkine {keys}:\n{self.transforms[keys].fkine(q[joint])}')
                if verbose: self.fkineD[keys] = np.around(self.transforms[keys].fkine(q[joint]), 5)
                else: self.fkineD[keys] = self.transforms[keys].fkine(q[joint])
            else: 
                # print(f'fkine {keys}:\n{self.transforms[keys].fkine(q)}')
                if verbose: self.fkineD[keys] = np.around(self.transforms[keys].fkine(q),5)
                else: self.fkineD[keys] = self.transforms[keys].fkine(q)
    
    def plotRobot(self, q, firstPose, movie):
        robot = self.transforms['DH']
        qt = rtb.jtraj(firstPose, q, 50)
        robot.plot(qt.q, backend='pyplot', movie=movie)
        
    
    def printTable(self):
         print(self.transforms['DH'])
    
    def jacobian(self, b0, q):
        self.fkine(q, verbose=False)
        jL = []; jA = []; lastRotation = np.eye(3)
        J = np.zeros((6, self.joints))
        t = self.fkineD['DH'].t
        for joint, keys in enumerate(self.transforms):
            if keys != 'DH' and joint < self.joints:
                if self.typeJoints[joint] == 'R':
                    if joint == 0: 
                        jA.append(b0)
                        lastPosition = np.array(t[:3] - np.zeros(3))
                        jL.append(np.around(np.cross(jA[joint], lastPosition), 5))
                       

                    elif joint>0: 
                        lastRotation = lastRotation @ self.fkineD[keys].R
                        jA.append(np.around((lastRotation @ b0),5))
                        lastT = self.fkineD[keys].t
                        lastPosition = np.array(t[:3] - lastT[:3])
                        cross = np.cross(jA[joint], lastPosition)
                        jL.append(np.around(cross, 5))
                    
                else:
                    if joint == 0: jL.append(b0)
                    
                    elif joint>0: 
                        lastRotation = lastRotation @ self.fkineD[keys].R
                        jL.append(np.around((lastRotation @ b0), 5))
                    
                    jA.append(np.zeros(3))
                
        
                J[:3, joint] = jL[joint]
                J[3:, joint] = jA[joint]
                self.jacobianD[f'J_{joint}'] = J

        return J
         
    def ikine(self, T, firstPose, q, unit, verbose=True):
        assert unit in ['deg', 'rad'], "Unit must be 'deg' or 'rad'"
        qt = rtb.jtraj(firstPose, q, 50)
        robot = self.transforms['DH']
        q_out, success, _, _, _ = robot.ik_LM(SE3(T), tol=1e-10, ilimit=1000, q0=qt.q, joint_limits=False)

        # Print result
        if success: 
            if verbose:
                print(f'Inverse Kinematics converged successfully! {np.around(np.rad2deg(q_out), 1)}')
                # print('Error: {:.3} mm, {:.3} degrees'.format(*self.compute_error(T, q_out)))
            
            if unit == 'deg':
                return np.rad2deg(q_out), success
            else:
                return q_out, success
        else:
            if verbose:
                print(f'Inverse Kinematics failed to converge! {np.around(np.rad2deg(q_out),1)}')
                # print('Error: {:.3} mm, {:.3} degrees'.format(*self.compute_error(T, q_out)))
            return q_out, success  

    def compute_error(self, T, q_out):
        """Return the error in millimeters and degrees"""
        robot = self.transforms['DH']
        T_out = robot.fkine(q_out)
        cartesian_error = np.linalg.norm(T[:3,3] - T_out.t)*1000

        # Compute magnitude orientation error
        R = Rot.from_matrix(T[:3,:3])
        R_out = Rot.from_matrix(T_out.R)

        # Calculate the difference between the two quaternions
        diff_quaternion = R * R_out.inv()
        orientation_error = np.abs(np.degrees(diff_quaternion.magnitude())) 

        return cartesian_error, orientation_error
    
    def lagrangeMethod(self):
        pass

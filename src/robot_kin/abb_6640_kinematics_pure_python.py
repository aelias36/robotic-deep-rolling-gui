
#######################
# abb_6640_kinematics
# Arthor: Chen-Lung Eric Lu, Rensselaer Polytechnic Institute
# email: luc5@rpi.edu
# This code provide forward and inverse kinematics function for ABB 6640 robot.
# The code uses general_robotics_toolbox from RPI to solve kinematics.
# 
# Forward Kinematics:
#   Input: joint angles (type: numpy array)
#   Output: Transform (type: general_robotics_toolbox.Transform)
#
# Inverse Kinematics:
#   Input: Rotation matrix (type: 3x3 numpy array), position (type: numpy array), last joint angle (type: numpy array)
#   Output: joint angles (type: numpy array)

from general_robotics_toolbox.general_robotics_toolbox import fwdkin
import numpy as np
import general_robotics_toolbox as rox
from random import random as rand
import time
import math

#from numba import jit

#import QuadProg as qp

ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])

class ABBKinematics(object):
    def __init__(self) -> None:
        super().__init__()

        self.robot = self.abb_irb_6640_255()

    # @jit(nopython=True)
    def abb_irb_6640_255(self):

        H = np.array((ez, ey, ey, ex, ey, ex)).T
        P = np.array([0*ez, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, 0*ex, 0*ey, 0.2*ex]).T
        # P = np.array([0.78*ez, 0.32*ex, 1.075*ez, 0.2*ez, 1.1425*ex, 0.2*ex, 0*ex]).T
        joint_type = np.array([0,0,0,0,0,0])
        joint_lower_limit = np.deg2rad(np.array([-170,-65,-180,-300,-120,-360*96]))
        joint_upper_limit = np.deg2rad(np.array([170,85,70,300,200,360*96]))
        joint_vel_limit = np.deg2rad(np.array([100,90,90,170,120,190]))

        p_tool = np.array([0,0,0])
        R_tool = rox.rot(ey, np.pi/2.0)
        # R_tool = rox.rot(ey, 0)

        return rox.Robot(H,P,joint_type,joint_lower_limit,joint_upper_limit,joint_vel_limit, R_tool=R_tool, p_tool=p_tool)

    def forkin(self,q):

        """
        Forward Kinematics of abb 6630 255
        :type   q: numpy array (length = 6 is required)
        :param  q: Input joint angles

        :rtype  T: general_robotics_toolbox.Transform
        :return T: Homogeneous transform matrix from the base frame to the tool frame
        """

        if len(q) != 6:
            raise ValueError("There must be exactly six joint angles.")

        q = np.array(q)
        
        return rox.fwdkin(self.robot,q)

    # @jit(nopython=True)
    def invkin(self, R, p, last_joints=None):
        
        """
        Inverse Kinematics of abb 6630 255
        :type   R: numpy array (3x3 matrix)
        :param  R: Rotation matrix from the base frame to the tool frame
        :type   p: numpy array (length=3 is required)
        :param  p: Position of the tool frame with respect to the base frame.
        :type   last_joints: numpy array (length = 6 is required)
        :param  last_joints: The joints of the robot at the last timestep. The returned 
                first returned joint configuration will be the closests to last_joints. Optional
        :rtype  q: list of numpy array
        :param  q: A list of zero or more joint angles that match the desired pose. An
                empty list means that the desired pose cannot be reached. If last_joints
                is specified, the first entry is the closest configuration to last_joints.
        """

        R = np.array(R)
        p = np.array(p)

        if (3,3) != np.shape(R):
            raise ValueError("Rotation matrix must have shape 3x3")
        elif len(p) != 3:
            raise ValueError("Position should be in 3 dimension.")
        elif last_joints is not None:
            if len(last_joints) != 6:
                raise ValueError("Last robot joint angles must have six joint values.")
        
        T = rox.Transform(R,p)

        q = rox.robot6_sphericalwrist_invkin(self.robot,T,last_joints=last_joints)

        return q

def test_func():
    
    q1 = np.deg2rad(rand()*340-170)
    q2 = np.deg2rad(rand()*150-65)
    q3 = np.deg2rad(rand()*250-180)
    q4 = np.deg2rad(rand()*600-300)
    q5 = np.deg2rad(rand()*240-120)
    q6 = np.deg2rad(rand()*720-360)

    q1,q2,q3,q4,q5,q6 = math.pi/2,math.pi/8,math.pi/10,math.pi/10,math.pi/8,math.pi/10

    abb = ABBKinematics()

    print("Initial q")
    print(np.array([q1, q2, q3, q4, q5, q6]))
    T = abb.forkin(np.array([q1, q2, q3, q4, q5, q6]))
    print("Robot T:")
    print(T)

    time_dur = np.array([])
    for i in range(1000):
        q1 = np.deg2rad(rand()*340-170)
        q2 = np.deg2rad(rand()*150-65)
        q3 = np.deg2rad(rand()*250-180)
        q4 = np.deg2rad(rand()*600-300)
        q5 = np.deg2rad(rand()*240-120)
        q6 = np.deg2rad(rand()*720-360)
        q = np.array([q1, q2, q3, q4, q5, q6])
        a = abb.forkin(q)
        st = time.perf_counter_ns()
        qsol = abb.invkin(a.R,a.p)
        et = time.perf_counter_ns()
        time_dur = np.append(time_dur, float(et-st)/1e6)
    print("Mean Time Elapsed:",np.mean(time_dur))
    print("Max Time Elapsed:",np.max(time_dur))
    print("Min Time Elapsed:",np.min(time_dur))

if __name__ == '__main__':
    test_func()

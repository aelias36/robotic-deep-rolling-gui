
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

from numba import jit

import QuadProg as qp

ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])

# @jit(nopython=True)
def abb_irb_6640_255():

    H = np.array((ez, ey, ey, ex, ey, ex)).T
    P = np.array([0*ez, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, 0*ex, 0*ey, 0.2*ex]).T
    # P = np.array([0.78*ez, 0.32*ex, 1.075*ez, 0.2*ez, 1.1425*ex, 0.2*ex, 0*ex]).T
    joint_type = np.array([0,0,0,0,0,0])
    joint_lower_limit = np.deg2rad(np.array([-170,-65,-180,-300,-120,-360*96]))
    joint_upper_limit = np.deg2rad(np.array([170,85,70,300,200,360*96]))
    joint_vel_limit = np.deg2rad(np.array([100,90,90,170,120,190]))

    p_tool = np.array([0,0,0])
    # R_tool = rox.rot(ey, np.pi/2.0)
    R_tool = rox.rot(ey, 0)

    return rox.Robot(H,P,joint_type,joint_lower_limit,joint_upper_limit,joint_vel_limit, R_tool=R_tool, p_tool=p_tool)

def forkin(q):

    """
    Forward Kinematics of abb 6630 255
    :type   q: numpy array (length = 6 is required)
    :param  q: Input joint angles

    :rtype  T: general_robotics_toolbox.Transform
    :return T: Homogeneous transform matrix from the base frame to the tool frame
    """

    if len(q) != 6:
        raise ValueError("There must be exactly six joint angles.")

    abb_robot = abb_irb_6640_255()
    q = np.array(q)
    
    return rox.fwdkin(abb_robot,q)

# @jit(nopython=True)
def invkin(R, p, last_joints=None):
    
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

    st = time.perf_counter_ns()
    abb_robot = abb_irb_6640_255()
    et = time.perf_counter_ns()
    print("Set up robot Time Elapse:",float(et-st)/1e6)
    
    T = rox.Transform(R,p)

    st = time.perf_counter_ns()
    q = rox.robot6_sphericalwrist_invkin(abb_robot,T,last_joints=last_joints)
    et = time.perf_counter_ns()
    print("IK Time Elapse:",float(et-st)/1e6)

    spatial_velocity_command = np.array([0, 0, 0, 10, 10, 1])
    quadprog = qp.QuadProg(abb_robot)
    st = time.perf_counter_ns()
    joints_vel = quadprog.compute_joint_vel_cmd_qp(np.deg2rad(q[0]), spatial_velocity_command)
    et = time.perf_counter_ns()
    print("QP Time Elapse:",float(et-st)/1e6)

    return q

def test_func():
    
    q1 = np.deg2rad(rand()*340-170)
    q2 = np.deg2rad(rand()*150-65)
    q3 = np.deg2rad(rand()*250-180)
    q4 = np.deg2rad(rand()*600-300)
    q5 = np.deg2rad(rand()*240-120)
    q6 = np.deg2rad(rand()*720-360)

    q1,q2,q3,q4,q5,q6 = math.pi/2,math.pi/8,math.pi/10,math.pi/10,math.pi/8,math.pi/10

    print("Initial q")
    print(np.array([q1, q2, q3, q4, q5, q6]))
    T = forkin(np.array([q1, q2, q3, q4, q5, q6]))
    print("Robot T:")
    print(T)

    # st = time.perf_counter_ns()
    # q = invkin(T.R,T.p)
    # et = time.perf_counter_ns()
    # print("Total Time Elapse:",float(et-st)/1e6)
    # print("Inv q:")
    # print(q)

if __name__ == '__main__':
    test_func()
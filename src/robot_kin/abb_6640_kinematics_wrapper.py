from ctypes import *
import numpy as np
import math
import time
from random import random as rand
import general_robotics_toolbox as rox

lib = CDLL('./librobkin.dll')

# create a Geek class
class SphKin(object):
  
    # constructor
    def __init__(self):
  
        # attribute
        lib.SphericalKin_new.restype = c_void_p
        lib.fwdkin.argtypes=[c_void_p, POINTER(c_double)]
        lib.fwdkin.restype=POINTER(c_double)
        lib.invkin.argtypes=[c_void_p, POINTER(c_double)]
        lib.invkin.restype=POINTER(c_double)
        lib.get_solnum.argtypes=[c_void_p]
        lib.get_solnum.restype=c_int

        self.obj = lib.SphericalKin_new()

        # joint limit
        self.joint_lower_limit = np.deg2rad(np.array([-170,-65,-180,-300,-120,-360*96]))
        self.joint_upper_limit = np.deg2rad(np.array([170,85,70,300,200,360*96]))
  
    # define method
    def forkin(self, q):
        qnp = np.array(q)
        qc = qnp.ctypes.data_as(POINTER(c_double))
        flat_T = lib.fwdkin(self.obj,qc)
        T = np.ctypeslib.as_array(flat_T, shape=(16,))
        T = np.reshape(T,(4,4)).T
        return rox.Transform(T[0:3,0:3],T[0:3,3])
    def invkin(self, R, p, last_joints=None):
        TRP = np.vstack((R,np.array([0,0,0]))).T
        flat_T = np.reshape(TRP,(12,))
        flat_T = np.append(flat_T, p)
        flat_T = np.append(flat_T, 1)
        flat_Tc = flat_T.ctypes.data_as(POINTER(c_double))
        qsolc = lib.invkin(self.obj,flat_Tc)
        qsol_n = lib.get_solnum(self.obj)
        qsol = np.ctypeslib.as_array(qsolc, shape=(qsol_n*6,))
        qsol = np.reshape(qsol,(qsol_n,6))

        # check joint limit
        qsol_t = np.array([])
        for sol in qsol:
            upper = np.prod(sol<self.joint_upper_limit)
            lower = np.prod(sol>self.joint_lower_limit)
            if upper*lower >0:
                qsol_t = np.append(qsol_t, sol)
        qsol_t = np.reshape(qsol_t,(int(qsol_t.size/6),6))

        if last_joints is not None:
            theta_dist = np.linalg.norm(np.subtract(qsol_t,last_joints), axis=1)
            return [qsol_t[i] for i in list(np.argsort(theta_dist))]
        else:
            return qsol_t

def test_func():

    abb = SphKin()
    q1,q2,q3,q4,q5,q6 = math.pi/2,-math.pi/2+0.2,math.pi/10,math.pi/10,math.pi/8,math.pi/10

    print("Initial q")
    print(np.array([q1, q2, q3, q4, q5, q6]))
    T = abb.forkin(np.array([q1, q2, q3, q4, q5, q6]))
    print("Robot T:")
    print(T)

    time_dur_fk = np.array([])
    time_dur_ik = np.array([])
    for i in range(1000):
        q1 = np.deg2rad(rand()*340-170)
        q2 = np.deg2rad(rand()*150-65)
        q3 = np.deg2rad(rand()*250-180)
        q4 = np.deg2rad(rand()*600-300)
        q5 = np.deg2rad(rand()*240-120)
        q6 = np.deg2rad(rand()*720-360)
        q = np.array([q1, q2, q3, q4, q5, q6])
        st = time.perf_counter_ns()
        a = abb.forkin(q)
        et = time.perf_counter_ns()
        time_dur_fk = np.append(time_dur_fk, float(et-st)/1e6)
        st = time.perf_counter_ns()
        qsol = abb.invkin(a.R,a.p)
        et = time.perf_counter_ns()
        time_dur_ik = np.append(time_dur_ik, float(et-st)/1e6)
    print("FK Mean Time Elapsed:",np.mean(time_dur_fk))
    print("FK Max Time Elapsed:",np.max(time_dur_fk))
    print("FK Min Time Elapsed:",np.min(time_dur_fk))
    print("IK Mean Time Elapsed:",np.mean(time_dur_ik))
    print("IK Max Time Elapsed:",np.max(time_dur_ik))
    print("IK Min Time Elapsed:",np.min(time_dur_ik))

if __name__ == '__main__':
    test_func()

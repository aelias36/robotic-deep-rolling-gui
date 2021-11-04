
#######################
# abb_6640_kinematics
# Arthor: Chen-Lung Eric Lu, Rensselaer Pos described in the documentation on type conversion, so we can use a simple list comprehension here to copy the C int values into a Python list of Python int objects, which Cython creates automatically along the way. You could also have iterated manlytechnic Institute
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

from libc.math cimport atan2, sin, cos, pow, sqrt
from libc.stdlib cimport malloc
import numpy as np
cimport numpy as np
from cpython cimport array
import array

ctypedef np.double_t DTYPE_t
norm = np.linalg.norm

cdef double dot(const double* a, double* b):
    cdef double sol = 0
    for i in range(3):
        sol = sol+a[i]*b[i]
    return sol
# cdef double norm(double[:] a):
#     cdef double sol = 0
#     for i in a:
#         sol += pow(i,2)
#     return sqrt(sol)
# cdef double[:] multscalar(double[:] a, double b):
#     cdef double[3] c
#     for i in range(3):
#         c[i] = a[i]*b
#     return c
# cdef double[:] addition(double[:] a, double[:] b):
#     cdef double[3] c
#     for i in range(3):
#         c[i] = a[i]+b[i]
#     return c
# cdef double[:] subtration(double[:] a, double[:] b):
#     cdef double[3] c
#     for i in range(3):
#         print('a',a[i])
#         print('b',b[i])
#         c[i] = a[i]-b[i]
#         print(i, c[i])
#     return c

cdef np.ndarray[DTYPE_t, ndim=1] cross(np.ndarray[DTYPE_t, ndim=1] a, np.ndarray[DTYPE_t, ndim=1] b):
    cdef np.ndarray[DTYPE_t, ndim=1] c=np.zeros(3, dtype=np.double)
    c[0]=a[1]*b[2]-a[2]*b[1]
    c[1]=a[2]*b[0]-a[0]*b[2]
    c[2]=a[0]*b[1]-a[2]*b[1]
    return c

cpdef double subprob0(np.ndarray[DTYPE_t, ndim=1] k, np.ndarray[DTYPE_t, ndim=1] p1, np.ndarray[DTYPE_t, ndim=1] p2):

    cdef double q=0

    if np.dot(k,p1)!=0 or np.dot(k,p2)!=0:
        raise ValueError("k must be perpendicular to p and q")
    
    p1=p1/norm(p1)
    p2=p2/norm(p2)

    q=2*atan2(norm(p1-p2),norm(p1+p2))

    if np.dot(k,cross(p1,p2))<0:
        q = -1*q

    return q

cpdef double subprob1(np.ndarray[DTYPE_t, ndim=1] k, np.ndarray[DTYPE_t, ndim=1] p1, np.ndarray[DTYPE_t, ndim=1] p2):

    p1=p1/norm(p1)
    p2=p2/norm(p2)
    k=k/norm(k)
    p1=p1-np.dot(k,p1)*k
    p2=p2-np.dot(k,p2)*k

    return subprob0(k,p1,p2)

# cpdef double[:,:] subprob2(p1,p2,k1,k2):

#     cdef double[2][2] q
#     cdef double k12=dot(k1,k2)

#     p1=multscalar(p1,1./norm(p1))
#     p2=multscalar(p2,1./norm(p2))
#     k1=multscalar(k1,1./norm(k1))
#     k2=multscalar(k2,1./norm(k2))

#     cdef double k1p1=dot(k1,p1)
#     cdef double k2p2=dot(k2,p2)
#     cdef double a=(k1p1-k12*k2p2)/(1-pow(k12,2))
#     cdef double b=(k2p2-k12*k1p1)/(1-pow(k12,2))
#     cdef double[:] k12c=cross(k1,k2)

#     cdef c2=(dot(p1,p1)-pow(a,2)-pow(b,2)-2*a*b*k12)/dot(k12c,k12c)
#     cdef bint v2nan=False

#     if c2<0:
#         q[0][0]=-999;q[1][0]=-999;q[0][1]=-999;q[1][1]=-999
#         return q
    
#     cdef double[:] ak1=multscalar(k1,a)
#     cdef double[:] bk2=multscalar(k2,b)
#     cdef double[:] v1,v2
#     if c2==0:
#         v1=addition(ak1,bk2)
#         v2nan=True
#     else:
#         v1=addition(addition(ak1,bk2),multscalar(k12c,sqrt(c2)))
#         v2=subtration(addition(ak1,bk2),multscalar(k12c,sqrt(c2)))
    
#     if v2nan:
#         q[0][0]=subprob1(p1,v1,k1); q[0][1]=-999
#         q[1][0]=subprob1(p2,v1,k2); q[1][1]=-999
#     else:
#         q[0][0]=subprob1(p1,v1,k1)
#         q[1][0]=subprob1(p2,v1,k2)
#         q[0][1]=subprob1(p1,v2,k1)
#         q[1][1]=subprob1(p2,v2,k2)
#     return q

# cpdef double ddot(double[:] a, double[:] b):
#     cdef double sol=0
#     return norm(a)

# def abb_irb_6640_255():

#     H = np.array([ez, ey, ey, ex, ey, ex]).T
#     P = np.array([0*ez, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, 0*ex, 0*ey, 0.2*ex]).T
#     # P = np.array([0.78*ez, 0.32*ex, 1.075*ez, 0.2*ez, 1.1425*ex, 0.2*ex, 0*ex]).T
#     joint_type = np.array([0,0,0,0,0,0])
#     joint_lower_limit = np.deg2rad(np.array([-170,-65,-180,-300,-120,-360*96]))
#     joint_upper_limit = np.deg2rad(np.array([170,85,70,300,200,360*96]))
#     joint_vel_limit = np.deg2rad(np.array([100,90,90,170,120,190]))

#     p_tool = np.array([0,0,0])
#     R_tool = rox.rot(ey, np.pi/2.0)

#     return rox.Robot(H,P,joint_type,joint_lower_limit,joint_upper_limit,joint_vel_limit, R_tool=R_tool, p_tool=p_tool)

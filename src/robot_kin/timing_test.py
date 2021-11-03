import sys
sys.path.append('../')


from robot_kin import abb_6640_kinematics as kin
import time
import numpy as np
from random import random as rand
import general_robotics_toolbox as rox

N = 1000

t_0 = time.time()

for i in range(N):
    q1 = np.deg2rad(rand()*340-170)
    q2 = np.deg2rad(rand()*150-65)
    q3 = np.deg2rad(rand()*250-180)
    q4 = np.deg2rad(rand()*600-300)
    q5 = np.deg2rad(rand()*240-120)
    q6 = np.deg2rad(rand()*720-360)
    q = np.array([q1, q2, q3, q4, q5, q6])
    T = kin.forkin(q)


t_1 = time.time()

R = rox.rot([1,0,0], np.pi)

for i in range(N):
  p = np.random.rand(3)*0.1 + np.array([1.0, 1.0, 1.0])
  q = kin.invkin(R,p)

t_2 = time.time()

print("Forward Kinematics:")
print((t_1 - t_0)/N/0.004 * 100.0, "%")

print("Inverse Kinematics:")
print((t_2 - t_1)/N/0.004 * 100.0, "%")

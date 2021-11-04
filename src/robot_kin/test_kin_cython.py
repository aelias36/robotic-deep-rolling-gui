import abb_6640_kinematics as ab
import numpy as np
import time

k1 = np.array([0,0,1], dtype=np.double)
k2 = np.array([1,0,0], dtype=np.double)
p1 = np.array([0,1,2], dtype=np.double)
p2 = np.array([1,0,2], dtype=np.double)

st = time.perf_counter_ns()
ab.subprob1(k1, p1, p2)
et = time.perf_counter_ns()
print("Sub1 Time Elapse:",float(et-st)/1e6)

# st = time.perf_counter_ns()
# ab.subprob2(p1, p2, k1, k2)
# et = time.perf_counter_ns()
# print("Sub2 Time Elapse:",float(et-st)/1e6)
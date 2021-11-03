
#######################
# ft_simulation
# Arthor: Chen-Lung Eric Lu, Rensselaer Polytechnic Institute
# email: luc5@rpi.edu
# This code provide simulated force torque.
# The code use simple spring system to simulate force.
# More accurate physics simulation can be extended in the future.
#
# class FTsim:
#   function init(k):
#       Input: spring constant (1x3 numpy array)
#   function read_ft_streaming(dp):
#       Input: position displacement in x y z (1x3 numpy array)

import numpy as np
from random import random as rand

ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])

class FTSim(object):
    def __init__(self, k) -> None:
        super().__init__()

        self.k = np.array(k)

    def read_ft_streaming(self, dp):

        """
        Read simulated force torque
        :type   dp: position displacement (length = 3 is required)
        :param  q: Input joint angles

        :rtype  T: general_robotics_toolbox.Transform
        :return T: Homogeneous transform matrix from the base frame to the tool frame
        """

        if len(dp) != 3:
            raise ValueError("Please gives 3 displacement for x y z direction.")
        
        dp = np.array(dp)
        force = np.multiply(np.less(dp,0),dp)*self.k
        ft = np.append([0,0,0],force)

        return ft

def test_2():
    # Some extra test for rolling on a plane
    
    # Only apply force in the Z direction
    k = 1e5
    ft_sensor = FTSim([0,0,k])
    
    # Should be some force
    print(ft_sensor.read_ft_streaming([0,0,-0.01]))
    # Should be double force
    print(ft_sensor.read_ft_streaming([0,0,-0.02]))
    # Should be no force
    print(ft_sensor.read_ft_streaming([0,0,+0.01]))
    print(ft_sensor.read_ft_streaming([-1,-2,+0.01]))


if __name__ == '__main__':

    # Example for ft simulation
    k = 1e5 # spring constant
    ft_sensor = FTSim([k,k,k]) # 1x3 array: k constant in x, y, z, direction

    # 1x3 array: displaysment in x, y, z direction (p_robot-p_d)
    # only pressing (dp < 0) create force
    ft = ft_sensor.read_ft_streaming([0,0,-0.01]) 

    print("============")
    print("Torque x", ft[0])
    print("Torque y", ft[1])
    print("Torque z", ft[2])
    print("Force x", ft[3])
    print("Force y", ft[4])
    print("Force z", ft[5])
    print("============")

    test_2()
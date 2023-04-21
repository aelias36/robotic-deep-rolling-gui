# Toolpath overview
TODO

# Toolpath parameters (TODO)
## Geometry
* part_length: Bounding box distance in X (rolling) direction
* part_width: Bounding box distance in Y (stepover) direction
* margin_width: Width of unrolled area on each side (X  / rolling direction)
* margin_length: Length of unrolled area on each side (Y / stepover direction)
* stepover: Width between centers of adjacent rolls (in Y direction)
* hover_height: Height of tool during stepover

## Force Profile
* f_load_unload_rate: N/s Rate w.r.t. to ramp force during loading and unloading
* f_max: Maximum force, used during constant force section
* f_min_prop: Proportion of F_max applied at very beginning and end of roll (0.1)
* f_ramp_dist: Distance to ramp from F_min to F_max, and F_max to F_min (12.7e-3 # m # 0.5 in). Note that f_ramp_dist can be at most half of x_max or part_length.

## Velocity Profile:
* accel: Acceleration used to ramp between different velocities
* v_slow: Slower velocity used at beginning and end of roll
* v_fast: Faster velocity used in middle of roll
* v_slow_dist: Distance at beginning and end of roll to travel at <= v_slow
* v_stepover: Velocity used during a stepover

## Helpful diagram for calculating certain parameters
![image](https://user-images.githubusercontent.com/123105763/233668145-265b75f3-98f3-4194-baa3-3609b12d7a52.png)
When you would like to change parameters such as the length of a roll, you need to be sure to adjust other parameters according to the following criteria, as they relate to the above diagram:
* f_ramp_dist can be at most half of x_max
* Area under the first three segments of the velocity profile (first acceleration, v_slow section, and second acceleration) can be at most half of x_max
* Area under the first two segments of the velocity profile (first acceleration, v_slow section) must be greater than f_ramp_dist

# Generating a toolpath from Python
To do this, you must have downloaded Python and installed the necessary dependencies as detailed in the installation tutorial located here: https://github.com/aelias36/robotic-deep-rolling-gui/blob/main/installation_tutorial.md.

1. Use the python file force_toolpath_gen.py located in src > toolpath_gen to generate a toolpath. 

2. The only part of this Python file that you need to edit is at the very bottom of it; the main() function. This is where parameters are specified. It is also where a .yaml file is read, which is where all of the parameters come from.

3. At the time that this tutorial is being written, all of the "params" statements in the main() function are commented out. This is because all of the parameters needed to generate a toolpath are being read from the .yaml file. So all that you have to change from toolpath to toolpath is the name of the .yaml file that the python code is reading from.
![image](https://user-images.githubusercontent.com/123105763/227229182-dfc2c576-5571-4eb3-a23c-18194f415900.png)

4. Once you edit the name of the .yaml file, run the python code, and a toolpath should pop out into your working directory. You can specify the name of the toolpath on line 376 of the python code.
![image](https://user-images.githubusercontent.com/123105763/227229282-82f4e46a-058a-46ea-b0c1-69019305485d.png)

# Generating a toolpath from an Excel file
TODO

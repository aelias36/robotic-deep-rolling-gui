# Toolpath overview
TODO

# Toolpath parameters (TODO)
## Geometry
* part_length
* part_width
* margin_width
* margin_length: Length of unrolled area on each side (Y / stepover direction)
* stepover: Width between centers of adjacent rolls (in Y direction)
* hover_height:

## Force Profile
* f_load_unload_rate: N/s Rate w.r.t. to ramp force during loading and unloading
* f_max: Maximum force, used during constant force section
* f_min_prop: 0.1
* f_ramp_dist: 12.7e-3 # m # 0.5 in

## Velocity Profile:
* accel
* v_slow
* v_fast
* v_fast
* v_slow_dist
* v_stepover

# Generating a toolpath from Python
TODO

# Generating a toolpath from an Excel file
TODO

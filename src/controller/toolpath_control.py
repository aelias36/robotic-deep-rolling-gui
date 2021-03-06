'''
toolpath_control.py
Alex Elias

Calculates the desired tool pose using a toolpath text file,
along with force feedback and the previous desired tool pose

PARAMETERS

force_ctrl_damping: damping for force control
    Higher damping means less velocity for the same force
force_epsilon: smallest value of force to be considered touching
    Make high enough to avoid problems with sensor noise
moveL_speed_lin: linear tool speed during moveL commands
moveL_speed_ang: angular tool speed during moveL commands
load_speed: tool speed during force loading
unload_speed: tool speed duruing force unloading

disable_f_ctrl: disable force control (True/False)
    Force control commands do not move the tool in the force control direction

N_lookahead: number of samples for force lookahead
    Force commands will be sent early by this amount
'''


from collections import namedtuple
import general_robotics_toolbox as rox

import numpy as np

TIMESTEP = 0.004
CONV_TIME = 0.5 / TIMESTEP

class ToolpathControl():
    def __init__(self):
        self.commands = []
        self.program_counter = 0
        self.params = None

        self.file_name = None
        self.file_comment_lines = None

        self.tare_func = None
        self.conv_counter = 0

        self.CmdMoveL = namedtuple("CmdMoveL", "x y z q0 qx qy qz")
        self.CmdLoadZ = namedtuple("CmdLoadZ", "")
        # f_la is force lookahead - calculated based on lookahead time
        self.CmdForceCtrlZ = namedtuple("CmdForceCtrlZ", "x y fz q0 qx qy qz f_la")
        self.CmdPosCtrl = namedtuple("CmdPosCtrl", "x y z q0 qx qy qz")
        self.CmdUnloadZ = namedtuple("CmdUnloadZ", "z")
        self.CmdTare = namedtuple("CmdTare", "")

    def load_toolpath(self, toolpath_lines):
        self.commands = []
        self.file_comment_lines = []
        self.program_counter = 0

        for line in toolpath_lines:
            line_sep = line.split()
            if line[0] == "#": # Comment
                self.file_comment_lines.append(line)
            elif line_sep[0] == "moveL":
                self.commands.append(
                    self.CmdMoveL(*[float(x) for x in line_sep[1:]]))

            elif line_sep[0] == "loadZ":
                self.commands.append(self.CmdLoadZ())

            elif line_sep[0] == "forceCtrlZ":
                self.commands.append(
                    self.CmdForceCtrlZ(*([float(x) for x in line_sep[1:]] + [None])))

            elif line_sep[0] == "posCtrl":
                self.commands.append(
                    self.CmdPosCtrl(*[float(x) for x in line_sep[1:]]))

            elif line_sep[0] == "unloadZ":
                self.commands.append(
                    self.CmdUnloadZ(*[float(x) for x in line_sep[1:]]))

            elif line_sep[0] == "tare":
                self.commands.append(self.CmdTare())
            else:
                raise ValueError("Wrong toolpath command: " + line)

        self.calculate_lookahead()

    def step(self, tool_pose, force, disable_f_ctrl = False):
        # TODO check if a program is loaded

        current_cmd = self.commands[self.program_counter]

        # return True -> move to next cmd
        # return False -> stay on command
        if type(current_cmd) is self.CmdMoveL:
            inc, T = self.moveL(current_cmd, tool_pose, force)

        elif type(current_cmd) is self.CmdLoadZ:
            inc, T = self.loadZ(tool_pose, force, disable_f_ctrl)

        elif type(current_cmd) is self.CmdForceCtrlZ:
            inc, T = self.forceCtrlZ(current_cmd, tool_pose, force, disable_f_ctrl)

        elif type(current_cmd) is self.CmdPosCtrl:
            inc, T = self.posCtrl(current_cmd)

        elif type(current_cmd) is self.CmdUnloadZ:
            inc, T = self.unloadZ(current_cmd, tool_pose, force)

        elif type(current_cmd) is self.CmdTare:
            inc, T = self.tare(tool_pose)
        
        else:
            raise ValueError('Wrong command type: ' + str(current_cmd))

        # TODO check if we're at the end of the program
        if inc:
            if self.program_counter == len(self.commands) - 1:
                return True, T
            self.program_counter += 1

        return False, T

    def moveL(self, cmd, tool_pose, force):
        # Calculate linear and angular errors
        p_des = np.array([cmd.x, cmd.y, cmd.z])
        R_des = rox.q2R([cmd.q0, cmd.qx, cmd.qy, cmd.qz])

        pos_err = tool_pose.p - p_des
        R_err = tool_pose.R.T.dot(R_des)
        R_err_k, R_err_theta = rox.R2rot(R_err)

        # Small error -> snap to correct position
        # Big error -> reduce error with constant vel
        pos_err_size = np.linalg.norm(pos_err)
        if pos_err_size > self.params['moveL_speed_lin'] * TIMESTEP:
            pos_err_dir = pos_err / pos_err_size
            p = tool_pose.p - self.params['moveL_speed_lin'] * TIMESTEP * pos_err_dir
            pos_is_done = False
        else:
            p = p_des
            pos_is_done = True

        if R_err_theta > self.params['moveL_speed_ang'] * TIMESTEP:
            R = rox.rot(R_err_k, self.params['moveL_speed_ang'] * TIMESTEP).dot(tool_pose.R)
            rot_is_done = False
        else:
            R = R_des
            rot_is_done = True

        return pos_is_done and rot_is_done, rox.Transform(R, p)

    def loadZ(self, tool_pose, force, disable_f_ctrl):
        if disable_f_ctrl:
            return True, tool_pose

        f_z_meas = force[5]
        if f_z_meas < self.params['force_epsilon']:
            p = tool_pose.p
            p[2] = tool_pose.p[2] - self.params['load_speed'] * TIMESTEP
            return False, rox.Transform(tool_pose.R, p)
        else:
            return True, tool_pose

    def forceCtrlZ(self, cmd, tool_pose, force, disable_f_ctrl):
        # TODO lookahead
        f_z_meas = force[5]
        R = rox.q2R([cmd.q0, cmd.qx, cmd.qy, cmd.qz])

        if disable_f_ctrl:
            p = [cmd.x, cmd.y, tool_pose.p[2]]
        
        else: # force control mode
            if cmd.f_la is not None:
                f_desired = max(cmd.f_la, self.params['force_epsilon'])
            else:
                f_desired = max(cmd.fz,   self.params['force_epsilon'])
            v_z = ( f_z_meas - f_desired ) / self.params['force_ctrl_damping']
            z = tool_pose.p[2] + v_z * TIMESTEP

            p = [cmd.x, cmd.y, z]

        return True, rox.Transform(R, p)


    def posCtrl(self, cmd):
        R = rox.q2R([cmd.q0, cmd.qx, cmd.qy, cmd.qz])
        p = [cmd.x, cmd.y, cmd.z]
        
        return True, rox.Transform(R,p)

    def unloadZ(self, cmd, tool_pose, force):
        f_z_meas = force[5]
        p = tool_pose.p
        if tool_pose.p[2] + self.params['unload_speed'] * TIMESTEP >= cmd.z:
            p[2] = cmd.z
            return True, rox.Transform(tool_pose.R, p)
        else:
            p[2] += self.params['unload_speed'] * TIMESTEP
            return False, rox.Transform(tool_pose.R, p)

    def tare(self, tool_pose):
        # Make sure we aren't moving by waiting CONV_TIME steps
        if self.conv_counter < CONV_TIME:
            self.conv_counter += 1
            return False, tool_pose
        else:
            self.conv_counter = 0
            if self.tare_func is not None:
                self.tare_func()
            else:
                print("Warning: tare_func not set")
            return True, tool_pose

    def calculate_lookahead(self):
        if (not self.commands) or (not self.params):
            return

        if self.params['N_lookahead'] == 0:
            return


        for i, cmd in enumerate(self.commands):
            if type(cmd) is not self.CmdForceCtrlZ:
                continue

            copy_from_ind = i + self.params['N_lookahead']

            if copy_from_ind > len(self.commands) - 1:
                f_la = 0.0
            elif any(type(c) is not self.CmdForceCtrlZ for c in self.commands[i:copy_from_ind+1]):
                f_la = 0.0
            else:
                f_la = self.commands[copy_from_ind].fz

            self.commands[i] = cmd._replace(f_la = f_la) # Maybe shouldn't be a namedtuple...




def timing_test():
    import time

    file = "../toolpath_gen/test_force_toolpath.txt"
    with open(file) as f:
        lines = f.readlines()

    tp_ctrl = ToolpathControl()
    tp_ctrl.load_toolpath(lines)

    tp_ctrl.params = {
        "force_ctrl_damping": 100.0,
        "force_epsilon": 10.0,
        "moveL_speed_lin": 1.0,
        "moveL_speed_ang": 1.0,
        "load_speed": 1.0,
        "unload_speed": 1.0
        }

    for i in range(20):
        print(tp_ctrl.commands[i])

    R = rox.rot([0,0,1], np.deg2rad(90))
    p = [0,0,0]
    tool_pose = rox.Transform(R,p)
    force = np.array([0,0,0,0,0,0])

    is_done = False
    t_0 = time.perf_counter()
    i = 0
    while not is_done:
        is_done, tool_pose = tp_ctrl.step(tool_pose, force)
        i +=1
    t_1 = time.perf_counter()

    print(is_done, tool_pose)
    print((t_1 - t_0)/i / TIMESTEP * 100, '%')

def main():
    import sys
    sys.path.append('../')
    from robot_kin import abb_6640_kinematics as kin
    from rpi_abb_irc5 import rpi_abb_irc5
    from ft_simulation import ft_simulation
    egm = rpi_abb_irc5.EGM()



    input("Careful - robot motion in this script")
    file = "../toolpath_gen/test_force_toolpath.txt"
    with open(file) as f:
        lines = f.readlines()

    tp_ctrl = ToolpathControl()
    tp_ctrl.load_toolpath(lines)

    tp_ctrl.params = {
        "force_ctrl_damping": 1.0e4,
        "force_epsilon": 10.0,
        "moveL_speed_lin": 0.1,
        "moveL_speed_ang": np.deg2rad(10.0),
        "load_speed": 0.001,
        "unload_speed": 0.01
        }

    for i in range(20):
        print(tp_ctrl.commands[i])

    robot_pose = None
    k = 1.0e5
    force_sim = ft_simulation.FTSim([0,0,-k])

    #force = force_sim.read_ft_streaming([0.,0.,-0.3])
    #print("test_force:", force)
    #input('...')

    wp_os = rox.Transform(rox.rot([0.,0.,1.], np.pi/2),  [1.5, 0.0, 1.7])
    tool_os = rox.Transform(rox.rot([0.,1.,0.], np.pi),  [0.0, 0.0, 0.0])

    while True:
        res, state = egm.receive_from_robot(.1)

        if res:
            # Clear queue
            i = 0
            while True:
                res_i, state_i = egm.receive_from_robot()
                if res_i: # there was another msg waiting
                    state = state_i
                    i += 1
                else: # previous msg was end of queue
                    break

            fb = state.robot_message.feedBack.cartesian
            pos = np.array([fb.pos.x, fb.pos.y, fb.pos.z])/1000.0 # mm -> m
            quat = [fb.orient.u0, fb.orient.u1, fb.orient.u2, fb.orient.u3]
            T_meas = rox.Transform(rox.q2R(quat), pos)
            if robot_pose is None:
                robot_pose = T_meas

            tool_pose =  robot_pose * tool_os
            tool_wp_pose = wp_os.inv()*tool_pose

            force = force_sim.read_ft_streaming(tool_wp_pose.p)
            #print("force:", force)
            is_done, tool_wp_pose = tp_ctrl.step(tool_wp_pose, force)
            
            tool_pose = wp_os*tool_wp_pose
            robot_pose = tool_pose*tool_os.inv()

            print(force[5], tp_ctrl.commands[tp_ctrl.program_counter])
            #print("robot_wp_pose:", robot_wp_pose)
            # print("robot_pose:", robot_pose)
            # print("tool_pose:", tool_pose)
            #print("tool_wp_pose:", tool_wp_pose)


            pos = robot_pose.p * 1000.0 # m -> mm
            quat = rox.R2q(robot_pose.R)
            egm.send_to_robot_cart(pos, quat)

            if is_done:
                break
        else:
            print("Waiting...")

    print("Done!")


if __name__ == '__main__':
    main()
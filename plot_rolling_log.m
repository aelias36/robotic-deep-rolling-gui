% plot_rolling_log.m
% Alex Elias
% Plots out the data from deep rolling log files created from GUI
%
% Run each section by pressing ctrl + enter
% (or press the run section button)


%% Read in log data
[file,path] = uigetfile('*.csv')
tab = readtable(strcat(path, file), 'CommentStyle',"#");

%% X Position vs Timestep
plot(diff(tab.EGM_X)); hold on
plot(diff(tab.Commanded_X)); hold off
ylim([-1e-4, 10e-4])
xlabel("Timestep")
ylabel("X Position (m)")
legend(["Feedback", "Commanded"])
title("X Position vs Timestep")

%% Delta t vs Timestep
plot(diff(tab.timestamp), 'x'); yline(0.004);
xlabel("Timestep")
ylabel("\Delta t (sec)")
title("\Delta t vs Timestep")

%% Joint Angle vs Time
plot(tab.timestamp, [tab.EGMq1, tab.EGMq2, tab.EGMq3, tab.EGMq4, tab.EGMq5, tab.EGMq6])
legend(["q_1", "q_2","q_3","q_4","q_5", "q_6"])
ylabel("Joint Angle (rad)")
xlabel("Time (sec)")
title("Joint Angle vs Time")

%% Position vs Timestep
plot([tab.EGM_X, tab.EGM_Y, tab.EGM_Z]); hold on;
plot( [tab.Commanded_X, tab.Commanded_Y, tab.Commanded_Z], 'LineWidth',3); hold off
legend(["X", "Y", "Z", "X_d", "Y_d", "Z_d"])
xlabel("Timestep")
ylabel("Position (m)")
title("Position vs Timestep")

%% Force / Torque vs Timestep
plot([tab.Fx, tab.Fy, tab.Fz, tab.Tx, tab.Ty, tab.Tz], 'LineWidth',3); hold on
plot(tab.Fz_des*1.05, 'r')
plot(tab.Fz_des*0.95, 'r')
plot(tab.Fz_des); hold off
legend(["F_x", "F_y", "F_z","T_x", "T_y", "T_z", "F_d"])
xlabel("Timestep")
ylabel("Force (N) / Torque (N\cdot{}m)")
title("Force / Torque vs Timestep")

%% Force vs X Position
plot(tab.EGM_X, tab.Fz, '.'); hold on
plot(tab.Commanded_X, tab.Fz_des*1.05, 'r');
plot(tab.Commanded_X, tab.Fz_des*0.95, 'r');
plot(tab.Commanded_X, tab.Fz_des); hold off
xlabel("X Position (m)")
ylabel("Force (N)")
legend(["Measured", "Desired"])
title("Force vs X Position")

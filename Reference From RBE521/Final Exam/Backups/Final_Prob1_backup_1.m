%% Kyle Mitchell
% RBE 521 Final Exam

close all;
%% Problem 1

% From HW 6 and problem statement:
v = 0.01; % m/s
duty = 4/6;
L = 0.030; % m
leg_lengths = [0, 0.050, 0.100]; % m
liftoff_height = 0.005; % m
body_height = 0.1; % m
body_l = 0.4; % m
body_w = 0.2; % m
body_h = 0.1; % m
D = leg_lengths(1) + leg_lengths(2) + (body_w/2);

% Q1
u_fb = duty / (1 - duty) * v; % m/s

% Q2
u_fg = v / (1 - duty); % m/s

% Q3
num_contact_legs = 4;

% Q4
phi_start = [4/6, 1/6, 2/6, 5/6, 0/6, 3/6]; % start of transfer phase
phi_end = [0/6, 3/6, 4/6, 1/6, 2/6, 5/6]; % end of transfer phase

% Q5
T = L / v; % sec

%% Q6
T_s = T * duty; % support time (sec)
T_t = T - T_s; % transfer time (sec)

% Define time steps and initialize generic x and z position vectors
t_steps = [0, (1 * T_t)/5, (2 * T_t)/5, (3 * T_t)/5, (4 * T_t)/5, T_t];
%t_steps = [0, (1 * T_t)/10, (2 * T_t)/10, (3 * T_t)/10, (4 * T_t)/10, (5 * T_t)/10, (6 * T_t)/10, (7 * T_t)/10, (8 * T_t)/10, (9 * T_t)/10, T_t];
%interval = 0.1;
x_pos_generic = zeros(1,size(t_steps,2));
y_pos_generic = zeros(1,size(t_steps,2));
z_pos_generic = zeros(1,size(t_steps,2));

% Calculate generic x and z positions of leg in transfer wrt ground
for i=1:size(t_steps,2)
    if i == 1 % if liftoff start time
        x_pos_generic(i) = 0;
        z_pos_generic(i) = 0;
    elseif i == 2 % if liftoff end time
        x_pos_generic(i) = 0;
        z_pos_generic(i) = liftoff_height;
    elseif i == (size(t_steps,2) - 1) % if touchdown start time
        x_pos_generic(i) = L;
        z_pos_generic(i) = liftoff_height;
    elseif i == size(t_steps,2) % if touchdown end time
        x_pos_generic(i) = L;
        z_pos_generic(i) = 0;
    else % if between liftoff and touchdown
        x_pos_generic(i) = L / 3 * ((t_steps(i) - 0.2) / 0.2);
        z_pos_generic(i) = liftoff_height;
        %sqrt(0.015^2 - (x_pos_generic(i) - 0.015)^2) - 0.005;
    end
end

% Center the x trajectory by subtracting its position by L/2
for i = 1:size(t_steps,2)
    x_pos_generic(i) = x_pos_generic(i) - L/2;
end

% Initialize generic x and z velocity vectors
x_vel_generic = zeros(1,6);
z_vel_generic = zeros(1,6);

% Calculate generic x and z velocities of one leg in transfer wrt ground
for i=1:size(t_steps,2)
    if i == 1 % if liftoff start time
        x_vel_generic(i) = 0;
        z_vel_generic(i) = 0;
    elseif i == 2 % if liftoff end time
        x_vel_generic(i) = 0;
        z_vel_generic(i) = liftoff_height / (t_steps(i) - t_steps(i-1));
    elseif i == (size(t_steps,2) - 1) % if touchdown start time
        x_vel_generic(i) = 0;
        z_vel_generic(i) = -liftoff_height / (t_steps(end) - t_steps(end-1));
    elseif i == size(t_steps,2) % if touchdown end time
        x_vel_generic(i) = 0;
        z_vel_generic(i) = 0;
    else % if between liftoff and touchdown
        x_vel_generic(i) = (x_pos_generic(i) - x_pos_generic(i-1)) / (t_steps(i) - t_steps(i-1));
        z_vel_generic(i) = (z_pos_generic(i) - z_pos_generic(i-1)) / (t_steps(i) - t_steps(i-1));
    end
end

% Plot generic x and z positions and velocities of one leg in transfer wrt ground
% figure
% tiledlayout(2,2);
% 
% nexttile
% plot(t_steps,x_pos_generic)
% set(gca, 'xtick', 0:0.2:1);
% xlabel('Transfer Time')
% ylabel('X Position of Foot wrt Ground')
% for point = 1:6
%   thisX = t_steps(point);
%   thisY = x_pos_generic(point);
%   labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
%   text(thisX, thisY, labelstr);
% end
% 
% nexttile
% plot(t_steps,z_pos_generic)
% set(gca, 'xtick', 0:0.2:1);
% xlabel('Transfer Time')
% ylabel('Z Position of Foot wrt Ground')
% for point = 1:6
%   thisX = t_steps(point);
%   thisY = z_pos_generic(point);
%   labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
%   text(thisX, thisY, labelstr);
% end
% 
% nexttile
% plot(t_steps,x_vel_generic)
% set(gca, 'xtick', 0:0.2:1);
% xlabel('Transfer Time')
% ylabel('X Velocity of Foot wrt Ground')
% for point = 1:6
%   thisX = t_steps(point);
%   thisY = x_vel_generic(point);
%   labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
%   text(thisX, thisY, labelstr);
% end
% 
% nexttile
% plot(t_steps,z_vel_generic)
% set(gca, 'xtick', 0:0.2:1);
% xlabel('Transfer Time')
% ylabel('Z Velocity of Foot wrt Ground')
% for point = 1:6
%   thisX = t_steps(point);
%   thisY = z_vel_generic(point);
%   labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
%   text(thisX, thisY, labelstr);
% end

%% Q7

% Start by applying generic foot trajectory to each leg based on relative
% phase of the leg in the overall cycle
t_cycle = [0:0.1:T];

transfer_start_time = phi_start * T; % start time of transfer phase of each leg in a total cycle
x_pos = zeros(6, 16);

leg1_joints = calc_joint_angles(1, x_pos_generic, z_pos_generic);
leg2_joints = calc_joint_angles(2, x_pos_generic, z_pos_generic);
leg3_joints = calc_joint_angles(3, x_pos_generic, z_pos_generic);
leg4_joints = calc_joint_angles(4, x_pos_generic, z_pos_generic);
leg5_joints = calc_joint_angles(5, x_pos_generic, z_pos_generic);
leg6_joints = calc_joint_angles(6, x_pos_generic, z_pos_generic);





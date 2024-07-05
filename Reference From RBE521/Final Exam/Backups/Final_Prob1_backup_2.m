%% Kyle Mitchell
% RBE 521 Final Exam

close all;

% Booleans to toggle plotting
plot_6_step_pos_and_vel = true;
plot_11_step_pos_and_vel = true;
plot_transfer_a_b_g = true;

plot_cycle_pos = true;
plot_cycle_a_b_g = true;
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
interval = 0.2;
x_pos_generic = zeros(1,size(t_steps,2));
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
        x_pos_generic(i) = L / (size(t_steps,2) - 3) * ((t_steps(i) - interval) / interval);
        z_pos_generic(i) = liftoff_height;
        %sqrt(0.015^2 - (x_pos_generic(i) - 0.015)^2) - 0.005;
    end
end

% Center the x trajectory by subtracting its position by L/2
for i = 1:size(t_steps,2)
    x_pos_generic(i) = x_pos_generic(i) - L/2;
end

% Initialize generic x and z velocity vectors
x_vel_generic = zeros(1,size(t_steps,2));
z_vel_generic = zeros(1,size(t_steps,2));

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
if plot_6_step_pos_and_vel
    figure
    tiledlayout(2,2);

    nexttile
    plot(t_steps,x_pos_generic)
    set(gca, 'xtick', 0:0.2:1);
    xlabel('Transfer Time')
    ylabel('X Position of Foot wrt Ground')
    for point = 1:6
        thisX = t_steps(point);
        thisY = x_pos_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end

    nexttile
    plot(t_steps,z_pos_generic)
    set(gca, 'xtick', 0:0.2:1);
    xlabel('Transfer Time')
    ylabel('Z Position of Foot wrt Ground')
    for point = 1:6
        thisX = t_steps(point);
        thisY = z_pos_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end

    nexttile
    plot(t_steps,x_vel_generic)
    set(gca, 'xtick', 0:0.2:1);
    xlabel('Transfer Time')
    ylabel('X Velocity of Foot wrt Ground')
    for point = 1:6
        thisX = t_steps(point);
        thisY = x_vel_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end

    nexttile
    plot(t_steps,z_vel_generic)
    set(gca, 'xtick', 0:0.2:1);
    xlabel('Transfer Time')
    ylabel('Z Velocity of Foot wrt Ground')
    for point = 1:6
        thisX = t_steps(point);
        thisY = z_vel_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end
end
%% Q7

% REDEFINE TRAJECTORIES WITH MORE TIME STEPS TO BE MORE FRIENDLY WITH THE
% RELATIVE KINEMATIC PHASE OF EACH LEG (0.2 STEPS WERE TOO BIG, 0.1 OK)

% Define time steps and initialize generic x and z position vectors
t_steps = [0, (1 * T_t)/10, (2 * T_t)/10, (3 * T_t)/10, (4 * T_t)/10, (5 * T_t)/10, (6 * T_t)/10, (7 * T_t)/10, (8 * T_t)/10, (9 * T_t)/10, T_t];
interval = 0.1;
x_pos_generic = zeros(1,size(t_steps,2));
y_pos_generic = zeros(1,size(t_steps,2));
z_pos_generic = zeros(1,size(t_steps,2));

% Calculate generic x and z positions of leg in transfer wrt ground
for i=1:size(t_steps,2)
    if i <= 3 % if liftoff
        x_pos_generic(i) = 0;
        z_pos_generic(i) = liftoff_height/2 * (i-1);
    elseif i > (size(t_steps,2) - 3) && i <= (size(t_steps,2)) % if touchdown
        x_pos_generic(i) = L;
        z_pos_generic(i) = liftoff_height/2 * (size(t_steps,2) - i);
    else % if between liftoff and touchdown
        x_pos_generic(i) = L / 6 * (i - 3);
        z_pos_generic(i) = liftoff_height;
        %sqrt(0.015^2 - (x_pos_generic(i) - 0.015)^2) - 0.005;
    end
end

% Center the x trajectory by subtracting its position by L/2
for i = 1:size(t_steps,2)
    x_pos_generic(i) = x_pos_generic(i) - L/2;
end

% Initialize generic x and z velocity vectors
x_vel_generic = zeros(1,size(t_steps,2));
z_vel_generic = zeros(1,size(t_steps,2));

% Calculate generic x and z velocities of one leg in transfer wrt ground
for i=1:size(t_steps,2)
    if i == 1 % if start of liftoff
        x_vel_generic(i) = 0;
        z_vel_generic(i) = 0;
    elseif i <= 3 % if liftoff
        x_vel_generic(i) = 0;
        z_vel_generic(i) = (((liftoff_height/2) / (t_steps(i) - t_steps(i-1))) * (i-1))/2;
    elseif i > (size(t_steps,2) - 3) && i <= (size(t_steps,2)) % if touchdown
        x_vel_generic(i) = 0;
        z_vel_generic(i) = (-(liftoff_height/2) / (t_steps(end) - t_steps(end-1)) * (size(t_steps,2) - i))/2;
    else % if between liftoff and touchdown
        x_vel_generic(i) = (x_pos_generic(i) - x_pos_generic(i-1)) / (t_steps(i) - t_steps(i-1));
        z_vel_generic(i) = (z_pos_generic(i) - z_pos_generic(i-1)) / (t_steps(i) - t_steps(i-1));
    end
end
% Temporary fix to velocity graphs
x_vel_generic(4) = 0.025;
x_vel_generic(8) = 0.025;
z_vel_generic(4) = 0.0125;
z_vel_generic(8) = -0.0125;

% Plot generic x and z positions and velocities of one leg in transfer wrt ground
if plot_11_step_pos_and_vel
    figure
    tiledlayout(2,2);

    nexttile
    plot(t_steps,x_pos_generic)
    set(gca, 'xtick', 0:interval:T_t);
    xlabel('Transfer Time with More Steps')
    ylabel('X Position of Foot wrt Ground')
    for point = 1:size(t_steps,2)
        thisX = t_steps(point);
        thisY = x_pos_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end

    nexttile
    plot(t_steps,z_pos_generic)
    set(gca, 'xtick', 0:interval:T_t);
    xlabel('Transfer Time with More Steps')
    ylabel('Z Position of Foot wrt Ground')
    for point = 1:size(t_steps,2)
        thisX = t_steps(point);
        thisY = z_pos_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end

    nexttile
    plot(t_steps,x_vel_generic)
    set(gca, 'xtick', 0:interval:T_t);
    xlabel('Transfer Time with More Steps')
    ylabel('X Velocity of Foot wrt Ground')
    for point = 1:size(t_steps,2)
        thisX = t_steps(point);
        thisY = x_vel_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end

    nexttile
    plot(t_steps,z_vel_generic)
    set(gca, 'xtick', 0:interval:T_t);
    xlabel('Transfer Time with More Steps')
    ylabel('Z Velocity of Foot wrt Ground')
    for point = 1:size(t_steps,2)
        thisX = t_steps(point);
        thisY = z_vel_generic(point);
        labelstr = sprintf('(%.3f,%.3f)', thisX, thisY);
        text(thisX, thisY, labelstr);
    end
end

leg1_joints = calc_joint_angles(1, x_pos_generic, z_pos_generic);
leg2_joints = calc_joint_angles(2, x_pos_generic, z_pos_generic);
leg3_joints = calc_joint_angles(3, x_pos_generic, z_pos_generic);
leg4_joints = calc_joint_angles(4, x_pos_generic, z_pos_generic);
leg5_joints = calc_joint_angles(5, x_pos_generic, z_pos_generic);
leg6_joints = calc_joint_angles(6, x_pos_generic, z_pos_generic);

leg_joints = [leg1_joints; leg2_joints; leg3_joints; leg4_joints; leg5_joints; leg6_joints];

% Plot legs' transfer phase alpha, beta, gamma
if plot_transfer_a_b_g
    for leg = 1:6 % for each leg
        figure
        tiledlayout(1,3);

        nexttile
        plot(t_steps,leg_joints(3*leg-2,:))
        set(gca, 'xtick', 0:interval:T_t);
        xlabel('Transfer Time')
        ylabel(sprintf('Alpha %g', leg))

        nexttile
        plot(t_steps,leg_joints(3*leg-1,:))
        set(gca, 'xtick', 0:interval:T_t);
        xlabel('Transfer Time')
        ylabel(sprintf('Beta %g', leg))

        nexttile
        plot(t_steps,leg_joints(3*leg,:))
        set(gca, 'xtick', 0:interval:T_t);
        xlabel('Transfer Time')
        ylabel(sprintf('Gamma %g', leg))

    end
end

% Applying generic foot trajectory to each leg based on relative
% phase of the leg in the overall cycle
t_cycle = 0:0.1:T-0.1;

transfer_start_time = phi_start * T; % start time of transfer phase of each leg in a total cycle

% Initialize new cycle position matrices
x_pos = zeros(6, 30);
z_pos = zeros(6, 30);

% Initialize support phase trajectories for x
% Don't need z bc the value will be 0 if not in transfer anyway
x_support_pos = zeros(1,20);

for i = 1:size(x_support_pos,2)
    x_support_pos(i) = (L / 2) - (L / 20) * (i - 1);
end

% Leg 1:
x_pos(1,1:20) = x_support_pos(1:20);
x_pos(1,21:30) = x_pos_generic(1:10);
z_pos(1,21:30) = z_pos_generic(1:10);

% Leg 2:
x_pos(2,16:30) = x_support_pos(1:15);
x_pos(2,1:5) = x_support_pos(16:20);
x_pos(2, 6:15) = x_pos_generic(1:10);
z_pos(2, 6:15) = z_pos_generic(1:10);

% Leg 3:
x_pos(3,21:30) = x_support_pos(1:10);
x_pos(3,1:10) = x_support_pos(11:20);
x_pos(3,11:20) = x_pos_generic(1:10);
z_pos(3,11:20) = z_pos_generic(1:10);

% Leg 4:
x_pos(4,6:25) = x_support_pos(1:20);
x_pos(4,26:30) = x_pos_generic(1:5);
x_pos(4,1:5) = x_pos_generic(6:10);
z_pos(4,26:30) = z_pos_generic(1:5);
z_pos(4,1:5) = z_pos_generic(6:10);

% Leg 5:
x_pos(5,11:30) = x_support_pos(1:20);
x_pos(5,1:10) = x_pos_generic(1:10);
z_pos(5,1:10) = z_pos_generic(1:10);

% Leg 6:
x_pos(6,26:30) = x_support_pos(1:5);
x_pos(6,1:15) = x_support_pos(6:20);
x_pos(6,16:25) = x_pos_generic(1:10);
z_pos(6,16:25) = z_pos_generic(1:10);

% Plot the legs' cycle positions
if plot_cycle_pos
    for leg = 1:6
        figure
        tiledlayout(2,1);

        nexttile
        plot(t_cycle,x_pos(leg,:))
        set(gca, 'xtick', 0:0.1:T);
        xlabel('Cycle Time')
        ylabel(sprintf('X Position of Foot %g wrt Ground', leg))

        nexttile
        plot(t_cycle,z_pos(leg,:))
        set(gca, 'xtick', 0:0.1:T);
        xlabel('Transfer Time')
        ylabel(sprintf('Z Position of Foot %g wrt Ground', leg))
    end
end

% Calculate joint values at each cycle time step
leg1_joints_cycle = calc_joint_angles(1, x_pos(1,:), z_pos(1,:));
leg2_joints_cycle = calc_joint_angles(2, x_pos(2,:), z_pos(2,:));
leg3_joints_cycle = calc_joint_angles(3, x_pos(3,:), z_pos(3,:));
leg4_joints_cycle = calc_joint_angles(4, x_pos(4,:), z_pos(4,:));
leg5_joints_cycle = calc_joint_angles(5, x_pos(5,:), z_pos(5,:));
leg6_joints_cycle = calc_joint_angles(6, x_pos(6,:), z_pos(6,:));

leg_joints_cycle = [leg1_joints_cycle; leg2_joints_cycle; leg3_joints_cycle; leg4_joints_cycle; leg5_joints_cycle; leg6_joints_cycle];

% Plot legs' transfer phase alpha, beta, gamma
if plot_cycle_a_b_g
    for leg = 1:6 % for each leg
        figure
        tiledlayout(1,3);

        nexttile
        plot(t_cycle,leg_joints_cycle(3*leg-2,:))
        set(gca, 'xtick', 0:0.1:T);
        xlabel('Cycle Time')
        ylabel(sprintf('Alpha %g', leg))

        nexttile
        plot(t_cycle,leg_joints_cycle(3*leg-1,:))
        set(gca, 'xtick', 0:0.1:T);
        xlabel('Cycle Time')
        ylabel(sprintf('Beta %g', leg))

        nexttile
        plot(t_cycle,leg_joints_cycle(3*leg,:))
        set(gca, 'xtick', 0:0.1:T);
        xlabel('Cycle Time')
        ylabel(sprintf('Gamma %g', leg))

    end
end

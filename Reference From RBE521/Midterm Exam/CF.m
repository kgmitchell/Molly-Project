function error = CF(x)

% x = real kinematic parameters

% Pods with nominal values (from Table 1 in Paper 3):
% pod#nom = [Si_x, Si_y, Si_z, Ui_x, Ui_y, Ui_z, Li_0]; % dim in mm
pod1nom = [92.1597,     84.4488,  0,   305.4001,   111.1565,  0,  604.8652];
pod2nom = [27.055,     122.0370,  0,   -56.4357,   320.0625,  0,  604.8652];
pod3nom = [-119.2146,   37.5882,  0,  -248.9644,   208.9060,  0,  604.8652];
pod4nom = [-119.2146,  -37.5882,  0,  -248.9644,  -208.9060,  0,  604.8652];
pod5nom = [27.055,    -122.0370,  0,   -56.4357,  -320.0625,  0,  604.8652];
pod6nom = [92.1597,    -84.4488,  0,   305.4001,  -111.1565,  0,  604.8652];
podsnom = [pod1nom; pod2nom; pod3nom; pod4nom; pod5nom; pod6nom];



% Minimum of 10 configurations around the workspace
% Each row = [x, y, z, a, b, c]
configs = [   0    0 900 0 0 0;
              0    0  600 0 0 0;
            %825    0  235 0 0 0;
            %760    0  450 0 0 0;
            615    0  690 5 -5 2;
            404    0  885 2 3 -1;
           -404    0  885 0 0 0;
           -615    0  690 0 0 0;
           %-760    0  450 0 0 0;
           %-825    0  235 0 0 0;
              %0  825  235 0 0 0;
              %0  760  450 0 0 0;
               0  615  690 1 6 -4;
               0  404  885 -4 3 1;
               0 -404  885 0 0 0;
               0 -615  690 0 0 0;
              %0 -760  450 0 0 0;
              %0 -825  235 0 0 0
               ]';

% Preallocate space for better computation speed
m = size(configs,2);
measured_pos = zeros(6,m);
R_vector = zeros(3,3*m);
L = zeros(3,6);
error = zeros(1,6*m);

[not_L, l, not_n, not_s, not_u, not_R] = RIK_MT([0 0 873.9652 0 0 0]');
lo = l'; % Nominal initial leg lengths

%lo_error = x(7,:)' - podsnom(:, 7);

% Simulation
% Convert given config to RFK config
for i = 1:m
    [not_L, curr_l, not_n, not_s, not_u, curr_R] = IK_MT(configs(:, i));
    measured_pos(:, i) = RFK_MT(configs(:,i), curr_l');
    %[not_L, not_l, not_n, not_s, not_u, curr_R] = IK_MT(configs(:, i));
    R_vector(:,3*i-2:3*i) = curr_R;
end

j = 0; % Will iterate to 6 * number of configs
for k = 1:m % For every configuration
    for i = 1:6 % For every leg (6 legs)
        j = j + 1;
        L(:,i) = configs(1:3,k) + R_vector(:,3*k-2:3*k) * podsnom(i,1:3)' - podsnom(i,4:6)'; % Nominal IK
        l(i) = norm(L(:,i),2); % Nominal leg lengths
        error(j) = ((norm(measured_pos(1:3,k) + R_vector(:,3*k-2:3*k) * x(1:3,i) - x(4:6,i)))^2 - (norm(x(7,i) + l(i) - 604.8652))^2)^2; 
    end
end
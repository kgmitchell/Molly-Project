% This function solves the direct kinematics of hexapod mechanism. The function takes the initial guess (P0) 
% (position and orientation of the upper platform) together with a vector containing leg 
% lengths (lg) as the input and returns the 
% approximation of the upper platform's position and orientation with the 
% specified precision/accuracy. The precision is considered to be 0.000001 mm.

function p = RFK_MT(P0, lg)

delta_p = 1; % Initial value that is > desired precision

% Robot Parameters (Given)
% Rm = 250.9339/2; % Radius of top platform
% Rf = 634.2376/2; % Radius of bottom platform
% alpha = 40*pi/180; % angle between legs at bottom platform
% beta = 85*pi/180; % angle between legs at top platform

% Pods with nominal values (from Table 1 in Paper 3):
% pod#nom = [Si_x, Si_y, Si_z, Ui_x, Ui_y, Ui_z, Li_0]; % dim in mm
pod1nom = [92.1597,     84.4488,  0,   305.4001,   111.1565,  0,  604.8652];
pod2nom = [27.055,     122.0370,  0,   -56.4357,   320.0625,  0,  604.8652];
pod3nom = [-119.2146,   37.5882,  0,  -248.9644,   208.9060,  0,  604.8652];
pod4nom = [-119.2146,  -37.5882,  0,  -248.9644,  -208.9060,  0,  604.8652];
pod5nom = [27.055,    -122.0370,  0,   -56.4357,  -320.0625,  0,  604.8652];
pod6nom = [92.1597,    -84.4488,  0,   305.4001,  -111.1565,  0,  604.8652];
podsnom = [pod1nom; pod2nom; pod3nom; pod4nom; pod5nom; pod6nom];

% Pods with simulated real values (from Table 2 in Paper 3):
% pod#nom = [Si_x, Si_y, Si_z, Ui_x, Ui_y, Ui_z, Li_0]; % dim in mm
pod1simr = [96.6610,     81.7602,   1.0684,   305.2599,   115.0695,   2.6210,  604.4299];
pod2simr = [22.2476,    125.2511,  -0.5530,   -55.2814,   322.9819,   4.2181,  607.2473];
pod3simr = [-122.4519,   36.6453,   4.3547,  -244.7954,   208.0087,   3.9365,  600.4441];
pod4simr = [-120.6859,  -34.4565,  -4.9014,  -252.5755,  -211.8783,  -3.0128,  605.9031];
pod5simr = [24.7769,   -125.0489,  -4.8473,   -53.9678,  -320.6115,   4.3181,  604.5251];
pod6simr = [91.3462,    -80.9866,   0.2515,   302.4266,  -109.4351,   3.3812,  600.0616];
podssimr = [pod1simr; pod2simr; pod3simr; pod4simr; pod5simr; pod6simr];

lo_error = podssimr(:, 7) - podsnom(:, 7);

% Step #1: Initial Guess
%P0 = [0 0 600 0 0 0]';
P(:,1) = P0;
i = 2;
while delta_p > 0.000001 % Step #6: See if delta_p is <<
    
    %o=P(1:3,i-1);
    
    % Step #2: Calculate Jacobian
    J = jacobianV_MT(P(:,i-1));
    
    % Step #3: Calculate T
    %theta = P(4:6,i-1); % Store a, b, and c in the theta vector
    a = P(4,i-1)*pi/180;
    b = P(5,i-1)*pi/180;
    c = P(6,i-1)*pi/180;

    B = [1     0       sin(b);
         0     cos(a) -cos(b)*sin(a);
         0     sin(a)  cos(b)*cos(a)]; % B of XYZ Euler

    T = [eye(3)       zeros(3,3);
        zeros(3,3)   B];
    % Rp is used to represent the Euler angles' error in the base frame 
%     Rp=[1,0,0,0,0,0;
%         0,1,0,0,0,0;
%         0,0,1,0,0,0;
%         0,0,0,1,0,sin(theta(2)*pi/180);
%         0,0,0,0,cos(theta(1)*pi/180),-sin(theta(1)*pi/180)*cos(theta(2)*pi/180);
%         0,0,0,0,sin(theta(1)*pi/180),cos(theta(1)*pi/180)*cos(theta(2)*pi/180)];

    % Step #4: IK
    [L, l, n] = IK_MT(P(:,i-1));
    delta_l_vector = (lg + lo_error) - l';
    delta_l = norm(delta_l_vector, 2);
    
    
    % Step #5: Substitution to calculate Pi vector
    P(:,i) = P(:,i-1) + pinv(J*T) * delta_l_vector;
    delta_p = norm(P(:,i) - P(:,i-1), 2);
    i = i + 1;
end
p = P(:,end);
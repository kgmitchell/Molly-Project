% This function solves the direct kinematics of hexapod mechanism. The function takes the initial guess (P0) 
% (position and orientation of the upper platform) together with a vector containing leg 
% lengths (lg) as the input and returns the 
% approximation of the upper platform's position and orientation with the 
% specified precision/accuracy. The precision is considered to be 0.000001 mm.

function p = FK_MT(lg)

delta_l = 1; % Initial value that is > desired precision

% Robot Parameters (Given)
% Rm = 250.9339/2; % Radius of top platform
% Rf = 634.2376/2; % Radius of bottom platform
% alpha = 40*pi/180; % angle between legs at bottom platform
% beta = 85*pi/180; % angle between legs at top platform

% Step #1: Initial Guess
P0 = [0 0 600 0 0 0]';
P(:,1) = P0;
i = 2;
while delta_l > 0.000001 % Step #6: See if delta_l is <<
    
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
    delta_l_vector = lg - l';
    delta_l = norm(delta_l_vector, 2);
    
    
    % Step #5: Substitution to calculate Pi vector
    P(:,i) = P(:,i-1) + pinv(J*T) * delta_l_vector;
    delta_p = norm(P(:,i) - P(:,i-1), 2);
    i = i + 1;
end
p = P(:,end);
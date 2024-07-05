% Inverse Kinematics

function [L, l, n, s, u, R] = IK_MT(P)

% Robot Parameters (Given)
%Rm = 250.9339/2; % Radius of top platform
%Rf = 634.2376/2; % Radius of bottom platform
Rm = 250/2; % Radius of top platform
Rf = 650/2; % Radius of bottom platform
alpha = 40*pi/180; % angle between legs at bottom platform
beta = 85*pi/180; % angle between legs at top platform

% Extracting Position and Euler Angle Information from the given desired pose
o=P(1:3,1);
a=P(4)*pi/180;
b=P(5)*pi/180;
c=P(6)*pi/180;

% XYZ Rotation Matrix (R)
R1=[1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)]; % Rx,a
R2=[cos(b), 0, sin(b); 0, 1, 0; -sin(b), 0, cos(b)]; % Ry,b
R3=[cos(c), -sin(c), 0; sin(c), cos(c), 0; 0, 0, 1]; % Rz,c
R=R1*R2*R3;

% % Calculating upper joint positions wrt. the upper coordinate frame
% s = [Rm*cos(beta/2), -Rm*sin(pi/6-beta/2), -Rm*sin(pi/6+beta/2), -Rm*cos(pi/3-beta/2), -Rm*cos(pi/3+beta/2),  Rm*cos(beta/2);
%      Rm*sin(beta/2),  Rm*cos(pi/6-beta/2),  Rm*cos(pi/6+beta/2), -Rm*sin(pi/3-beta/2), -Rm*sin(pi/3+beta/2), -Rm*sin(beta/2);
%      0,               0,                    0,                    0,                    0,                    0];
% 
% % Calculating lower joint positions wrt. the lower coordinate frame
% u = [Rf*cos(alpha/2), -Rf*sin(pi/6-alpha/2), -Rf*sin(pi/6+alpha/2), -Rf*cos(pi/3-alpha/2), -Rf*cos(pi/3+alpha/2),  Rf*cos(alpha/2);
%      Rf*sin(alpha/2),  Rf*cos(pi/6-alpha/2),  Rf*cos(pi/6+alpha/2), -Rf*sin(pi/3-alpha/2), -Rf*sin(pi/3+alpha/2), -Rf*sin(alpha/2);
%      0,                0,                     0,                     0,                     0,                     0];

% Pods with nominal values (from Table 1 in Paper 3):
% pod#nom = [Si_x, Si_y, Si_z, Ui_x, Ui_y, Ui_z, Li_0]; % dim in mm
pod1nom = [92.1597,     84.4488,  0,   305.4001,   111.1565,  0,  604.8652];
pod2nom = [27.055,     122.0370,  0,   -56.4357,   320.0625,  0,  604.8652];
pod3nom = [-119.2146,   37.5882,  0,  -248.9644,   208.9060,  0,  604.8652];
pod4nom = [-119.2146,  -37.5882,  0,  -248.9644,  -208.9060,  0,  604.8652];
pod5nom = [27.055,    -122.0370,  0,   -56.4357,  -320.0625,  0,  604.8652];
pod6nom = [92.1597,    -84.4488,  0,   305.4001,  -111.1565,  0,  604.8652];
podsnom = [pod1nom; pod2nom; pod3nom; pod4nom; pod5nom; pod6nom];

s = [pod1nom(1,1:3)', pod2nom(1,1:3)', pod3nom(1,1:3)', pod4nom(1,1:3)', pod5nom(1,1:3)', pod6nom(1,1:3)'];
u = [pod1nom(1,4:6)', pod2nom(1,4:6)', pod3nom(1,4:6)', pod4nom(1,4:6)', pod5nom(1,4:6)', pod6nom(1,4:6)'];

% Put it in the IK equation
for i=1:6
    L(:,i) = o + R * s(:,i) - u(:,i); % legs vector
    l(i) = norm(L(:,i),2); % legs vector magnitude
    n(:,i) = L(:,i)/l(i); % legs unit vector
end
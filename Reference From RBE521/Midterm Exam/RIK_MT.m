% 'Real' Inverse Kinematics

function [L, l, n, s, u, R] = RIK_MT(P)

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

% Pods with simulated real values (from Table 2 in Paper 3):
% pod#simr = [Si_x, Si_y, Si_z, Ui_x, Ui_y, Ui_z, Li_0]; % dim in mm
pod1simr = [96.6610,     81.7602,   1.0684,   305.2599,   115.0695,   2.6210,  604.4299];
pod2simr = [22.2476,    125.2511,  -0.5530,   -55.2814,   322.9819,   4.2181,  607.2473];
pod3simr = [-122.4519,   36.6453,   4.3547,  -244.7954,   208.0087,   3.9365,  600.4441];
pod4simr = [-120.6859,  -34.4565,  -4.9014,  -252.5755,  -211.8783,  -3.0128,  605.9031];
pod5simr = [24.7769,   -125.0489,  -4.8473,   -53.9678,  -320.6115,   4.3181,  604.5251];
pod6simr = [91.3462,    -80.9866,   0.2515,   302.4266,  -109.4351,   3.3812,  600.0616];
podssimr = [pod1simr; pod2simr; pod3simr; pod4simr; pod5simr; pod6simr];

s = [pod1simr(1,1:3)', pod2simr(1,1:3)', pod3simr(1,1:3)', pod4simr(1,1:3)', pod5simr(1,1:3)', pod6simr(1,1:3)'];
u = [pod1simr(1,4:6)', pod2simr(1,4:6)', pod3simr(1,4:6)', pod4simr(1,4:6)', pod5simr(1,4:6)', pod6simr(1,4:6)'];

% Put it in the IK equation
for i=1:6
    L(:,i) = o + R * s(:,i) - u(:,i); % legs vector
    l(i) = norm(L(:,i),2); % legs vector magnitude
    n(:,i) = L(:,i)/l(i); % legs unit vector
end
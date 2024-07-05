% Velocity Jacobian

function J = jacobianV_MT(P)

% Rm = 250.9339/2;
% Rf = 634.2376/2;
% alpha = 40*pi/180;
% beta = 85*pi/180;
% Xp = P(1:3,1);
% theta(1) = P(4)*pi/180;
% theta(2) = P(5)*pi/180;
% theta(3) = P(6)*pi/180;

[L, l, n, s, u, R] = IK_MT(P);

J = [n(:,1)' , cross(R * s(:,1), n(:,1))';
     n(:,2)' , cross(R * s(:,2), n(:,2))';
     n(:,3)' , cross(R * s(:,3), n(:,3))';
     n(:,4)' , cross(R * s(:,4), n(:,4))';
     n(:,5)' , cross(R * s(:,5), n(:,5))';
     n(:,6)' , cross(R * s(:,6), n(:,6))'];
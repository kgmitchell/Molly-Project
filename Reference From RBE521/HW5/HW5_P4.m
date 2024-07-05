clear all;
clc;
close;
%% ----------------------- *** INITIAL DATA *** ------------------------%%
l1=.4;
l2=.4;
v=0.1; % desired velocity of COG (m/sec)
u_g = 0.9; % foot average horizontal velocity wrt ground
b = 0.75; % duty factor
u_b=(b/(1-b))*v; % average swing velocity of the leg wrt the body
u=v/(1-b); % average swing velocity of the leg wrt the ground
L=0.4; % Stride Length which is the amount of COG displacement in one cycle time.
T= L/v; % Cycle Period
Tt=(1-b)*T; % transfer time
Ts=b*T; % support time
h=0.6; % height of the robot, constant
%% ------------------- *** GAIT GENERATION *** ---------------------- %%
p(1)=0; % Kinematic phase of leg 1
p(2)=p(1)+1/2; % Kinematic phase of leg 2
p(3)=p(1)+b; % Kinematic phase of leg 3
p(4)=p(2)+b; % Kinematic phase of leg 4

%What if p(i) is equal or greater than 1?
    for i=1:size(p,2)
        while p(i) >= 1
            p(i) = p(i) - 1;
        end
    end
p % When touchdown occurs
%% ------------- *** POSITIONAL FOOT TRAJECTORY PLANNING *** ------------%%
tt = 0.01:0.01:Tt;
st = 0.01:0.01:Ts;
ct = 0.01:0.01:T;

delta_body = ones(1,size(ct,2));

for i = 1:size(delta_body,2)
    delta_body(i) = v * ct(i);
end

%%%%%%%%%%HERE EVERY THING IS IN Gi COORDINATE SYSTEM WHICH IS GROUND
%COORDINATE SYSTEM FOR EACH LEG

%Let's find positions:
for i=1:size(tt,2)
    if tt(i) <= 0.1 * Tt % Liftoff
        xf_g_Tt(1:4,i)=0;
        zf_g_Tt(1:4,i)=0.1 * (10*tt(i));
    elseif tt(i) > 0.9 * Tt % Touchdown
        xf_g_Tt(1:4,i)=xf_g_Tt(1:4,i-1);
        zf_g_Tt(1:4,i)= zf_g_Tt(1:4,i-1) - 0.01;
    else
        xf_g_Tt(1:4,i) = L/0.8 * (tt(i)-0.1*Tt);
        zf_g_Tt(1:4,i)= 0.5 * sqrt((0.4 * Tt)^2-(tt(i)-(0.5*Tt))^2) + 0.09 * Tt;
    end
end

% Put positions into total cycle
xf_g = zeros(4,400);
zf_g = zeros(4,400);

xf_gtemp = zeros(4,400);
zf_gtemp = zeros(4,400);

xf_g(1,301:400)=xf_g_Tt(1,:);
xf_g(2,101:200)=xf_g_Tt(2,:);
xf_g(3,201:300)=xf_g_Tt(3,:);
xf_g(4,1:100)=xf_g_Tt(4,:);

xf_gtemp(1,301:400)=xf_g_Tt(1,:);

xf_gtemp(2,101:200)=xf_g_Tt(2,:);
for i = 201:400
    xf_gtemp(2,i)=xf_g_Tt(2,100);
end

xf_gtemp(3,201:300)=xf_g_Tt(3,:);
for i = 301:400
    xf_gtemp(3,i)=xf_g_Tt(3,100);
end

xf_gtemp(4,1:100)=xf_g_Tt(4,:);
for i = 101:400
    xf_gtemp(4,i)=xf_g_Tt(4,100);
end

xf_g_withbody(1,:) = xf_gtemp(1,:) - delta_body;
xf_g_withbody(2,:) = xf_gtemp(2,:) - delta_body;
xf_g_withbody(3,:) = xf_gtemp(3,:) - delta_body;
xf_g_withbody(4,:) = xf_gtemp(4,:) - delta_body;

for i=1:400
    xf_g_withbody(1,i) = xf_g_withbody(1,i) + 0.5 * (L - 0.8*v);
    xf_g_withbody(2,i) = xf_g_withbody(2,i) - 0.25 * (L - 0.8*v);
    xf_g_withbody(3,i) = xf_g_withbody(3,i) + 0.25 * (L - 0.8*v);
    xf_g_withbody(4,i) = xf_g_withbody(4,i) - 0.5 * (L - 0.8*v);
end

zf_g(1,301:400)=zf_g_Tt(1,:);
zf_g(2,101:200)=zf_g_Tt(2,:);
zf_g(3,201:300)=zf_g_Tt(3,:);
zf_g(4,1:100)=zf_g_Tt(4,:);


%Let's find velocities:
for i=1:size(tt,2)
    if i == 1 
        xvelf_b_Tt(1:4, i) = 0;
        xvelf_g_Tt(1:4, i) = 0;
        zvelf_b_Tt(1:4, i) = -v;
        zvelf_g_Tt(1:4, i) = 0;
    else
        xvelf_b_Tt(1:4, i) = ((xf_g_Tt(1:4, i) - xf_g_Tt(1:4, i-1))/0.01) - v;
        xvelf_g_Tt(1:4, i) = ((xf_g_Tt(1:4, i) - xf_g_Tt(1:4, i-1))/0.01);
        zvelf_b_Tt(1:4, i) = ((zf_g_Tt(1:4, i) - zf_g_Tt(1:4, i-1))/0.01);
        zvelf_g_Tt(1:4, i) = zvelf_b_Tt(1:4, i);
    end
end

% Put velocities into total cycle
xvelf_g = zeros(4,400);
zvelf_g = zeros(4,400);

xvelf_g(1,301:400)=xvelf_g_Tt(1,:);
xvelf_g(2,101:200)=xvelf_g_Tt(2,:);
xvelf_g(3,201:300)=xvelf_g_Tt(3,:);
xvelf_g(4,1:100)=xvelf_g_Tt(4,:);

zvelf_g(1,301:400)=zvelf_g_Tt(1,:);
zvelf_g(2,101:200)=zvelf_g_Tt(2,:);
zvelf_g(3,201:300)=zvelf_g_Tt(3,:);
zvelf_g(4,1:100)=zvelf_g_Tt(4,:);

xvelf_b = zeros(4,200);
zvelf_b = zeros(4,200);

xvelf_b(1,301:400)=xvelf_b_Tt(1,:);
xvelf_b(2,101:200)=xvelf_b_Tt(2,:);
xvelf_b(3,201:300)=xvelf_b_Tt(3,:);
xvelf_b(4,1:100)=xvelf_b_Tt(4,:);

zvelf_b(1,301:400)=zvelf_b_Tt(1,:);
zvelf_b(2,101:200)=zvelf_b_Tt(2,:);
zvelf_b(3,201:300)=zvelf_b_Tt(3,:);
zvelf_b(4,1:100)=zvelf_b_Tt(4,:);

%Let's find more positions:
total_x = 0;
total_z = 0;
for i=1:size(tt,2)
    total_x = total_x + xvelf_b_Tt(1:4,i);
    xf_b_Tt(1:4,i) = total_x * 0.01;
    total_z = total_z + zvelf_b_Tt(1:4,i);
    zf_b_Tt(1:4,i) = total_z * 0.01;
end

% Put positions into total cycle
xf_b = zeros(4,200);
zf_b = zeros(4,200);

xf_b(1,301:400)=xf_b_Tt(1,:);
xf_b(2,101:200)=xf_b_Tt(2,:);
xf_b(3,201:300)=xf_b_Tt(3,:);
xf_b(4,1:100)=xf_b_Tt(4,:);

zf_b(1,301:400)=zf_b_Tt(1,:);
zf_b(2,101:200)=zf_b_Tt(2,:);
zf_b(3,201:300)=zf_b_Tt(3,:);
zf_b(4,1:100)=zf_b_Tt(4,:);
%% ----------------------- *** PLOT  DATA *** ---------------------%%
% Part b plots
% for leg = 1:4
%     figure
%     tiledlayout(2,4);
% 
%     nexttile
%     plot(ct,xf_g_withbody(leg,:))
%     xlabel('Cycle Time')
%     ylabel(sprintf('Leg %g x pos f/g', leg))
% 
%     nexttile
%     plot(ct,zf_g(leg,:))
%     xlabel('Cycle Time')
%     ylabel(sprintf('Leg %g z pos f/g', leg))
% 
%     nexttile
%     plot(xf_g(leg,:),zf_g(leg,:))
%     xlabel(sprintf('Leg %g x pos f/g', leg))
%     ylabel(sprintf('Leg %g z pos f/g', leg))
% 
%     nexttile
%     plot(xf_b(leg,:),zf_b(leg,:))
%     xlabel(sprintf('Leg %g x pos f/b', leg))
%     ylabel(sprintf('Leg %g z pos f/b', leg))
% 
%     nexttile
%     plot(ct,xvelf_g(leg,:))
%     xlabel('Cycle Time')
%     ylabel(sprintf('Leg %g x vel f/g', leg))
% 
%     nexttile
%     plot(ct,zvelf_g(leg,:))
%     xlabel('Cycle Time')
%     ylabel(sprintf('Leg %g z vel f/g', leg))
% 
%     nexttile
%     plot(ct,xvelf_b(leg,:))
%     xlabel('Cycle Time')
%     ylabel(sprintf('Leg %g x vel f/b', leg))
% 
%     nexttile
%     plot(ct,zvelf_b(leg,:))
%     xlabel('Cycle Time')
%     ylabel(sprintf('Leg %g z vel f/b', leg))
% end

%% ---------------------------INVERSE KINEMATICS------------------------%%
% for i=1:4 % for all 4 legs
%     for j=1:size(ct,2) % time
% 
%         % Find invisible straight leg (hip to foot)
%         L(i,j) = sqrt(h^2 + xf_g_withbody(i,j)^2);
% 
%         % Find angle between straight leg and vertical
%         A(i,j) = atan2(xf_g_withbody(i,j), h);
%         A_deg(i,j) = A(i,j) * 180/pi;
% 
%         % Use LoC to find angles within l1-l2-straight_leg triangle
%         B(i,j) = acos((straight_leg(i,j)^2 + l1^2 -l2^2)/(2 * straight_leg(i,j) * l1)); % isos angles
%         C(i,j) = acos((l1^2 + l2^2 -straight_leg(i,j)^2)/(2 * l1 * l2)); % obtuse angle
%         B_deg(i,j) = B(i,j) * 180/pi;
%         C_deg(i,j) = C(i,j) * 180/pi;
% 
%         % Calculate joint angles (0 direction is facing horizontally for
%         % both legs)
%         theta1_rad(i,j) = -(pi/2) + A(i,j) + B(i,j);
%         theta2_rad(i,j) = -(pi - C(i,j));
% 
%         theta1_deg(i,j) = theta1_rad(i,j) * 180/pi;
%         theta2_deg(i,j) = theta2_rad(i,j) * 180/pi;
%     end
% end

for i=1:4 % for all 4 legs
    for j=1:size(ct,2) % time

        xpos = xf_b(i,j);
        zpos = zf_b(i,j);

        xfoot = xpos - 0.2;
        zfoot = zpos - h;
        
        alpha(i,j) = atan2(zfoot, xfoot);
        L = norm([xfoot, zfoot]);
        rho(i,j) = acos((2*l1^2-L^2)/(2*l1^2));
        phi(i,j) = pi/2 - rho(i,j)/2;
        gamma(i,j) = pi - rho(i,j);
        beta(i,j) = alpha(i,j) - phi(i,j);

        beta_deg(i,j) = beta(i,j) * 180/pi;
        gamma_deg(i,j) = gamma(i,j) * 180/pi;
    end
end

%% Part C Plots
for leg = 1:4
    figure
    tiledlayout(2,3);

    nexttile
    plot(ct,beta(leg,:))
    xlabel('Cycle Time')
    ylabel(sprintf('Beta %g', leg))
    figure
    nexttile
    plot(ct,gamma(leg,:))
    xlabel('Cycle Time')
    ylabel(sprintf('Gamma %g', leg))

end
%% Joint Velocities
%Let's find joint velocities:

t1_angvel = zeros(1,size(ct,2));
t2_angvel = zeros(1,size(ct,2));

for j = 1:1 %each leg
    for i=1:size(ct,2)
        if i == 1
            t1_angvel(j, i) = ((beta(j, i) - beta(j, size(ct,2)))/0.01);
            t2_angvel(j, i) = ((gamma(j, i) - gamma(j, size(ct,2)))/0.01);
        else
            t1_angvel(j, i) = ((beta(j, i) - beta(j, i-1))/0.01);
            t2_angvel(j, i) = ((gamma(j, i) - gamma(j, i-1))/0.01);
        end
    end
end

%% Part D Plots
figure
tiledlayout(2,1);
for leg = 1:1
    nexttile
    plot(ct,t1_angvel(leg,:))
    xlabel('Cycle Time')
    ylabel(sprintf('Theta 1 Ang Vel, Leg %g', leg))

    figure
    nexttile
    plot(ct,t2_angvel(leg,:))
    xlabel('Cycle Time')
    ylabel(sprintf('Theta 2 Ang Vel, Leg %g', leg))
end
%% Part E Simulation of leg
figure
hip = [l1*cos(beta(1,i)), l1*sin(beta(1,i))];
knee = [l2*cos(beta(1,i) + gamma(1,i)), l2*sin(beta(1,i) + gamma(1,i))];
link1 = quiver(0, 0, hip(1), hip(2), 'off', '-k', 'ShowArrowHead', 'off');
hold on
link2 = quiver(hip(1), hip(2), knee(1), knee(2), 'off', '-k', 'ShowArrowHead', 'off');

for i=1:size(ct,2)
    hip = [l1*cos(beta(1,i)), l1*sin(beta(1,i))];
    knee = [l2*cos(beta(1,i) + gamma(1,i)), l2*sin(beta(1,i) + gamma(1,i))];

    link1.UData = hip(1);
    link1.VData = hip(2);
    hold on
    link2.UData = knee(1);
    link2.VData = knee(2);
    link2.XData = hip(1);
    link2.YData = hip(2);
    axis([-1 1 -h h])
    pause(0.01)
end


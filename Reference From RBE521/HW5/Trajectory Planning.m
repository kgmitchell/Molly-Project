clear all;
clc;
%% ----------------------- *** INITIAL DATA *** ------------------------%%
l1=.020;
l2=.050;
l3=.100;
b=0.75;
v=.03; % desired velocity of COG (m/sec)
u_b=(b/(1-b))*v; % average swing velocity of the leg wrt the body
u=v/(1-b); % average swing velocity of the leg wrt the ground
L=.03; % Stride Length which is the amount of COG displacement in one cycle time.
T= L/v; % Cycle Period
Tt=(1-b)*T;
Ts=b*T;
%% ------------------- *** GAIT GENERATION *** ---------------------- %%
% b=.9
p(1)=0; % Kinematic phase of leg 1
p(2)=p(1)+1/2; % Kinematic phase of leg 2
p(3)=p(1)+b; % Kinematic phase of leg 3
p(4)=p(2)+b; % Kinematic phase of leg 4
p(5)=p(3)+b; % Kinematic phase of leg 5
p(6)=p(4)+b; % Kinematic phase of leg 6

%What if p(i) is equal or greater than 1?

    for i=1:6
        if p(i) >= 1
            p(i) = p(i) - 1;
        end
    end
p


%% ------------- *** POSITIONAL FOOT TRAJECTORY PLANNING *** ------------%%
%Transferring time is divided to 5 equal extents. Such a selection is completely
%arbitrary.
t0=0;
t1=Tt/5;
t2=2*t1;
t3=Tt-t2;
t4=Tt-t1;
t5=Tt;

Ttt=[t0,t1,t2,t3,t4,t5];

%%%%%%%%%%HERE EVERY THING IS IN Gi COORDINATE SYSTEM WHICH IS GROUND
%COORDINATE SYSTEM FOR EACH LEG
xdotmaxf_g= 0.3; %from the slides %maximum leg transferring speed.

zdotmaxf_g=xdotmaxf_g; % it is arbitrary.

xdotf_g=[0,0,xdotmaxf_g,xdotmaxf_g,0,0];

%Let's find horizonatal positions:
for i=1:6
    xf_g(i,1)=-L/2;
    xf_g(i,2)=xf_g(i,1) + 0;
    xf_g(i,3)=xf_g(i,2) + (t2-t1) * xdotmaxf_g / 2 ;
    xf_g(i,4)=xf_g(i,3) + (t3-t2) * xdotmaxf_g;
    xf_g(i,5)=xf_g(i,4) + (t4-t3) * xdotmaxf_g / 2 ;
    xf_g(i,6)=xf_g(i,5) + 0;
end


zdotf_g=[0,zdotmaxf_g,0,0,-zdotmaxf_g,0];

%Similarly for the vertical positions:
for i=1:6
    zf_g(i,1)=0;
    zf_g(i,2)=zf_g(i,1)+(t1-t0)*zdotmaxf_g/2;
    zf_g(i,3)=zf_g(i,2)+(t2-t1)*zdotmaxf_g/2;
    zf_g(i,4)=zf_g(i,3)+0;
    zf_g(i,5)=zf_g(i,4)-(t4-t3)*zdotmaxf_g/2;
    zf_g(i,6)=zf_g(i,5)-(t5-t4)*zdotmaxf_g/2;
end

%%%%%%%% Taking care of cooridinte systems

h=.1; %(m) height of the robot. Here h=l3 because it is assumed in the home positon.
D=l1+l2; %(m) Distance between the foot tip and hip joint from top view

alphaH=[-30*pi/180, 30*pi/180, -90*pi/180, 90*pi/180, -150*pi/180, 150*pi/180];

xb_g(1,1)= - (1-b) * L / 2 - D * cos(alphaH(1)); % frame b is a frame attached to the hip joint
xb_g(2,1)= - (1-b) * L / 2 - D * cos(alphaH(2));
xb_g(3,1)= - (1-b) * L / 2;
xb_g(4,1)= - (1-b) * L / 2;
xb_g(5,1)= - (1-b) * L / 2 - D * cos(alphaH(5))
xb_g(6,1)= - (1-b) * L / 2 - D * cos(alphaH(6))


% x axis is parallel to the x axis of frame g (Gi).
zb_g(1,1)=h;
zb_g(2,1)=h;
zb_g(3,1)=h;
zb_g(4,1)=h;
zb_g(5,1)=h;
zb_g(6,1)=h;



dt=Tt/5;
xdotb_g=v;
zdotb_g=0;

for i=1:6
    for t=1:5 
        xb_g(i,t+1)=xb_g(i,t)+xdotb_g*dt;
        zb_g(i,t+1)=zb_g(i,t)+zdotb_g*dt;
    end
    for t=1:6
        xf_b(i,t)=xf_g(i,t)-xb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
    end
end

% yf_b

yf_b=[D*sin(30*pi/180), -D*sin(30*pi/180), D,  -D,  D*sin(30*pi/180),  -D*sin(30*pi/180)];

for i=1:6 % this is becaus of 6 legs

    for j=1:6 % time
    
        xf_H(i,j)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        yf_H(i,j)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        zf_H(i,j)=zf_b(i,j);
    end
end
  xf_H
  yf_H
  zf_H
% xf_b
% yf_b
% zf_b
xb_g
%% ----------------------- *** PLOT  DATA *** ---------------------%%
subplot(3,2,1)
plot(Ttt,xdotf_g)
subplot(3,2,2)
plot(Ttt,zdotf_g)

subplot(3,2,3)
plot(Ttt,xf_g(4,:))
subplot(3,2,4)
plot(Ttt,zf_g(4,:))

subplot(3,2,5)
plot(xf_g(4,:),zf_g(4,:))

subplot(3,2,6)
plot(xf_b(4,:),zf_b(4,:))

 
%% ---------------------------INVERSE KINEMATICS------------------------%%


for i=1:6 % for all 6 legs
    for j=1:6 % time
        Alpha(i,j)=(atan(yf_H(i,j)/xf_H(i,j)));
        l(i,j)=sqrt(yf_H(i,j)^2+xf_H(i,j)^2);
        d(i,j)=sqrt(zf_H(i,j)^2+(l(i,j)-l1)^2);
        Beta(i,j)=acos((l2^2+d(i,j)^2-l3^2)/(2*l2*d(i,j)))-atan(abs(zf_H(i,j))/(l(i,j)-l1));
        Gamma(i,j)=pi-(acos((l2^2+l3^2-d(i,j)^2)/(2*l2*l3)));

    end
end
A=Alpha*180/pi
B=Beta*180/pi
G=Gamma*180/pi
Ttt

%% -------------- *** VELOCITY FOOT TRAJECTORY PLANNING *** -------------%%
ydotf_b=0;
for t=1:6
    xdotf_b(t)=xdotf_g(t)-xdotb_g;
    zdotf_b(t)=zdotf_g(t)-zdotb_g;
end
for i=1:6
    for j=1:6
        xdotf_H(i,j)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xdotf_b(j);ydotf_b;zdotf_b(j)];
        ydotf_H(i,j)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xdotf_b(j);ydotf_b;zdotf_b(j)];
        zdotf_H(i,j)=zdotf_b(j);
    end
end
for j=1:6 % time
    for i=1:6 % leg numnber
        theta1=Alpha(i,j);
        theta2=Beta(i,j);
        theta3=Gamma(i,j);
        J(1,1)=-(-sin(theta1)*sin(theta2)*cos(theta3)-sin(theta1)*cos(theta2)*sin(theta3))*l3-sin(theta1)*l2*cos(theta2)-l1*sin(theta1);
        J(1,2)=-(-cos(theta1)*sin(theta2)*sin(theta3)+cos(theta1)*cos(theta2)*cos(theta3))*l3-cos(theta1)*l2*sin(theta2);
        J(1,3)=(cos(theta1)*sin(theta2)*sin(theta3)-cos(theta1)*cos(theta2)*cos(theta3))*l3;
        J(2,1)=-(cos(theta1)*cos(theta2)*sin(theta3)+cos(theta1)*sin(theta2)*cos(theta3))*l3+cos(theta1)*l2*cos(theta2)+l1*cos(theta1);
        J(2,2)=-(-sin(theta1)*sin(theta2)*sin(theta3)+sin(theta1)*cos(theta2)*cos(theta3))*l3-sin(theta1)*l2*sin(theta2);
        J(2,3)=-(-sin(theta1)*sin(theta2)*sin(theta3)+sin(theta1)*cos(theta2)*cos(theta3))*l3;
        J(3,1)=0;
        J(3,2)=-(-cos(theta2)*sin(theta3)-sin(theta2)*cos(theta3))*l3-l2*cos(theta2);
        J(3,3)=-(-cos(theta2)*sin(theta3)-sin(theta2)*cos(theta3))*l3;
        Thetadot(i,j,:)=inv(J)*[xdotf_H(i,j);ydotf_H(i,j);zdotf_H(i,j)];
    end
end

Thetadot

%% -------------------- *** The End *** -------------------------- %%
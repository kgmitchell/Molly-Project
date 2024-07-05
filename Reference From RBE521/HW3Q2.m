clear all
clc
z=800;
nominal_S_X=[92.1597 , 27.055 , -119.2146 , -119.2146 , 27.055 , 92.1597 ]' ;
nominal_S_Y=[84.4488 , 122.037 , 37.5882 , -37.5882 , -122.037 , -84.4488 ]' ;
nominal_S_Z=[0 ,0 ,0 ,0 ,0 ,0 ]' ;

nominal_U_X=[305.4001 , -56.4357 , -248.9644 , -248.9644 , -56.4357 , 305.4001 ]' ;
nominal_U_Y=[111.1565 , 320.0625 , 208.9060 , -208.9060 , -320.0625 , -111.1565 ]' ;
nominal_U_Z=[0 ,0 ,0 ,0 ,0 ,0 ]' ;

nominal_intital_L=[604.8652 ,604.8652 ,604.8652 ,604.8652 ,604.8652 ,604.8652 ]' ;

simulated_real_S_X=[96.6610 , 22.2476 , -122.4519 , -120.6859 , 24.7769 , 91.3462 ]' ;
simulated_real_S_Y=[81.7602 , 125.2511 , 36.6453 , -34.4565 , -125.0489 , -80.9866 ]' ;
simulated_real_S_Z=[1.0684 ,-0.5530 ,4.3547 ,-4.9014 ,-4.8473 ,0.2515 ]' ;

simulated_real_U_X=[305.2599 , -55.2814 , -244.7954 , -252.5755 , -53.9678 , 302.4266 ]' ;
simulated_real_U_Y=[115.0695 , 322.9814 , 208.0087 , -211.8783 , -320.6115 , -109.4351 ]' ;
simulated_real_U_Z=[2.6210 ,4.2181 ,3.9365 ,-3.0128 ,4.3181 ,3.3812 ]' ;

simulated_intital_L=[604.4299 ,607.2473 ,600.4441 ,605.9031 ,604.5251 ,604.0616]' ;

Error_in_S_X=-(nominal_S_X-simulated_real_S_X);
Error_in_S_Y=-(nominal_S_Y-simulated_real_S_Y);
Error_in_S_Z=-(nominal_S_Z-simulated_real_S_Z);

Error_in_U_X=-(nominal_U_X-simulated_real_U_X);
Error_in_U_Y=-(nominal_U_Y-simulated_real_U_Y);
Error_in_U_Z=-(nominal_U_Z-simulated_real_U_Z);

Error_in_L=-(nominal_intital_L-simulated_intital_L);

 for i=1:1:6
     delta(:,i)=[ Error_in_S_X(i) ; Error_in_S_Y(i) ; Error_in_S_Z(i) ; Error_in_U_X(i) ;Error_in_U_Y(i) ; Error_in_U_Z(i) ; Error_in_L(i) ] ;
 end
 Total_Delta=[delta(:,1) ; delta(:,2) ; delta(:,3) ; delta(:,4) ; delta(:,5) ; delta(:,6) ];
 Square_root_of_Error=zeros(1,1000);
x_value=zeros(1,1000);
y_value=zeros(1,1000);
i=1;
zeros_vector=zeros(1,7);

lmin = 604.8652;
lmax = 1100;
z = 800;
center1 = [214.42; 26.71; 0];
center2 = [-81.87; 198.03; 0];
center3 = [-128.19; 171.31; 0];
center4 = [-129.08; -171.32; 0];
center5 = [-82.89; -198.03; 0];
center6 = [214.42; -26.71; 0];

for x=-700:5:700
    for y=-700:5:700
        p=[x , y , 800 ,0 , 0 ,0 ]';

        % spheres equations from paper 1
        RightUS3 = (x - center6(1))^2 + (y - center6(2))^2 + (z - center6(3))^2 - lmax^2;
        RightUS1 = (x - center4(1))^2 + (y - center4(2))^2 + (z - center4(3))^2 - lmax^2;
        RightUS2 = (x - center5(1))^2 + (y - center5(2))^2 + (z - center5(3))^2 - lmax^2;

        LeftUS3 = (x - center3(1))^2 + (y - center3(2))^2 + (z - center3(3))^2 - lmax^2;
        LeftUS1 = (x - center1(1))^2 + (y - center1(2))^2 + (z - center1(3))^2 - lmax^2;
        LeftUS2 = (x - center2(1))^2 + (y - center2(2))^2 + (z - center2(3))^2 - lmax^2;
    
        RightLS3 = (x - center3(1))^2 + (y - center3(2))^2 + (z - center3(3))^2 - lmin^2;
        RightLS1 = (x - center1(1))^2 + (y - center1(2))^2 + (z - center1(3))^2 - lmin^2;
        RightLS2 = (x - center2(1))^2 + (y - center2(2))^2 + (z - center2(3))^2 - lmin^2;

        LeftLS3 = (x - center6(1))^2 + (y - center6(2))^2 + (z - center6(3))^2 - lmin^2;
        LeftLS1 = (x - center4(1))^2 + (y - center4(2))^2 + (z - center4(3))^2 - lmin^2;
        LeftLS2 = (x - center5(1))^2 + (y - center5(2))^2 + (z - center5(3))^2 - lmin^2;



        if (RightUS3 <= 0 && RightUS1 <= 0 && RightUS2 <= 0 && LeftUS3 <= 0 && LeftUS1 <= 0 && LeftUS2 <= 0 ...
                && RightLS3 >= 0 && RightLS1 >= 0 && RightLS2 >= 0 && LeftLS3 >= 0 && LeftLS1 >= 0 && LeftLS2 >= 0)


        [L,n,s,R ]= IK(p);
        Velocity_Jacobian = [n(:,1)' , cross(R * s(:,1), n(:,1))';
        n(:,2)' , cross(R * s(:,2), n(:,2))';
        n(:,3)' , cross(R * s(:,3), n(:,3))';
        n(:,4)' , cross(R * s(:,4), n(:,4))';
        n(:,5)' , cross(R * s(:,5), n(:,5))';
        n(:,6)' , cross(R * s(:,6), n(:,6))'];
    
        Identification_matrix= [ n(:,1)'  -n(:,1)'  -1  , zeros_vector , zeros_vector , zeros_vector, zeros_vector, zeros_vector ;
            zeros_vector , n(:,2)' , -n(:,2)'  , -1 , zeros_vector , zeros_vector , zeros_vector , zeros_vector ;
            
           zeros_vector , zeros_vector ,  n(:,3)' , -n(:,3)'  , -1  , zeros_vector , zeros_vector , zeros_vector ;
           zeros_vector , zeros_vector , zeros_vector , n(:,4)' , -n(:,4)'  , -1  , zeros_vector , zeros_vector ;
           zeros_vector, zeros_vector , zeros_vector , zeros_vector , n(:,5)' , -n(:,5)'  , -1 , zeros_vector  ;
           zeros_vector, zeros_vector, zeros_vector, zeros_vector, zeros_vector,  n(:,6)' , -n(:,6)'  , -1 ; ];

          Error_in_pose=pinv(Velocity_Jacobian)*(-Identification_matrix *  Total_Delta);

          Square_root_of_Error(i)=[sqrt(Error_in_pose(1)^2 +Error_in_pose(2)^2 +Error_in_pose(3)^2 )];
          
          x_value(i)=[x];
          y_value(i)=[y];
          i=i+1;  
        end

        
        
    end 
end

size_of_identification=size(Identification_matrix);
size_of_Jacobian=size(Velocity_Jacobian);
 x_value;
 y_value;
 Square_root_of_Error;
d1=size(x_value);
d2=size(y_value);
d3=size(Square_root_of_Error);

% xB = reshape(x_value,[4,3]);
% yB = reshape(y_value,[4,3]);
% zB = reshape(Square_root_of_Error,[4,3]);
% figure
% surf(xB,yB,zB)
% [X,Y,Z] = meshgrid(min(x_value):10:max(x_value) ,min(y_value):10:max(y_value) , min(Square_root_of_Error):10:max(Square_root_of_Error));
% surf(X,Y,Z)
 

plot3(x_value, y_value,Square_root_of_Error);

xlabel('x position')
ylabel('y position')
zlabel('position error (mm)') 


%Inverse Kinematics of Parallel Robots with Prismatic Legs

function [L,n,s,R ]= IK(p)


%% Desired Pose (Given)
% P = [0 0 0 0 0 0]';


%% Robot Parameters (Given)
% Rm=300/2;
% Rf=500/2;
% alpha=40*pi/180;
% beta=80*pi/180;

%% Extracting Position and Euler Angle Information from the given desired pose

o=p(1:3,1); 
a=p(4)*pi/180; 
b=p(5)*pi/180; 
c=p(6)*pi/180;

%% Calculating Rotation Matrix from Euler Angles

% R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)]; % Rx,a
% R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)]; % Ry,b
% R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1]; % Rz,c
% 
% R = R1*R2*R3; % Rxyz

R=eye(3);
%% Calculating upper joint positions wrt. the upper coordinate frame

s1=[92.1597 , 84.4488 , 0]';
s2=[27.055  , 122.037, 0]';
s3=[-119.2146 , 37.5882 , 0]';
s4=[-119.2146  , -37.5882 , 0]';
s5=[27.055  , -122.037 , 0]';
s6=[92.1597  , -84.4488 , 0]';


s = [s1 , s2 , s3 , s4 , s5 , s6];

%% Calculating lower joint positions wrt. the lower corrdinate frame

u1=[305.4001  , 111.1565, 0]';
u2=[-56.4357, 320.0625, 0]';
u3=[-248.9644  , 208.9060 , 0]';
u4=[-248.9644, -208.9060 , 0]';
u5=[-56.4357  , -320.0625, 0]';
u6=[305.4001  , -111.1565 , 0]';

u = [u1 , u2 , u3 , u4 , u5 , u6];


%% Put it in the IK equation

for i = 1:6
    l (: , i) = o + R * s(: , i) - u (: , i);
    L(i) = norm (l (: , i),2);
    n(:,i)= l (: , i)/L(i);
   N(i)=norm(n(:,i),2); % I did it to check that n is a unit vector.
   
end
end
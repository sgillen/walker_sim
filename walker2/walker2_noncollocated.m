function [dX, TAU] = walker2_noncollocated(t,X) 
%TODO this is can be made more efficient (probably not worthwhile though) 

%dynamic equations of motion for a slightly different three link model than
%before

%draws heavily from acrobot_eom.m
%sean gillen 8/29/17

%mass and geometry properites
m1 = 1; m2 = 1; m3 = 1; %mass
l1 = 2; l2=2; l3=2; %lengths of our three links
lc1 = l1/2; lc2 = l2/2; lc3 = l3/2; % location of the center of mass for each of the three links
g=9.8; %gravity
J1 = 3; J2 = 3; J3 = 3; %moments of inertia  
b2 = 5; b3 = 5; %motor damping terms

th1  = X(1);
th2  = X(2);
th3  = X(3);

dth1 = X(4);
dth2 = X(5);
dth3 = X(6);


%define our Torques 
tau2 = 0;
tau3 = 0;

TAU = [tau2; tau3];

% D is our mass matrix
% C is the coriolis matrix
% G is the gravity matrix

%all were derived in walker2_eom_derivation


d11 = J1 + l1^2*m2 + l1^2*m3 + lc1^2*m1;
d12 = l1*lc2*m2*cos(th1 - th2);
d13 = l1*lc3*m3*cos(th1 - th3);
d21 = l1*lc2*m2*cos(th1 - th2);
d22 = J2 + lc2^2*m2;
d23 = 0;
d31 = l1*lc3*m3*cos(th1 - th3);
d32 = 0;
d33 = lc3^2*m3;

D = [d11 d12 d13;d21 d22 d23; d31 d32 d33];

c11 = 0;
c12 = dth2*l1*lc2*m2*sin(th1 - th2);
c13 = dth3*l1*lc3*m3*sin(th1 - th3);
c21 = -dth1*l1*lc2*m2*sin(th1 - th2);
c22 = 0;
c23 = 0;
c31 = -dth1*l1*lc3*m3*sin(th1 - th3) ;
c32 = 0;
c33 = 0;

%throw non conservative forces here (modeling back emf which is just a
%constant times tdth so I throw it in with the other terms being multipled
%by dth).

c12 = c12 - b2;
c13 = c13 - b3;
c22 = c22 + b2;
c33 = c33 + b3;


C = [c11 c12 c13;c21 c22 c23; c31 c32 c33];

g1 = g*lc1*m1*cos(th1);
g2 = g*lc2*m2*cos(th2);
g3 = g*lc3*m3*cos(th3);

G = [g1;g2;g3];

% %derived in waker2_eom_derivation
% d2th3= -(- l1*lc3*m3*sin(th1 - th3)*dth1^2 - tau3 + b3*dth3 + g*lc3*m3*cos(th3) + d2th1*l1*lc3*m3*cos(th1 - th3))/(m3*lc3^2 + J3);
% d2th2 = -(- l1*lc2*m2*sin(th1 - th2)*dth1^2 - tau2 + b2*dth2 + g*lc2*m2*cos(th2) + d2th1*l1*lc2*m2*cos(th1 - th2))/(m2*lc2^2 + J2);
% d2th1 = -(l1*lc2*m2*sin(th1 - th2)*dth2^2 - b2*dth2 + l1*lc3*m3*sin(th1 - th3)*dth3^2 - b3*dth3 + tau2 + tau3 + g*lc1*m1*cos(th1) + d2th2*l1*lc2*m2*cos(th1 - th2) + d2th3*l1*lc3*m3*cos(th1 - th3))/(J1 + l1^2*m2 + l1^2*m3 + lc1^2*m1);

B = [-1 -1; 1 0; 0 1];

kp = 5;
kd = .5;
qdes = pi/2;

TAU(2) = 0;
TAU(1) = d22.^(-1).*(d12+d33).^(-1).*((-1).*d22.*d33.*g1+d12.*d33.*g2+d12.*d22.*g3+(-1).*d12.*d22.*TAU(2)+d12.*d33.*TAU(2)+c31.*d12.*d22.*(dth1))+c21.*d12.*d33.*(dth1)+d12.*d22.*d13.*(kp.*(qdes+(-1).*th1)+(-1).*kd.*(dth1))+d12.*d12.*d33.*(kp.*(qdes+(-1).*th1)+(-1).*kd.*(dth1))+(-1).*d11.*d22.*d33.*(kp.*(qdes+(-1).*th1)+(-1).*kd.*(dth1))+(-1).*c12.*d22.*d33.*(dth2)+(-1).*c13.*d22.*d33.*(dth3);

torque_limit = 10000;  % [Nm] limit in torque magnitude
TAU(1) = sign(TAU(1))*min(abs(TAU(1)),torque_limit);

d2q = D^-1*(B*TAU - C*[dth1;dth2;dth3] - G);
%d2q(3) = 0;
dX = [dth1;dth2;dth3;d2q];
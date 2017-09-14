function [dX, TAU] = walker_noncollocated(t,X) 
%TODO this is can be made more efficient (probably not worthwhile though) 

%dynamic equations of motion for a three link walker described here: https://www.ijsr.net/archive/v2i5/IJSRON2013995.pdf
%draws heavily from acrobot_eom.m
%sean gillen 8/29/17

%mass and geometry properites(mass, lengths, moments of inertia, and
%gravity)

m = 1; mh = 3; mt = 3; l = 2; r=2; g=9.8;

q = [0;0;0];
q(1)  = X(1);
q(2)  = X(2);
q(3)  = X(3);

dq = [0;0;0];
dq(1) = X(4);
dq(2) = X(5);
dq(3) = X(6);

theta(3) = q(3);
theta(2) = -q(2); %q(3) + q(2); %I'm not entirely sure this works
theta(1) = q(1); %(q(3) + q(1));%q(3) - q(1); 

%define our Torques 
tau1 = 0;
tau2 = 0;

TAU = [tau1; tau2];


% our robot eqution is D(q)d2q + C(q,dq)dq + G(q) = Bu
% q is our set of Generalized coordinates
% D is the inetia matrix
% C is the corialis matrix
% G is gravity
% B ins the input matrix

%Inertia Matrix (cheated to get this TODO derive equations of motion by
%hand D
m11 = (5/4*m + mh + mt)*r^2;
m12 = (-1/2*m*r^2*cos(theta(1) - theta(2)));
m13 = mt*r*l*cos(theta(1) - theta(3));
m22 = 1/4*m*r^2;
m23 = 0;
m33 = mt*l^2;

D = [m11, m12, m13; m12, m22, m23; m13, m23, m33];

%Coriolis Matrix C
c12 = -1/2*m*r^2*sin(theta(1) - theta(2))*-dq(2);
c13 = mt*r*l*sin(theta(1) - theta(3))*dq(3);
c21 = 1/2*m*r^2*sin(theta(1) - theta(2))*dq(1);
c31 = -mt*r*l*sin(theta(1) - theta(3))*dq(1);

C = [0, c12, c13; c21, 0, 0; c31, 0, 0];

% Gravity matrix G

g1 = -1/2*g*(2*mh + 3*m + 2*mt)*r*sin(theta(1));
g2 = 1/2*g*m*r^2*sin(theta(2));
g3 = -g*mt*l*sin(theta(3));

G = [g1;g2;g3];

%B the input matrix
B = [-1, 0; 0, -1; 1,1];

%TAU(2) = 150*(pi - q(2));
%TAU(3) = 150*(pi - q(3)); 
    
kp = 50;
kd = 5;
qdes = 0; 

%derived in mathematica, notebook: walker_linearization.nb
TAU(2) = 0;
TAU(1) = m22.^(-1).*(m12+m33).^(-1).*((-1).*m22.*m33.*g1+m12.*m33.*g2+m12.*m22.*g3+(-1).*m12.*m22.*TAU(2)+m12.*m33.*TAU(2)+c31.*m12.*m22.*(dq(1))+c21.*m12.*m33.*(dq(1))+m12.*m22.*m13.*(kp.*(qdes+(-1).*q(1))+(-1).*kd.*(dq(1)))+m12.*m12.*m33.*(kp.*(qdes+(-1).*q(1))+(-1).*kd.*(dq(1)))+(-1).*m11.*m22.*m33.*(kp.*(qdes+(-1).*q(1))+(-1).*kd.*(dq(1)))+(-1).*c12.*m22.*m33.*(dq(2))+(-1).*c13.*m22.*m33.*(dq(3)));

torque_limit = 10000;  % [Nm] limit in torque magnitude
TAU(1) = sign(TAU(1))*min(abs(TAU(1)),torque_limit);

d2q = D^-1*(B*TAU - C*dq - G);
dX = [dq(1);dq(2);dq(3);d2q(1);d2q(2);d2q(3)];


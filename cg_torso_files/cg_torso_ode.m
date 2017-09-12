function [dX,u] = cg_torso_ode(t,X);

%% Look up parameters from a separate (single) file 
P = cg_torso_params;
J1=P.J1; J2=P.J2; J3=P.J3;
L1=P.L1; L1c=P.L1c; L3c=P.L3c;
m1=P.m1; m2=P.m2; m3=P.m3;
g = P.g; % gravity


% Cheat below, for open-loop u case:
%[dX,u] = cg_torso_ode_u(t,X);
%return;

th1 = X(1);
th2 = X(2);
th3 = X(3);
dth1 = X(4);
dth2 = X(5);
dth3 = X(6);


%% Inertia matrix (M) and conservative torque terms (C)
%sgillen - may be able to save some time by not computing non theta dependent values
%everyime, but probably not worthwhile.

M11 = J1 + J2 + J3 + L1^2*m1 + L1^2*m2 + L1^2*m3 + L1c^2*m1 + L1c^2*m2 + L3c^2*m3 - 2*L1*L1c*m1 + 2*L1*L1c*m2*cos(th2) + 2*L1*L3c*m3*cos(th3);
M12 = J2 + L1c^2*m2 + L1*L1c*m2*cos(th2);
M13 = J3 + L3c^2*m3 + L1*L3c*m3*cos(th3);
M21 = J2 + L1c^2*m2 + L1*L1c*m2*cos(th2);
M22 = J2 + L1c^2*m2;
M23 = 0;
M31 = J3 + L3c^2*m3 + L1*L3c*m3*cos(th3);
M32 = 0;
M33 = J3 + L3c^2*m3;
C1 = L1c*g*m2*cos(th1 + th2) + L3c*g*m3*cos(th1 + th3) + L1*g*m1*cos(th1) + L1*g*m2*cos(th1) + L1*g*m3*cos(th1) - L1c*g*m1*cos(th1) - L1*L1c*dth2^2*m2*sin(th2) - L1*L3c*dth3^2*m3*sin(th3) - 2*L1*L1c*dth1*dth2*m2*sin(th2) - 2*L1*L3c*dth1*dth3*m3*sin(th3);
C2 = L1c*g*m2*cos(th1 + th2) + L1*L1c*dth1^2*m2*sin(th2);
C3 = L3c*g*m3*cos(th1 + th3) + L1*L3c*dth1^2*m3*sin(th3);

M = [M11, M12, M13; M21, M22, M23; M31, M32, M33];
C = [C1; C2; C3];

% M*d2th + C = Xi, where Xi are the non-conservative torques, i.e.,
% Xi = [0; tau2; tau3].
% Let u = [tau2; tau3], and Xi = [0 0; 1 0; 0 1]*u = 
% So, dX = AX + Bu formulation yields B = [zeros(3,2); inv(M)*[0 0;1 0;0 1]

%% Control law: now implemented inside "cg_torso_controller.m"
% Simple case, with PD control, to start...
% th3_abs = th1+th3;
% dth3_abs = dth1+dth3;
% th2_rel = th2;
% dth2_rel = dth2;
% 
% ff = 1;
% Kp2=400*ff; Kd2=40*ff;
% Kp3=400*ff; Kd3=40*ff;
% 
% th3_ref = 60*pi/180; % absolute angle, wrt x axis, measured CCW
% th2_ref = (180+35)*pi/180;
% 
% u2 = Kp2*(th2_ref - th2_rel) + Kd2*(0 - dth2_rel);
% u3 = Kp3*(th3_ref - th3_abs) + Kd3*(0 - dth3_abs);
% u = [u2; u3];

u = cg_torso_controller(t,X);

umat = [0 0; 1 0; 0 1]; % Which EOMs does u affect?
d2th = M \ (-C + umat*u);
dth = X(4:6); % velocity states, in order to match positions...
dX = [dth; d2th];

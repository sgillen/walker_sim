function [dX,u] = cg_torso_ode_u(t,X);

persistent tuse uuse

if isempty(uuse)
    load cg_torso_uset_data tset uset xset
    tuse = tset;
    uuse = uset;
end

th1 = X(1);
th2 = X(2);
th3 = X(3);
dth1 = X(4);
dth2 = X(5);
dth3 = X(6);

%% Look up parameters from a separate (single) file
P = cg_torso_params;
J1=P.J1; J2=P.J2; J3=P.J3;
L1=P.L1; L1c=P.L1c; L3c=P.L3c;
m1=P.m1; m2=P.m2; m3=P.m3;
g = P.g; % gravity

%% Inertia matrix (M) and conservative torque terms (C)
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

%% Control law:

if t>tuse(end);
    %u = [0;0];
    u = uuse(end,:)';
else
    u2 = interp1(tuse,uuse(:,1),t,'spline');
    u3 = interp1(tuse,uuse(:,2),t,'spline');
    u = [u2; u3];
end
umat = [0 0; 1 0; 0 1];
d2th = M \ (-C + umat*u);
dth = X(4:6); % velocity states, in order to match positions...
dX = [dth; d2th];

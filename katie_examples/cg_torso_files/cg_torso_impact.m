function Xplus = cg_torso_impact(Xminus)
% Katie Byl, UCSB, 7/17/17.

P = cg_torso_params;
J1=P.J1; J2=P.J2; J3=P.J3;
L1=P.L1; L1c=P.L1c; L3c=P.L3c;
m1=P.m1; m2=P.m2; m3=P.m3;

% For derivation, see bottom part of "cg_torso_eom.m"...


th1 = Xminus(1);
th2 = Xminus(2);
th3 = Xminus(3);
dth_minus = Xminus(4:6); % pre-impact Angular Velocities
Qm = [ J1 + J2 + J3 + L1c^2*m1 + L1c^2*m2 + L3c^2*m3 - L1^2*m1*cos(th2) - L1^2*m2*cos(th2) - L1^2*m3*cos(th2) - L1*L1c*m1 - L1*L1c*m2 + L1*L1c*m1*cos(th2) + L1*L1c*m2*cos(th2) + L1*L3c*m3*cos(th3) - L1*L3c*m3*cos(th2 - th3), m2*L1c^2 - L1*m2*L1c + J2, m3*L3c^2 - L1*m3*cos(th2 - th3)*L3c + J3
    m1*L1c^2 - L1*m1*L1c + J1,                         0,                                        0
    m3*L3c^2 + L1*m3*cos(th3)*L3c + J3,                         0,                            m3*L3c^2 + J3];

% Now, update "meanings" of all angles, to match post-impact situation.
% THEN, evaluate Qplus (Qp):

% After impact,
% 1) th1p = th1m + th2m + pi  -> absolute angle from hip,
% and then "-pi" to take opposite angle, wrt new toe (instead of wrt
% hip joint)
% 2) th2p = (th1m+pi) - th1p = (th1m+pi) - th1m - th2m + pi = -th2m
% i.e., th2p = -th2m, since 2*pi = 0
% 3) th3p = (th1m+th3m) - th1p = th1m + th3m - th1m - th2m + pi
% th3p = th3m - th2m + pi

%Am = [1 1 0; 0 -1 0; 0 -1 1];
%Bm = [pi; 0; pi];  % [almost, but need 2*pi "tweaks" -- see below

% 1) th1p = (th1m+th2m) - pi
% 2) th2p = 2*pi - th2m
% 3) th3p = th1m + th2m + th3m - 2*pi
Am = [1 1 0; 0 -1 0; 0 -1 1];
Bm = [-pi; 2*pi; pi];

th_minus = [th1;th2;th3];
th_plus = Am*th_minus + Bm;
th1 = th_plus(1); th2 = th_plus(2); th3 = th_plus(3);



Qp = [ J1 + J2 + J3 + L1^2*m1 + L1^2*m2 + L1^2*m3 + L1c^2*m1 + L1c^2*m2 + L3c^2*m3 - 2*L1*L1c*m1 + 2*L1*L1c*m2*cos(th2) + 2*L1*L3c*m3*cos(th3), m2*L1c^2 + L1*m2*cos(th2)*L1c + J2, m3*L3c^2 + L1*m3*cos(th3)*L3c + J3
    m2*L1c^2 + L1*m2*cos(th2)*L1c + J2,                      m2*L1c^2 + J2,                                  0
    m3*L3c^2 + L1*m3*cos(th3)*L3c + J3,                                  0,                      m3*L3c^2 + J3];

dth_plus = Qp \ (Qm * dth_minus);
%dth_plus = (Qp \ Qm) * dth_minus


Xplus = [th_plus; dth_plus];




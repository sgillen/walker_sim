function P = cg_torso_params
%sgillen: called in cg_torso_ode

P.g = 9.81; % gravity

P.m1 = 10;
P.L1 = 1;
P.L1c = 0.5; % wrt hip joint (going down each leg)
P.J1 = P.m1*0.16; % m*r^2, approx guess

P.L3 = 0.8; % L3 is only for "animation" (not dynamics)
P.L3c = 0.4; % only "L3c" affect dynamics
P.m3 = 10;
P.J3 = P.m3*0.25; % to be played with...

P.m2 = P.m1;
P.J2 = P.J1;
%P.L2 = P.L1;
%P.L2c = P.L1c;

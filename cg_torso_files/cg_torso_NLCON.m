function [C,Ceq] = cg_torso_NLCON(X)

X2 = cg_torso_step(X);
Ceq = norm(X-X2);
C = 0;
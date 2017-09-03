% segway_eom.m
%
% "Segway-Style" Inverted Pendulum: Equations of Motion.
% (See notes from Lecture 13 for a description of the system.)
% Katie Byl, 2012.

clear all; format compact  % compact produces single-spaced output

% Define symbolic variables in matlab:
syms phiw thetab L mb Jb mw Jw Rw g b tau

% 1a. GC's (generalized coordinates), and their derivatives:
GC = [{phiw},{thetab}]; % Using ABSOLUTE angles here
dphiw = fulldiff(phiw,GC); % time derivative. GC are variables (over time)
dthetab = fulldiff(thetab,GC);

% 1b. Geometry of the masses/inertias, given GC's are freely changing...
xw = Rw*phiw;
xb = xw+L*sin(thetab);
yw = 0;
yb = L*cos(thetab);

% 1c. Define any required velocity terms (for masses):
dxw = fulldiff(xw,GC);
dxb = fulldiff(xb,GC);
dyb = fulldiff(yb,GC);

% 2. Kinetic Energy:
T = (1/2)*(mw*dxw^2 + Jw*dphiw^2 + mb*(dxb^2 + dyb^2) + Jb*dthetab^2)

% 3. Potential Energy:
V = mb*g*yb

% 4. Lagrangian:
L = T-V

% 5. EOMs:
eq1 = fulldiff(diff(L,dphiw),GC) - diff(L,phiw)
eq2 = fulldiff(diff(L,dthetab),GC) - diff(L,thetab);
eq2 = simplify(eq2)

% 6. Xi: non-conservative terms
Xi1 = tau - b*(dphiw-dthetab)  % Motor torque tau, and back emf damping b
Xi2 = -tau + b*(dphiw-dthetab)  % (equal and opposite to above)


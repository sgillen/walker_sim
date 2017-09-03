% derivation for EOMS for a slightly modifed walker model, mostly a
% tutorial in the symbolic toolbox/fulldiff I guess

%following along with segway_eom.m provided by katie

%assumptions - the center of mass is balanced along the WIDTH of each rod, we make no assumptions as to where it lies along the length 

syms J1 J2 J3 m1 m2 m3 th1 th2 th3 g tau2 tau3 l1 l2 l3 lc1 lc2 lc3 b2 b3 

GC = [{th1},{th2},{th3}];

dth1 = fulldiff(th1,GC);
dth2 = fulldiff(th2,GC);
dth3 = fulldiff(th3,GC);

xh = l1*cos(th1); %this is the location of the "hip"
x2 = xh + lc2*cos(th2);
x3 = xh + lc3*cos(th3);

yh = l1*sin(th1); %this is the location of the "hip"
y2 = yh + lc2*sin(th2);
y3 = yh + lc3*sin(th3);

dx2 = fulldiff(x2,GC);
dx3 = fulldiff(x3,GC);

dy2 = fulldiff(y2,GC);
dy3 = fulldiff(y3,GC);

%kinetic energy
T =   1/2*J1*dth1^2 + 1/2*m1*(dth1*lc1)^2    ... %contribution from link 1
    + 1/2*J2*dth2^2 + 1/2*m2*(dx2^2 + dy2^2) ... % link 2
    + 1/2*J3*dth3^2 + 1/2*m3*(dx3^2 + dy3^2);    % link 3

%potential energy 
V = -(m1*g*sin(th1)*lc1 + m2*g*sin(th2)*lc2 + m3*g*sin(th3)*lc3);

%lagrangian
L = T - V;

%EOMS
eq1 = simplify(fulldiff(diff(L,dth1),GC) - diff(L,th1))
eq2 = simplify(fulldiff(diff(L,dth2),GC) - diff(L,th2))
eq3 = simplify(fulldiff(diff(L,dth3),GC) - diff(L,th3))

%non conservtive forces.. (I THINK these were done correctl)
Xi1 = -(tau2 - b2*dth2) -(tau3 - b3*dth3);
Xi2 = tau2 - b2*dth2;
Xi3 = tau3 - b3*dth3; 

% you have to tell matlab these are symbols or else the next step will
% confuse it
% syms d2th1 d2th2 d2th3
% 
% solve(eq1 == Xi1, d2th1)
% solve(eq2 == Xi2, d2th2)
% solve(eq3 == Xi3, d2th3)




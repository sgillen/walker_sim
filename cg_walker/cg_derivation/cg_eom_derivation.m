% derivation for EOMS for a compass gait walker with rodlike legs

%assumptions - the center of mass is balanced along the WIDTH of each rod, we make no assumptions as to where it lies along the length 

syms J1 J2 m1 m2 th1 th2 g tau l1 l2 lc1 lc2 b2 b3 

GC = [{th1},{th2}];

dth1 = fulldiff(th1,GC);
dth2 = fulldiff(th2,GC);
xh = l1*cos(th1); %this is the location of the "hip"
x2 = xh + lc2*cos(th2);

yh = l1*sin(th1); %this is the location of the "hip"
y2 = yh + lc2*sin(th2);

dx2 = fulldiff(x2,GC);
dy2 = fulldiff(y2,GC);

%kinetic energy
T =   1/2*J1*dth1^2 + 1/2*m1*(dth1*lc1)^2    ... %contribution from link 1
    + 1/2*J2*dth2^2 + 1/2*m2*(dx2^2 + dy2^2) ... % link 2

%potential energy 
V = -(m1*g*sin(th1)*lc1 + m2*g*sin(th2)*lc2);

%lagrangian
L = T - V;

%EOMS
eq1 = simplify(fulldiff(diff(L,dth1),GC) - diff(L,th1))
eq2 = simplify(fulldiff(diff(L,dth2),GC) - diff(L,th2))

%non conservtive forces.. (I THINK these were done correctly)
% Xi1 = -(tau2 - b2*dth2) -(tau3 - b3*dth3);
% Xi2 = tau2 - b2*dth2;
% Xi3 = tau3 - b3*dth3; 

% you have to tell matlab these are symbols or else the next step will
% confuse it
% syms d2th1 d2th2 d2th3
% 
% solve(eq1 == Xi1, d2th1)
% solve(eq2 == Xi2, d2th2)
% solve(eq3 == Xi3, d2th3)




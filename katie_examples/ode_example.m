% Katie Byl, june 19, 2017

% Simulate the following dynamic system:
%    m*d2x/dt2 + b*dx/dt + k*x = F
% This is a spring-mass-cart system.

% States are positions (DOFs: degrees of freedom) and velocities of
% the DOFs.
%
% Here, x is the DOF, so x and dx/dt (position and velocity) are
% the STATES.
%
% We use ode45 to implement numerical integration.  To do so,
% we need a function that will output the DERIVATIVES of the
% states in an input state vector, as a function of time (potentially)
% and of the current states.
%
% e.g., Let X be a 2x1 state vector.
% Let X(1) = position (x),
% X(2) = velocity (dx/dt).
% What is the derivative of X(1)?  just dx/dt, which is actually
% just the second state.
% What is the derivative of X(2)? It is d2x/dt2, and we have an
% equation to solve for this:
%  d2x/dt2 = (1/m)*(F - b*dx/dt - k*x) = (1/m)*(F - b*X(2) - k*X(1)),
% since X(1) = x, and X(2) = dx/dt.

x_start = 1.5;
dx_start = 0;
Xinit = [x_start; dx_start];
S = odeset('AbsTol',1e-6,'RelTol',1e-6);
Tsim = 20; % seconds to simulation

[tout,xout] = ode45(@mysimfun,[0 Tsim],Xinit,S);

figure(21); clf
plot(tout,xout)
legend('x','dx/dt')
xlabel('Time (s)')
ylabel('States [m; m/s]')

xb = .3*([0 1 1 0 0]); % box shape
yb = [0 0 .2 .2 0];

figure(22); clf; axis image; hold on
axis([-2 2 -1 2]);
clear M3;
p3a = plot([-2 x_start],[.1 .1],'k-');
p3b = plot(x_start+xb,yb,'r-');
tu = 0:.1:Tsim;
mi = 0; % movie index

for n=1:length(tu)
    xnow = interp1(tout,xout(:,1),tu(n));
    set(p3a,'XData',[-2 xnow]);
    set(p3b,'XData',xnow+xb);
    drawnow
    mi=mi+1;
    M3(mi) = getframe;
end

figure(23); clf; axis off
fps = 1/tu(2);
movie(M3,1,fps);

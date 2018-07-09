function dX = mysimfun(t,X)
% See ode_example, which describes the system.
% This function simply outputs a 2x1 vector of derivatives, dX,
% of the 2x1 input vector X.
%
% Note: the first word in any matlab function is "function", as at
% the top here...

x = X(1);
dx = X(2);

k = 100;
m = 5;
b = 2;

F = 0; % You can reset this to something else!

d2x = (1/m)*(F - b*dx - k*x);

dX = [dx; d2x];
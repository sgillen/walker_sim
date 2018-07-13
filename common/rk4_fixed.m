%single iteration of an rk4 solver, for use with cg_walker_sim, tspan
%should only have two numbers!

 function X = rk4_fixed(func, tspan, X0, u)
 dt = tspan(end) - tspan(1);
 t0 = tspan(1);
 
 k1 = dt*func(t0, X0,u);
 k2 = dt*func(t0 + dt/2, X0 + k1/2,u);
 k3 = dt*func(t0 + dt/2, X0 + k2/2,u);
 k4 = dt*func(t0 + dt, X0 + k3,u);
 
 X = X0 + 1/6*(k1 + 2*k2 + 2*k3 + k4);

 end
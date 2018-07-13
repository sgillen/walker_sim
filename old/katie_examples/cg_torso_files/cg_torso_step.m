function [Xnext,thit] = cg_torso_step(X,bDraw);

if ~exist('bDraw','var')
    bDraw = false;
end

Tmax = 2; % may need to change...
S = odeset('AbsTol',1e-8); %,'RelTol',1e-8);

% Below, choose "normal" vs "open loop u" method:
[tout,xout] = ode45(@cg_torso_ode,[0 Tmax],X,S);
%[tout,xout] = ode45(@cg_torso_ode_u,[0 Tmax],X,S);


xy_start = [0,0]; % where stance toe starts on the ground...
%bDraw = true;
[thit,Xnext] = cg_torso_animate(tout,xout,xy_start,bDraw);
if isempty(Xnext)
    Xnext=zeros(6,1);
else
    Xnext = cg_torso_impact(Xnext);
end
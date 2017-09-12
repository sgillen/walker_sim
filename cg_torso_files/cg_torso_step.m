function [Xnext,thit] = cg_torso_step(X,bDraw, xy_start)

if ~exist('bDraw','var')
    bDraw = false;
end

if ~exist('xy_start','var')
    xy_start = [0,0]; % where stance toe starts on the ground...
end

Tmax = 2; % may need to change...
S = odeset('AbsTol',1e-8, 'Events' , @fall_event); %,'RelTol',1e-8);

% Below, choose "normal" vs "open loop u" method:
[tout,xout] = ode45(@cg_torso_ode,[0 Tmax],X,S);
%[tout,xout] = ode45(@cg_torso_ode_u,[0 Tmax],X,S);


%bDraw = true;
[thit,Xnext] = cg_torso_animate(tout,xout,xy_start,bDraw,[.3,.1]);

if isempty(Xnext)
    Xnext=zeros(6,1);
else
    Xnext = cg_torso_impact(Xnext);
end
clear all % clears persistent variables in cg_torso_controller.m
format compact

clear all % clears persistent variables in cg_torso_controller.m
format compact

%sgillen - not too sure where this Xpost came from, but seems to result in
%a cleaner first step.. so maybe empirically placed here?
Xinit = [ 1.9051
    2.4725
   -0.8654
   -1.2174
    0.5065
    0.2184];

Tmax = 3;

xy_start = [0,0]; % where stance toe starts on the ground...
bDraw = true;
Xlist = Xinit; % list all post-impact states over time

options = odeset('Events', @fall_event);


wn = 30; zeta = 1;
Params.Ctype = 2; % make sure this fits within cg_torso_controller.m
Params.Kp2 = max(0,wn*wn + n*10);
Params.Kd2 = max(0,2*zeta*wn + n);
Params.Kp3 = max(0,wn*wn + n*10);
Params.Kd3 = max(0,2*zeta*wn + n)
Params.th3_ref = 40*pi/180;
Params.th2_ref = (180+30)*pi/180;

cg_controller = CGTorsoController(Params);


walker = GCWalker(cg_controller);

   % simulate another x steps with our new controller
   
%    for i = 1
%        
%        i
%        [tout,xout] = ode45(@cg_torso_ode,[0 Tmax],Xinit,options);
%        [thit,Xhit,xy_start] = cg_torso_animate(tout,xout,xy_start,bDraw, [0,0]);
%        
%        Xlist = [Xlist, cg_torso_impact(Xhit)];
%        Xinit = Xlist(:,end);
%        
%        
%        
%        [tout,xout] = ode45(@cg_torso_ode,[0 Tmax],Xinit,options);
%        [thit,Xhit,xy_start] = cg_torso_animate(tout,xout,xy_start,bDraw, [0,0]);
%        
%        % New limit cycle, with new controller defined
%        cg_torso_find_limit_cycle
%        
%    end
%% Next, we could solve for the nominal u and d2X associated with the
% limit cycle, as input parameters for PFL control:
% Tend = thit + 0.5;
% fi = find(tout<=Tend);
% Xnom = xout(fi,:)';
% tnom = tout(fi)';
% d2Xnom = zeros(5,length(tnom));
% unom = zeros(2,length(tnom));
% 
% for n=1:length(tnom)
%     [d2xnom(:,n),unom(:,n)] = cg_torso_ode(tnom(n),Xnom(:,n));
% end


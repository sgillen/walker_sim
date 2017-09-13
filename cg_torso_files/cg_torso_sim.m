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

wn = 30; zeta = 1;
Params.Ctype = 2; % make sure this fits within cg_torso_controller.m
Params.Kp2 = wn*wn;
Params.Kd2 = 2*zeta*wn;
Params.Kp3 = wn*wn;
Params.Kd3 = 2*zeta*wn;
Params.th3_ref = 40*pi/180;
Params.th2_ref = (180+30)*pi/180;

controller = CGTorsoController(Params);

walker = CGTorsoWalker(controller);

[t,y] = walker.runSim([0 Tmax], Xinit);

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

%% cg_torso_sim (sean gillen 9/13/17)
% this script serves as the entry point for the walker simulation. it
% instansiates a


clear all % clears persistent variables in cg_torso_controller.m
format compact


% wn = 30; zeta = 1;
% Params.Ctype = 2; % make sure this fits within cg_torso_controller.m
% Params.Kp2 = wn*wn;
% Params.Kd2 = 2*zeta*wn;
% Params.Kp3 = wn*wn;
% Params.Kd3 = 2*zeta*wn;
% Params.th3_ref = 40*pi/180;
% Params.th2_ref = (180+30)*pi/180;


wn = 30; zeta = 1;
Params.Ctype = 2; % make sure this fits within cg_torso_controller.m
Params.Kp2 = 0;
Params.Kd2 = 0;
Params.Kp3 = 0;
Params.Kd3 = 0;
Params.th3_ref = 40*pi/180;
Params.th2_ref = (180+30)*pi/180;

controller = CGTorsoController();
walker = CGTorsoWalker(controller);

Xinit =[ 1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184]; %state vars at the start of our simulation
Xnext = Xinit

walker.cgFindLimitCycle(Xinit)

% for i = 1:10 
% 
%     [Xnext,flag] = walker.runSim(Xnext);
%     flag
%     if flag == 2
%         break
%     end
%     walker.cgTorsoAnimate(walker.t,walker.X)
%    
% 
% end



%% Next, we could solve for the nominal u and d2X associated with the
%limit cycle, as input parameters for PFL control:
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

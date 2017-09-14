%% cg_torso_sim (sean gillen 9/13/17)
% this script serves as the entry point for the walker simulation. it
% instansiates a


clear all % clears persistent variables in cg_torso_controller.m
format compact

controller = CGTorsoController();
controller.Ctype = 1; 

walker = CGTorsoWalker(controller);

Xinit =[ 1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184]; %state vars at the start of our simulation

% Xinit=[1.9294
%     2.4247
%    -0.7119
%    -1.1819
%     0.5618
%     1.2083];

Xnext = Xinit

walker.cgFindLimitCycle(Xinit)

% 
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

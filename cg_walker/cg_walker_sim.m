%% cg_torso_sim (sean gillen 9/13/17)
% this script serves as the entry point for the walker simulation. The meat
% of the functionality is in the two classes, but this script is the one
% creating the objects and calling the right methods
% 


clear all % clears persistent variables in cg_torso_controller.m
format compact

controller = CGTorsoController();
controller.Ctype = 1; 

walker = CGTorsoWalker(controller);
% 
%  Xinit=[1.9294
%         2.4247
%         -0.7119
%         -1.1819
%         0.5618
%         1.2083];

num_controllers = 15
num_noise_vals = 15
num_trials = 10

eivals = zeros(6,num_controllers,num_noise_vals,num_trials);


%walkers = cell(imax,jmax,kmax);

Xinit =[ 1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184]; %state vars at the start of our simulation

%this seems like magic to me, but if you fill a value of an array with an
%object it will populate smaller values in the array with the default
%object, effecitvley initializing our walker matrix

walkers(num_controllers,num_noise_vals,num_trials) = CGTorsoWalker();
controllers(num_controllers) = CGTorsoController(); 
  

for i = 1:num_controllers;
   
    %each 
    controller(i) = CGTorsoController();
    controller(i).Kp2 = i*100; 
    
    for j = 1:num_noise_vals
        
        for k = 1:num_trials
            
            fprintf("%2.2f %% complete \n",(i*j*k)/(num_controllers*num_noise_vals*num_trials)*100)
            %(i*j*k)/(num_controllers*num_noise_vals*num_trials)*100
            
            walkers(i,j,k) = CGTorsoWalker(controller(i));
            walkers(i,j,k).initSensorNoise(k,.05 * (j > 1),0.02*(j));  % the logical expression here is just an easy way to make the bias term zero in only the perfect sensing case
            %eivals(:,i) = walkers(i).cgFindLimitCycle(Xinit);
            eivals(:,i,j,k) = walkers(i,j,k).cgFindLimitCycleEvent(Xinit);
            
            
        end
    end  
end
%walker.cgTorsoAnimate(walker.t,walker.X);

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

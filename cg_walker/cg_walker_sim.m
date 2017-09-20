%% cg_torso_sim (sean gillen 9/13/17)
% this script serves as the entry point for the walker simulation. The meat
% of the functionality is in the two classes, but this script is the one
% creating the objects and calling the right methods
%
%
% 


clear all % clears persistent variables in cg_torso_controller.m
format compact

 Xinit=[1.9294
        2.4247
        -0.7119
        -1.1819
        0.5618
        1.2083];

num_controllers = 15
num_noise_vals = 10
num_trials = 3

eivals = zeros(6,num_controllers,num_noise_vals,num_trials);


%walkers = cell(imax,jmax,kmax);

Xinit =[ 1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184]; %state vars at the start of our simulation

%this seems like magic to me, but if you fill a value of an array with an
%object it will populate smaller values in the array with the default
%object, effecitvley initializing our walker matrix
walkers(num_controllers,num_noise_vals,num_trials) = CGTorsoWalker();
controllers(num_controllers) = CGTorsoController(); 
  
count = 0
for i = 1:num_controllers
   
    %each 
    controllers(i) = CGTorsoController();
    controllers(i).Kp2 = i*100; 
    
    for j = 1:num_noise_vals
        
        for k = -1:1
            
            fprintf("controller %i, noise_val %i, trial %i,  %2.2f%% complete \n", i,j,k ,(count)/(num_controllers*num_noise_vals*num_trials)*100)
            count = count+1;
            %(i*j*k)/(num_controllers*num_noise_vals*num_trials)*100
            
            walkers(i,j,k+2) = CGTorsoWalker(controllers(i));
            walkers(i,j,k+2).initSensorNoise(1, .01*j*k , 0);  % the logical expression here is just an easy way to make the bias term zero in only the perfect sensing case
            %eivals(:,i) = walkers(i).cgFindLimitCycle(Xinit);
            eivals(:,i,j,k+2) = walkers(i,j,k+2).cgFindLimitCycleEvent(Xinit);
            
            
        end
    end  
end

% 
% tot = zeros(num_controllers,num_noise_vals);
% 
% for i = 1:num_controllers
%    for j = 1:num_noise_vals
%        for k = 1:num_trials
%            if abs(max(eivals(:,i,j,k))) < 1
%                tot(i,j) = tot(i,j) + 1;
%            end
%        end
%    end
% end
% 
% tot = 0;
% 
%  for k = 1:num_trials
%             if abs(max(eivals(:,i,j,k))) < 1
%                 tot = tot + 1;
%             end
%  end

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

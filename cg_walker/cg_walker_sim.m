% cg_torso_sim (sean gillen 9/13/17)
% this script serves as the entry point for the walker simulation. The meat
% of the functionality is in the two classes, but this script is the one
% creating the objects and calling the right methods
%

clear all % clears persistent variables in cg_torso_controller.m
format compact

%these define how big a sweep we do over each parameter
num_controllers = 12
num_noise_vals = 10
num_trials = 3
max_steps = 11


%The idea here is to create a whole array of walker objects that each keep
%track of their state after they try and take some steps, this let's you
%choose one walker configuration, load it into memory, and see why it fell
%down for example.

%this seems like magic to me, but if you fill a value of an array with an
%object it will populate smaller values in the array with the default
%object, effectivley initializing our walker matrix
walkers(num_controllers,num_noise_vals,num_trials,max_steps) = CGTorsoWalker();
controllers(num_controllers) = CGTorsoController(); 


%there may be a much better way to do these, especially since for the most
%part each trial is independent of any others, could massivley parrellize
count = 0;

%what are we interested in changing?
% walker.controller.Kp2
% walker.controller.Kp3
% walker.controller.th2_ref
% walker.controller.th3_ref
% walker.L1c + walker.L2c == these should be kept the same
% walker.L3c > moments of inertia should change with these.. maybe point mass approx?





for i = 1:num_controllers
   
    %define each controller we will use, change the Kp2 value
    controllers(i) = CGTorsoController();
    controllers(i).Kp3 = (i-1)*100; 
    walkers(i) = CGTorsoWalker(controllers(i));
    walkers(i).findLimitCycle();
    
end



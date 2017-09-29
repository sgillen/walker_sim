% cg_torso_sim (sean gillen 9/13/17)
% this script serves as the entry point for the walker simulation. The meat
% of the functionality is in the two classes, but this script is the one
% creating the objects and calling the right methods
%

clear all % clears persistent variables in cg_torso_controller.m
format compact


%what are we interested in changing?
% walker.controller.Kp2
% walker.controller.Kp3
% walker.controller.th2_ref
% walker.controller.th3_ref
% walker.L1c + walker.L2c == these should be kept the same
% walker.L3c > moments of inertia should change with these.. maybe point mass approx?


max_steps = 1000;
walker(max_steps) = CGTorsoWalker();

cur_max_step = zeros(1,max_steps);
cur_max_step(1) = maxStep(walker(1))

rng(7);

for step_num = 2:max_steps
    
    step_size = .05; %will need to find a smart way to change this
    param = floor(rand*8) + 1; %we select a random order to try the parameters
    direction = (-1)^(floor(rand*2)); %maybe a bit confusing, but this just selects between 1 and -1 randomly.
    
    %this is ugly, but efficient
    switch param
        case 1
            walker(step_num).controller.Kp2 = walker(step_num-1).controller.Kp2 + step_size*direction;
        case 2
            walker(step_num).controller.Kd2 = walker(step_num-1).controller.Kd2 + step_size*direction;
        case 3
            walker(step_num).controller.Kp3 = walker(step_num-1).controller.Kp3 + step_size*direction;
        case 4
            walker(step_num).controller.Kd3 = walker(step_num-1).controller.Kd3 + step_size*direction;
            
        case 5
            walker(step_num).controller.th2_ref = walker(step_num-1).controller.th2_ref + step_size*direction;
        case 6
            walker(step_num).controller.th3_ref = walker(step_num-1).controller.th3_ref + step_size*direction;
            
        case 7
            walker(step_num).L1c = walker(step_num-1).L1c + step_size*direction;
            walker(step_num).L2c = walker(step_num-1).L2c + step_size*direction;
        case 8
            walker(step_num).L3c = walker(step_num-1).L3c + step_size*direction;
    end
    
       
        candidate_max_step = maxStep(walker(step_num))
        cur_max_step(step_num-1)
        step_num
        
        if candidate_max_step > cur_max_step(step_num-1)
            cur_max_step(step_num) = candidate_max_step;
        else
            walker(step_num) = copy(walker(step_num -1));
            cur_max_step(step_num) = cur_max_step(step_num-1);
        end
            
    
    
    
    
end





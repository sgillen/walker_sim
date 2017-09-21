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
step_size = zeros(num_controllers,num_noise_vals,num_trials)
  
%I know you are not supposed to use loops in matlab but I couldn't think of
%a good way to vectorize it..

%there may be a much better way to do these, especially since for the most
%part each trial is independent of any others, could massivley parrellize
count = 0
for i = 1:num_controllers
   
    %define each controller we will use, change the Kp2 value
    controllers(i) = CGTorsoController();
    controllers(i).Kp2 = i*100; 
    
    for j = 1:num_noise_vals
        
        for k = 1:num_trials
            %tell the user how much time is left
            count = count+1;
            
            %step keeps track of how big our step up/dow is
            step = 0; 
            for n = 1:(max_steps)  
                fprintf("controller %i, noise_val %i, trial %i, step %i  %2.2f%% complete \n", i,j,k,n ,(count)/(num_controllers*num_noise_vals*num_trials)*100)

                %make the walker, pass it it's controller, init some noise
                %and make a step.
                walkers(i,j,k,n) = CGTorsoWalker(controllers(i));
                walkers(i,j,k,n).initSensorNoise(1, .01*j*(k-2) , 0); %the second argument here is the bias and the third the extra noise on top of that 
                walkers(i,j,k,n).xy_step = [.3,.03*(n-1)]; %n-1 so that we have one case where we are walking on level ground
       
                %if we failed to take a step up just stop and do the next
                %loop
                if max(abs(walkers(i,j,k,n).findLimitCycle())) > 1
                    break
                end
                
                %if we did take a step then make the next one bigger
                
                step = step + 1;
            end
            
            step_size(i,j,k) = n; 
            
        end
    end  
end



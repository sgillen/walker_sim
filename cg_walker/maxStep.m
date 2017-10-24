function [step_height]= maxStep(in)
% sgillen 10/13/17
% This function serves as the objective function, intended for input to
% functions in the global optimization toolbox. Accepts a length 6 vector
% as input, representing 6 parameters for the 2 PD controllers on the
% walker. Inputs are meant to be between 0 and 1 and are scaled
% appropiatley below. 

% the "objective" being optimized here is the maximum step up the walker
% can take


walker = CGTorsoWalker()

walker.controller.th2_ref = in(1) * 2*pi;
walker.controller.th3_ref = in(2) * 2*pi; 

walker.controller.kp2 = in(3) * 1000;
walker.controller.kd2 = in(4) * 100; 

walker.controller.kp3 = in(5) * 1000;
walker.controller.kd3 = in(6) * 100; 


% this function increases the y in xy_step until the walker falls over

step_inc = .01; %tells us how finely to increase the step height by


walker.Xinit = [1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184] %this is the defualt when you make a walker object, helps us to only find sane limit cycles
walker.findLimitCycle(); %this will make the first call to take step

%make sure we found a valid limit cycle.
%if we didn't return 0 for the step height. 
if max(abs(walker.eival)) > 1 
    step_height = 0; 
    return 
end

Xinit_nom = walker.Xinit; %this is changed by findLimitCycle

for i = 0:100
    step_height = step_inc*i;
    walker.xy_step = [.2, step_height]; %raise the step height
    walker.xy_end = [0,0]; %reset our starting position
    walker.Xinit = Xinit_nom; 
    
    
    %take 10 steps, the first is a step up, every other one is after the
    %step and to make sure we are reasonably stable. 
    for i = 1:10
        [~,flag] = walker.takeStep();
        if flag ~= 1
            return
        end
    end
    
 
end

%if we got here then the walker was able to take the maximum number of
%steps and we return
return


end
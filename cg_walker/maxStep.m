function [step_height]= maxStep(walker, dir)
% sgillen 10/13/17
% This function will take your walker and figure out how big of a step it
% can take without falling over. dir should be +- 1, it tells you if you
% want to take a step up (dir == 1) or down (dir == -1)

% the "objective" being optimized here is the maximum step up the walker
% can take

% this function increases the y in xy_step until the walker falls over

step_inc = .01*dir; %tells us how finely to increase the step height by


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
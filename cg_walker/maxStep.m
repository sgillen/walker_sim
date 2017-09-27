function [step_height]= maxStep(walker)
% this function increases the y in xy_step until the walker falls over

step_inc = .01; %tells us how finely to increase the step height by


walker.Xinit = [1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184] %this is the defualt when you make a walker object, helps us to only find sane limit cycles
walker.findLimitCycle(); %this will make the first call to take step 
Xinit_nom = walker.Xinit; %this is changed by findLimitCycle

for i = 0:100
    step_height = step_inc*i;
    walker.xy_step = [.2, step_height]; %raise the step height
    walker.xy_end = [0,0]; %reset our starting position
    walker.Xinit = Xinit_nom; 
    
    [~,flag] = walker.takeStep();
    if flag ~= 1
       return
    end
    
    [~,flag] = walker.takeStep();
    if flag ~= 1
       return
    end

end

%if we got here then the walker was able to take the maximum number of
%steps and we return
return


end
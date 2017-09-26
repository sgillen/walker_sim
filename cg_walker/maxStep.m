function [step_height]= maxStep(walker)
% this function increases the y in xy_step until the walker falls over

persistent step_inc;
step_inc = .01; %tells us how finely to increase the step height by


for i = 0:100
    walker.xy_step = [.2, step_inc*i] 
    walker.xy_start = [0,0];
    
    if walker.takeStep() ~= 1
       step_height = step_inc*i;
       return
    end
    
    if walker.takeStep() ~= 1
       step_height = step_inc*i;
       return
    end
    
    if max(abs(walker.findLimitCycle)) > 1
       step_height = step_inc*i; 
       return
    end
    
end

%if we got here then the walker was able to take the maximum number of
%steps and we return

step_height = step_inc*i;
return


end
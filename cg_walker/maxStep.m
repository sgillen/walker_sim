function [step_height]= maxStep(walker, dir)
% sgillen 10/13/17
% This function will take your walker and figure out how big of a step it
% can take without falling over. dir should be +- 1, it tells you if you
% want to take a step up (dir == 1) or down (dir == -1)

% the "objective" being optimized here is the maximum step up the walker
% can take

% this function increases the y in xy_step until the walker falls over
 step_height = 0; 

step_inc = .01*dir; %tells us how finely to increase the step height by


walker.Xinit = [1.9051; 2.4725; 0 ; -1.2174; 0.5065; 0.2184] %this is the default when you make a walker object, helps us to only find sane limit cycles 
[eival, Xinit_nom] = walker.findLimitCycle(); %this will make the first call to take step

%make sure we found a valid limit cycle.
%if we didn't return 0 for the step height. 
if max(abs(eival)) > 1 
    step_height = 0; 
    return 
end

for i = 0:100
    walker.reset() %this returns the walker to the origin
    walker.xy_step = [.2, step_inc*i]; %raise the step height
    walker.Xinit = Xinit_nom; 
    
    
    %take 10 steps, the first is a step up, every other one is after the
    %step and to make sure we are reasonably stable. 
    
    %TODO, find eigenvalues of the return map? and then maybe take steps if eigenvalues are
    %between .9 and 2 or something?
    
    
    walker.takeStep(); %step up (or down)
    [Xnext,flag] = walker.takeStep(); %take another step to get both feet up the step
    [Xnext,flag] = walker.takeStep(); %take another step to get both feet up the step


    if flag ~= 1 % this means we fell and can stop the iteration
        return
    end
    
    damt = 1e-4;
    J = zeros(6,6);
    
        for n=1:6
            d = zeros(6,1); d(n)=damt;
            xtemp = walker.runSim(Xnext + d);
            xtemp2 = walker.runSim(Xnext - d);
            J(:,n) = (1/(2*damt))*(xtemp-xtemp2);
        end
    
        [eivec,eival] = eig(J);
    
        if max(abs(eival)) > 1
            return
        end
    
    
step_height = step_inc*i;
    
 
end

%if we got here then the walker was able to take the maximum number of
%steps and we return
return


end
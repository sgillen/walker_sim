function [step_height]= maxStep(walker, dir)
% sgillen 10/13/17
% This function will take your walker and figure out how big of a step it
% can take without falling over. dir should be +- 1, it tells you if you
% want to take a step up (dir == 1) or down (dir == -1)

% the "objective" being optimized here is the maximum step up the walker
% can take

% this function increases the y in xy_step until the walker falls over
step_height = 0; 
 
step_inc = .005*dir; %tells us how finely to increase the step height by

walker.reset()

walker.xy_step = [0,0];
walker.Xinit = [1.9051; 2.4725; walker.th3_ref - 1.9051 ;  -1.1583; 0.7449 ;-0.3878]; %this is the default when you make a walker object, helps us to only find sane limit cycles 

try 
    [eival1, Xinit_nom1,flag1] = walker.findLimitCycle(); %this will make the first call to take step
    [eival2, Xinit_nom2,flag2] = walker.findLimitCycle(); %this will make the first call to take step
catch
    lasterror
    step_height = -.03;
    return
end

%make sure we found a valid limit cycle.
%if we didn't return 0 for the step height. 

if flag1 <= 0 && flag2  <=0
   step_height = flag1*.01 + flag2*.001;
   return
end

if max(abs(eival1)) > 1 && max(abs(eival2) > 1)
    step_height = -.02; 
    return 
end

if max(abs(eival1) > max(abs(eival2)))
    Xinit_nom = Xinit_nom2;
else
    Xinit_nom = Xinit_nom1;
end

tol = .01;
max_step = .4;
dh = max_step;
num_iterations = 25; % number of iterations to try, it's always 
tmp_step_height = step_height; 
for i = 0:num_iterations
    dh = dh/2;
    walker.reset(); %this returns the walker to the origin
    walker.xy_step = [.2, tmp_step_height]; %raise the step height
    walker.Xinit = Xinit_nom; 
     
    
    walker.takeStep(); %step up (or down)
    
    if(walker.xy_end{walker.step_num}(1) < walker.xy_step(1))
        tmp_step_height = tmp_step_height - dh;
        continue;        
    end
            
    [Xnext,flag] = walker.takeStep(); %take another step to get both feet up the step
    [Xnext,flag] = walker.takeStep(); %take another step to get both feet up the step


    if flag ~= 1 % this means we fell 
        tmp_step_height = tmp_step_height - dh;
        continue; 
    end
    
    damt = 1e-4;
    J = zeros(6,6);
    
        for n=1:6
            d = zeros(6,1); d(n)=damt;
            xtemp = walker.runSim(Xnext + d);
            xtemp2 = walker.runSim(Xnext - d);
            J(:,n) = (1/(2*damt))*(xtemp-xtemp2);
        end
    
        try
            [~,eival] = eig(J);
        catch
            tmp_step_height = tmp_step_height - dh;
            continue;
        end

    
        if max(abs(eival)) > 1
             tmp_step_height = tmp_step_height - dh;
             continue;
            
        else
            step_height = tmp_step_height;
            tmp_step_height = tmp_step_height + dh;
        end
    
    
         if(tmp_step_height < 0)
             step_height = -.04;
             return
         end
  
end

%if we got here then the walker was able to take the maximum number of
%steps and we return
return


end
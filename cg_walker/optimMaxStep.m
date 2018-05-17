function [step_height, walker] = optimMaxStep(in)
% sgillen 4/20/18
% This function serves as the objective function, intended for input to
% functions in the global optimization toolbox. Accepts a length 6 vector
% as input, representing 6 parameters for the 2 PD controllers on the
% walker. Inputs are meant to be between 0 and 1 and are scaled
% appropiatley below. 

% the "objective" being optimized here is the maximum step up the walker
% can take


walker = CGTorsoWalker();
%walker.controller.th3_ref = 0;

walker.controller.th3_ref = in(1) * 30*pi/180; 

%walker.controller.kp2 = in(3) * 1000;
%walker.controller.kd2 = in(4) * 100; 

walker.controller.kp3 = in(2) * 800 +200; %one is the max input from the solver so 1000 is the max kp3 we will allow.
walker.controller.kd3 = in(3) * 100; 

  
walker.m3 = in(4)*25 + 5;
walker.m2 = (30 - walker.m3)/2;
walker.m1 = walker.m2;

walker.J1 = 1/3*walker.m1*walker.L1^2
walker.J2 = 1/3*walker.m2*walker.L2^2
walker.J3 = 1/3*walker.m3*walker.L3^2





%this just makes the maximum step up into a minimization problem
%The 100 is just because some solvers don't like starting at zero. 
step_height = 100 - maxStep(walker,1);


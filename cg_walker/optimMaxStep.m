function [step_height] = optimMaxStep(in)
% sgillen 4/20/18
% This function serves as the objective function, intended for input to
% functions in the global optimization toolbox. Accepts a length 6 vector
% as input, representing 6 parameters for the 2 PD controllers on the
% walker. Inputs are meant to be between 0 and 1 and are scaled
% appropiatley below. 

% the "objective" being optimized here is the maximum step up the walker
% can take


walker = CGTorsoWalker()

%walker.controller.th2_ref = in(1) * 2*pi;
%walker.controller.th3_ref = in(2) * 2*pi; 

%walker.controller.kp2 = in(3) * 1000;
%walker.controller.kd2 = in(4) * 100; 

walker.controller.kp3 = in(1) * 1000; %one is the max input from the solver so 1000 is the max kp3 we will allow.
walker.controller.kd3 = in(2) * 100; 

step_height = maxStep(walker,1);


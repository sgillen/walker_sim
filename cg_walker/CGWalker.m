classdef CGWalker
%% This class 

properties
    g = 9.81; % gravity
    
    m1 = 10;
    L1 = 1;
    L1c = 0.5; % wrt hip joint (going down each leg)
    J1 = m1*0.16; % m*r^2, approx guess
    
    L3 = 0.8; % L3 is only for "animation" (not dynamics)
    L3c = 0.4; % only "L3c" affect dynamics
    m3 = 10;
    J3 = m3*0.25; % to be played with...
    
    m2 = m1;
    J2 = J1;
end

methods 
    
    
end

    
end
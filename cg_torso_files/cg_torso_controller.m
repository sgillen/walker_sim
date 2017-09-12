function [u,Params] = cg_torso_controller(t,X,bReset,Params)
%% This function is several different control strategies bundled into one.
%    to set the control gains you need to do a call like this:
%
%    cg_torso_controller([],[],bReset,Params);
%    
%    where bReset is logical true, and Params is a struct whos structure
%    you can infer from this file
%
%    once everything is set up you can use this function to calculate your
%    control effor as follows: 
%    
%    u = cg_torso_controller(t,X);
%
%    where t and X are the same vectors you would pass to an ode45 function
%   
%   
%  TODO we don't actually use the t vector, maybe eliminate it?
%  TODO this might be cleaner as a class, athough oop in matlab is a bit
%  jank


%% define control parameters.

persistent Kp2 Kp3 Kd2 Kd3 th3_ref th2_ref Ctype

% give a numeric ID to each "control type"...

PD_Ctype = 1; % simple PD control, we control the ABSOLUTE TORSO angle and the RELATIVE SWING angle
PD_AbsSwing_Ctype = 2; % simple PD control, we control the ABSOLUTE TORSO angle and the ABSOLUTE SWING angle
PFL_Ctype = 3; % to be implemented. suggest using abs torso and rel interleg angle


%% reset our gains if we need to, or set them to defaults if we don't have values already
if exist('bReset','var') && bReset % if "true"
    Ctype = Params.Ctype;
    Kp2 = Params.Kp2;
    Kd2 = Params.Kd2;
    Kp3 = Params.Kp3;
    Kd3 = Params.Kd3;
    th3_ref = Params.th3_ref;
    th2_ref = Params.th2_ref;
    
    u = true;
    
    %if you reset the parameters we return right away
    return
    
elseif isempty(Kp2)
    Kp2=400; Kd2=40;
    Kp3=400; Kd3=40;
    th3_ref = 60*pi/180; % absolute angle, wrt x axis, measured CCW
    th2_ref = (180+35)*pi/180;
    Ctype = PD_Ctype;
  
end
    
% Define states, we need to do this regardless of the controller type we go
% with

th1 = X(1);
th2 = X(2);
th3 = X(3);
dth1 = X(4);
dth2 = X(5);
dth3 = X(6);

switch Ctype
    
    case PD_Ctype
        % Combine states to define parameters to be directly controlled:
        th3_abs = th1+th3;
        dth3_abs = dth1+dth3;
        th2_rel = th2;
        dth2_rel = dth2;
        
        % Below is the simple PD control law
        u2 = Kp2*(th2_ref - th2_rel) + Kd2*(0 - dth2_rel);
        u3 = Kp3*(th3_ref - th3_abs) + Kd3*(0 - dth3_abs);
        
     case PD_AbsSwing_Ctype
        % Combine states to define parameters to be directly controlled:
        th3_abs = th1+th3;
        dth3_abs = dth1+dth3;
        th2_abs = th1+th2;
        dth2_abs = dth1+dth2;
        
        % Below is the simple PD control law
        u2 = Kp2*(th2_ref - th2_abs) + Kd2*(0 - dth2_abs);
        u3 = Kp3*(th3_ref - th3_abs) + Kd3*(0 - dth3_abs);
    otherwise
        error('You have your controller configured to an invalid type!!\n')
end

u = [u2; u3];
end

function [u,Params] = cg_torso_controller(t,X,bReset,Params)

%% To be done: allow for a flag, bReset, to enable user to
% define control parameters.

persistent Kp2 Kp3 Kd2 Kd3 th3_ref th2_ref Ctype

PD_Ctype = 1; % give a numeric ID to each "control type"...
PFL_Ctype = 2; % to be implemented. suggest using abs torso and rel interleg angle

if exist('bReset','var') && bReset % if "true"
    Ctype = Params.Ctype;
    Kp2 = Params.Kp2;
    Kd2 = Params.Kd2;
    Kp3 = Params.Kp3;
    Kd3 = Params.Kd3;
    th3_ref = Params.th3_ref;
    th2_ref = Params.th2_ref;
    
    u = true;
else
    
    if isempty(Kp2)
        Kp2=400; Kd2=40;
        Kp3=400; Kd3=40;
        th3_ref = 60*pi/180; % absolute angle, wrt x axis, measured CCW
        th2_ref = (180+35)*pi/180;
        Ctype = PD_Ctype;
    end
    
    Params.Ctype = Ctype;
    Params.Kp2 = Kp2;
    Params.Kd2 = Kd2;
    Params.Kp3 = Kp3;
    Params.Kd3 = Kd3;
    Params.th3_ref = th3_ref;
    Params.th2_ref = th2_ref;
    
    % Define states:
    th1 = X(1);
    th2 = X(2);
    th3 = X(3);
    dth1 = X(4);
    dth2 = X(5);
    dth3 = X(6);
    
    if Ctype == PD_Ctype
        % Combine states to define parameters to be directly controlled:
        th3_abs = th1+th3;
        dth3_abs = dth1+dth3;
        th2_rel = th2;
        dth2_rel = dth2;
        
        % Below is the simple PD control law
        u2 = Kp2*(th2_ref - th2_rel) + Kd2*(0 - dth2_rel);
        u3 = Kp3*(th3_ref - th3_abs) + Kd3*(0 - dth3_abs);
    else
        error('Whoops...')
    end
    
    u = [u2; u3];
end

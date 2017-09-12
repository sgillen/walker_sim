classdef CGTorsoController
    %% This class is several different control strategies bundled into one.
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
    
    %% properties
   
    
    properties (Constant)
        % give a numeric ID to each "control type"...
        PD_CTYPE = 1; % simple PD control, we control the ABSOLUTE TORSO angle and the RELATIVE SWING angle
        PD_ABSWING_CTYPE = 2; % simple PD control, we control the ABSOLUTE TORSO angle and the ABSOLUTE SWING angle
        PFL_CTYPE = 3; % to be implemented. suggest using abs torso and rel interleg angle 
    end
    
     properties
        Kp2; Kd2;
        Kp3; Kd3;
        th3_ref; th2_ref;
        Ctype = 1; %this is PD_CTYPE
    end
    
    methods
        %% constructor, you can either pass params or use the default values
        function obj = CGTorsoController(Params)
            if nargin > 0
                obj.Kp2 = Params.Kp2;
                obj.Kd2 = Params.Kd2;
                obj.Kp3 = Params.Kp3;
                obj.Kd3 = Params.Kd3;
                obj.th2_ref = Params.th2_ref;
                obj.th3_ref = Params.th3_ref;
                obj.Ctype = Params.Ctype;
            else
                %default values, this is what you get if you pass in nothing
                obj.Kp2=400;
                obj.Kd2=40;
                obj.Kp3=400;
                obj.Kd3=40;
                obj.th3_ref = 60*pi/180; % absolute angle, wrt x axis, measured CCW
                obj.th2_ref = (180+35)*pi/180;
                obj.Ctype = obj.PD_Ctype;
            end
        end
        
        
        %% this is sort of the meat of the class, we use the configured gains and given measurments to compute our control efforts
        function [u] = calculate_control_efforts(t,X)
            
            th1 = X(1);
            th2 = X(2);
            th3 = X(3);
            dth1 = X(4);
            dth2 = X(5);
            dth3 = X(6);
            
            %this class is set up so you can use a variety of control
            %schemes, see the constant parameters above to see what you
            %need to pass to the constructor to have each
            switch obj.Ctype
                
                case obj.PD_Ctype
                    % Combine states to define parameters to be directly controlled:
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_rel = th2;
                    dth2_rel = dth2;
                    
                    % Below is the simple PD control law
                    u2 = obj.Kp2*(obj.th2_ref - th2_rel) + obj.Kd2*(0 - dth2_rel);
                    u3 = obj.Kp3*(obj.th3_ref - th3_abs) + obj.Kd3*(0 - dth3_abs);
                    
                case obj.PD_AbsSwing_Ctype
                    % Combine states to define parameters to be directly controlled:
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_abs = th1+th2;
                    dth2_abs = dth1+dth2;
                    
                    % Below is the simple PD control law
                    u2 = obj.Kp2*(obj.th2_ref - th2_abs) + obj.Kd2*(0 - dth2_abs);
                    u3 = obj.Kp3*(obj.th3_ref - th3_abs) + obj.Kd3*(0 - dth3_abs);
                otherwise
                    error('You have your controller configured to an invalid type!!\n')
            end
            
            u = [u2; u3];
        end
    end
end

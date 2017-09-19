classdef CGTorsoController
    %% This class is several different control strategies bundled into one. It is designed to work with CGTorseWalker
    %   the basic workflow I've been using is the same as with the
    %   CGTorsoWalker: I use the default constructor and then change the
    %   parameters I want: 
    % 
    %   controller = CGTorsoController()
    %   controler.Kp2 = 500;
    %   controller.Ctype = 2;
    %
    %   the Ctype property is what determines which controller we use, see
    %   below for what each number corresponds to
    
    %% properties
    properties (Constant)
        % give a numeric ID to each "control type"...
        PD_CTYPE = 1; % simple PD control, we control the ABSOLUTE TORSO angle and the RELATIVE SWING angle
        PD_ABSWING_CTYPE = 2; % simple PD control, we control the ABSOLUTE TORSO angle and the ABSOLUTE SWING angle
        PFL_CTYPE = 3; % to be implemented. suggest using abs torso and rel interleg angle
    end
    
    properties
        %default values, this is what you get if you pass in nothing
        Kp2=400;
        Kd2=40;
        Kp3=400;
        Kd3=40;
        th3_ref = 60*pi/180; % absolute angle, wrt x axis, measured CCW
        th2_ref = (180+35)*pi/180;
        Ctype = 1;
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
            end
            %if no argument is passed in, we just use default values as
            %defined in the params
        end
        
        
        %% this is sort of the meat of the class, we use the configured gains and given measurments to compute our control efforts
        function [u] = calculateControlEfforts(obj,X,M,C,G)
            
            th1 = X(1);
            th2 = X(2);
            th3 = X(3);
            dth1 = X(4);
            dth2 = X(5);
            dth3 = X(6);
            
            %this class is set up so you can use a variety of control
            %schemes, see the constant parameters above to see what you
            %need to pass to the constructor to have each trigger
            switch obj.Ctype
                
                case obj.PD_CTYPE
                    % Combine states to define parameters to be directly controlled:
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_rel = th2;
                    dth2_rel = dth2;
                    
                    % Below is the simple PD control law
                    u2 = obj.Kp2*(obj.th2_ref - th2_rel) + obj.Kd2*(0 - dth2_rel);
                    u3 = obj.Kp3*(obj.th3_ref - th3_abs) + obj.Kd3*(0 - dth3_abs);
                    
                case obj.PD_ABSWING_CTYPE
                    % Combine states to define parameters to be directly controlled:
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_abs = th1+th2;
                    dth2_abs = dth1+dth2;
                    
                    % Below is the simple PD control law
                    u2 = obj.Kp2*(obj.th2_ref - th2_abs) + obj.Kd2*(0 - dth2_abs);
                    u3 = obj.Kp3*(obj.th3_ref - th3_abs) + obj.Kd3*(0 - dth3_abs);
                case obj.PFL_CTYPE
                    %Partial Feedback linearization, as found in
                    %walker2_noncollocated
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_abs = th1+th2;
                    dth2_abs = dth1+dth2;
                    
                    u3 = 0;
                    u2 = M(2,2).^(-1).*(M(1,2)+M(3,3)).^(-1).*((-1).*M(2,2).*M(3,3).*G(1)+M(1,2).*M(3,3).*G(2)+M(1,2).*M(2,2).*G(3)+(-1).*M(1,2).*M(2,2).*TAU(2)+M(1,2).*M(3,3).*TAU(2)+C(3,1).*M(1,2).*M(2,2).*(dth1))+C(3,1).*M(1,2).*M(3,3).*(dth1)+M(1,2).*M(2,2).*M(1,3).*(obj.Kp2.*(obj.th2_ref+(-1).*th1)+(-1).*obj.Kd2.*(dth1))+M(1,2).*M(1,2).*M(3,3).*(obj.Kp2.*(obj.th2_ref+(-1).*th1)+(-1).*obj.Kd2.*(dth1))+(-1).*d11.*M(2,2).*M(3,3).*(obj.Kp2.*(obj.th2_ref+(-1).*th1)+(-1).*obj.Kd2.*(dth1))+(-1).*c12.*M(2,2).*M(3,3).*(dth2_abs)+(-1).*C(1,3).*M(2,2).*M(3,3).*(dth3_abs);
                    
                    torque_limit = 10000;  % [Nm] limit in torque magnitude
                    u2 = sign(u2)*min(abs(u2),torque_limit);
                    
                    
                otherwise
                    error('You have your controller configured to an invalid type!!\n')
            end
            
            u = [u2; u3];
        end
    end
end

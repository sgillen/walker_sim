classdef CGTorsoController  < matlab.mixin.Copyable
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
        PFL_CTYPE = 3; % PFL control, doesn't work very well in practice right now
        CSTEP_CTYPE = 4; %this means "careful step" it's a state machine, just take a look at the code
    end
    
    properties
        %default values, 
        kp2=400;
        kd2=40;
        kp3=400;
        kd3=40;
        th3_ref = 45*pi/180; % absolute angle, wrt x axis, measured CCW
        th2_ref = ((360 - 60)*pi)/180; %absolute or relative depending on which controller you choose 
        Ctype = 2; % PD control about the absolute angle
        
        cont = 0; %tells the simulator if the controller is done yet, which will stop the simulation
        step_num = 0; %lets the simulator tell the controller how many steps it has taken, on a real robot this would be handled by a contact sensor basically. 
        
    end
    
    methods
      %% constructor, you can either pass params or use the default values
        function obj = CGTorsoController(Params)
           % ifjust use default values as defined in the params
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
                    u2 = obj.kp2*(obj.th2_ref - th2_rel) + obj.kd2*(0 - dth2_rel);
                    u3 = obj.kp3*(obj.th3_ref - th3_abs) + obj.kd3*(0 - dth3_abs);
                    
                case obj.PD_ABSWING_CTYPE
                    % Combine states to define parameters to be directly controlled:
                    % TODO, unwrap our angles
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_abs = th1+th2;
                    dth2_abs = dth1+dth2;
                    
                    % Below is the simple PD control law
                    u2 = obj.kp2*(obj.th2_ref - th2_abs) + obj.kd2*(0 - dth2_abs);
                    u3 = obj.kp3*(obj.th3_ref - th3_abs) + obj.kd3*(0 - dth3_abs);
                    
                    
                case obj.PFL_CTYPE
                    %Partial Feedback linearization, as found in
                    %walker2_noncollocated
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_abs = th1+th2;
                    dth2_abs = dth1+dth2;
                    
                    u3 = 0;
                    u2 = M(2,2).^(-1).*(M(1,2)+M(3,3)).^(-1).*((-1).*M(2,2).*M(3,3).*G(1)+M(1,2).*M(3,3).*G(2)+M(1,2).*M(2,2).*G(3)+(-1).*M(1,2).*M(2,2).*TAU(2)+M(1,2).*M(3,3).*TAU(2)+C(3,1).*M(1,2).*M(2,2).*(dth1))+C(3,1).*M(1,2).*M(3,3).*(dth1)+M(1,2).*M(2,2).*M(1,3).*(obj.kp2.*(obj.th2_ref+(-1).*th1)+(-1).*obj.kd2.*(dth1))+M(1,2).*M(1,2).*M(3,3).*(obj.kp2.*(obj.th2_ref+(-1).*th1)+(-1).*obj.kd2.*(dth1))+(-1).*d11.*M(2,2).*M(3,3).*(obj.kp2.*(obj.th2_ref+(-1).*th1)+(-1).*obj.kd2.*(dth1))+(-1).*c12.*M(2,2).*M(3,3).*(dth2_abs)+(-1).*C(1,3).*M(2,2).*M(3,3).*(dth3_abs);
                    
                    torque_limit = 10000;  % [Nm] limit in torque magnitude
                    u2 = sign(u2)*min(abs(u2),torque_limit);
                    
                %just testing this out first
                case obj.CSTEP_CTYPE
                    th3_abs = th1+th3;
                    dth3_abs = dth1+dth3;
                    th2_abs = th1+th2;
                    dth2_abs = dth1+dth2;
                    
                  
                    if(obj.step_num == 0)
                    
                        %this is hard coded becuase I am a terrible
                        %programmer
                        u2 = 800*(5.9341 - th2_abs) + 0; 
                        u3 = obj.kp3*(obj.th3_ref - th3_abs) + obj.kd3*(0 - dth3_abs);
                        
                        obj.cont = 1;
                        
                    else            
                        % Below is the simple PD control law
                        u2 = obj.kp2*(obj.th2_ref - th2_abs) + obj.kd2*(0 - dth2_abs);
                        u3 = obj.kp3*(obj.th3_ref - th3_abs) + obj.kd3*(0 - dth3_abs);
                        
                        obj.cont = 0;
                    end
                    
                otherwise
                    error('You have your controller configured to an invalid type!!\n')
            end
            
            
            u = [u2; u3];
        end
    end
end

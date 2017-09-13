classdef CGTorsoWalker
    %% This class
    
    properties
        %these are all DEFAULT values, if you pass in a param variable to the
        %constructor these will be overridden, if you prefer you can also use
        %the defualts and then set values you want to change peicemeal
        
        g = 9.81; % gravity
        
        m1 = 10; m2 = 10; m3 = 10;  %total mass for each joint, center of mass defined by Lxc vars
        L1 = 1;  L2 = 1;  L3 = 0.8; %length for each joint
        L1c = 0.5; L2c = 0.5; L3c = 0.4; % location of each joints center of mass wrt hip joint (going down each leg)
        J1 = 10*0.16; J2 = 10*0.16; J3 = 10*.25 % m*r^2, approx guess, J3 to be played with
        
        %this is a controller object to passed in to serve as the walkers controller
        controller
        
        %enviroment/location properties
        xy_stance = [0,0]; %this is the xy coordinate of our stance leg, we need to keep track of it to walk "forward" in the simulation
        xy_step = [0,0]; %this is the xy coordinate of our step. y is the step height, positve values is a step up and negative values a step down
        
        
        
    end
    
    
    methods
        %% Constructor
        function obj = CGTorsoWalker(controller)
            if nargin > 0
                obj.controller = controller;
                
%                 obj.g = params.g; % gravity
%                 
%                 obj.m1  = params.m1;   obj.m2  = params.m2;   obj.m3  = params.m3;
%                 obj.L1  = params.L1;   obj.L2  = params.L2;   obj.L3  = params.L3;
%                 obj.L1c  = params.L1c; obj.L2c  = params.L2c; obj.L3c  = params.L3c;
%                 obj.J1  = params.J1;   obj.J2  = params.J2;   obj.J3  = params.J3;
                
                
            else % use the defualt controller constructor, the mass/geometric properties already have default values
                obj.controller = CGTorsoController()
            end
            
            
        end
        
        %% Walker ODE, this is the function we will pass to ode45 (or whichever solver we choose)
        function [dX,u] = walkerODE(obj,t,X)
            
            th1 = X(1);
            th2 = X(2);
            th3 = X(3);
            dth1 = X(4);
            dth2 = X(5);
            dth3 = X(6);
            
            
            %Inertia matrix (M) and conservative torque terms (C)
            %sgillen - may be able to save some time by not computing non theta dependent values
            %everyime, but probably not worthwhile.
            
            M11 = obj.J1 + obj.J2 + obj.J3 + obj.L1^2*obj.m1 + obj.L1^2*obj.m2 + obj.L1^2*obj.m3 + obj.L1c^2*obj.m1 + obj.L1c^2*obj.m2 + obj.L3c^2*obj.m3 - 2*obj.L1*obj.L1c*obj.m1 + 2*obj.L1*obj.L1c*obj.m2*cos(th2) + 2*obj.L1*obj.L3c*obj.m3*cos(th3);
            M12 = obj.J2 + obj.L1c^2*obj.m2 + obj.L1*obj.L1c*obj.m2*cos(th2);
            M13 = obj.J3 + obj.L3c^2*obj.m3 + obj.L1*obj.L3c*obj.m3*cos(th3);
            M21 = obj.J2 + obj.L1c^2*obj.m2 + obj.L1*obj.L1c*obj.m2*cos(th2);
            M22 = obj.J2 + obj.L1c^2*obj.m2;
            M23 = 0;
            M31 = obj.J3 + obj.L3c^2*obj.m3 + obj.L1*obj.L3c*obj.m3*cos(th3);
            M32 = 0;
            M33 = obj.J3 + obj.L3c^2*obj.m3;
            C1 = obj.L1c*obj.g*obj.m2*cos(th1 + th2) + obj.L3c*obj.g*obj.m3*cos(th1 + th3) + obj.L1*obj.g*obj.m1*cos(th1) + obj.L1*obj.g*obj.m2*cos(th1) + obj.L1*obj.g*obj.m3*cos(th1) - obj.L1c*obj.g*obj.m1*cos(th1) - obj.L1*obj.L1c*dth2^2*obj.m2*sin(th2) - obj.L1*obj.L3c*dth3^2*obj.m3*sin(th3) - 2*obj.L1*obj.L1c*dth1*dth2*obj.m2*sin(th2) - 2*obj.L1*obj.L3c*dth1*dth3*obj.m3*sin(th3);
            C2 = obj.L1c*obj.g*obj.m2*cos(th1 + th2) + obj.L1*obj.L1c*dth1^2*obj.m2*sin(th2);
            C3 = obj.L3c*obj.g*obj.m3*cos(th1 + th3) + obj.L1*obj.L3c*dth1^2*obj.m3*sin(th3);
            
            M = [M11, M12, M13; M21, M22, M23; M31, M32, M33];
            C = [C1; C2; C3];
            
            % M*d2th + C = Xi, where Xi are the non-conservative torques, i.e.,
            % Xi = [0; tau2; tau3].
            % Let u = [tau2; tau3], and Xi = [0 0; 1 0; 0 1]*u =
            % So, dX = AX + Bu formulation yields B = [zeros(3,2); inv(M)*[0 0;1 0;0 1]
            
            u = obj.controller.calculate_control_efforts(X);
            
            umat = [0 0; 1 0; 0 1]; % Which EOMs does u affect?
            d2th = M \ (-C + umat*u);
            dth = X(4:6); % velocity states, in order to match positions...
            dX = [dth; d2th];
        end
        
        %% TODO
        function obj = runSim(obj,t,X0)
            %TODO basically just call ode45 
            [~,~] = ode45(@(tt,xx)obj.walkerODE(tt,xx), t, X0);
            
        end
        
        %% Fall Event, we pass this to ode45 , it helps us terminate early so we don't waste a ton of time if the walker falls down
        function [value,isterminal,direction] = fallEvent(t,y)
            %tol is how far below zero we will allow a leg to go before we consider it below the horizontal
            tol = .2;
            
            yh = P.L1*sin(y(1));
            y3 = yh + P.L3*sin(y(3) + y(1));
            
            value = max(0,min([yh+tol,y3+tol])); %basically this call returns 0 if any of yh y2 y3 is less then 0
            isterminal = 1;        % Stop the integration
            direction = 0;         % All direction
        end
        
        %% Step event, this is the same as the fall event but we look at the swing leg (which is supposed to impact the ground)
        function [value,isterminal,direction] = stepEvent(t,y)
            %tol is how far below zero we will allow a leg to go before we consider it below the horizontal
            tol = .05;
            
            y2 = obj.L1*sin(y(1)) + obj.L1*sin(y(2) + y(1));
         
            value = max(0,y2); %basically this call returns 0 if any of yh y2 y3 is less then 0
            isterminal = 1;        % Stop the integration
            direction = 0;         % All direction
        end
        
        %% Detect Collisions (still think maybe we should do this inside the ODE?)
        function [thit,Xhit,xy_end] = cg_torso_animate(tout,xout,xy_start, bDraw ,xy_wall)
            
            thit = []; Xhit = []; % stays empty, if not "hit" with ground is detected
            xy_end = [0,0]; %this IS assigned at the end
            
            if ~exist('bDraw','var')
                bDraw = true;
            end
            if ~exist('xy_start','var')
                xy_start = [0,0];
            end
            
            %xy_wall tells us where our step up starts and how high it is, negative y
            %component is a step down
            if ~exist('xy_wall','var')
                xy_wall = [0,0];
            end
            
            
            % Below, absolute angles
            th1a = xout(:,1);
            th2a = xout(:,1)+xout(:,2);
            th3a = xout(:,1)+xout(:,3);
            x0 = xy_start(1);
            y0 = xy_start(2);
            
            xw = xy_wall(1);
            yw = xy_wall(2);
            
            % delgo is amt past stance toe, for xhit checks
            % may want to pass this in as well
            delgo = 1e-3;
            
            % Look up parameters from a separate (single) file
            P = cg_torso_params;
            %J1=P.J1; J2=P.J2; J3=P.J3;
            L1=P.L1; L1c=P.L1c; L3c=P.L3c;
            L3 = P.L3;
            %m1=P.m1; m2=P.m2; m3=P.m3;
            %g = P.g; % gravity
            
            % look and draw...
            dt = (1/25);
            tu = 0:dt:max(tout);
            bDidHit = false;
            
            %sgillen - still not entirely sure why an iterpolation approach was used..
            % not thoroughly convinced it saves computation, it does sort of reduce
            % accuracy.
            for n=1:length(tu);
                t1 = interp1(tout,th1a,tu(n));
                t2 = interp1(tout,th2a,tu(n));
                t3 = interp1(tout,th3a,tu(n));
                xh = x0+L1*cos(t1);
                yh = y0+L1*sin(t1);
                xe = xh+L1*cos(t2);
                ye = yh+L1*sin(t2);
                
                
                %check if the xcoordinate for the end of our swing leg is past the wall
                %if it it we need to check if our y is past the step height otherwise
                %we are looking at
                if xe > xw
                    yg = yw;
                else
                    yg = y0;
                end
                
                if xe>(x0+delgo) && ye<=yg && n>1
                    %keyboard
                    % Check preview time
                    t1m = interp1(tout,th1a,tu(n-1));
                    t2m = interp1(tout,th2a,tu(n-1));
                    t3m = interp1(tout,th3a,tu(n-1));
                    xhm = x0+L1*cos(t1m);
                    yhm = x0+L1*sin(t1m);
                    xem = xhm+L1*cos(t2m);
                    yem = yhm+L1*sin(t2m);
                    if yem>yg
                        
                        bDidHit = true;
                        nu = n+[-1 0];
                        thit = interp1([yem, ye],[tu(nu)],yg);
                        Xhit = zeros(6,1);
                        for n2=1:6
                            Xhit(n2) = interp1(tout,xout(:,n2),thit);
                        end
                        %keyboard
                        if bDraw
                            title('HIT DETECTED!');
                            %fprintf("tout = %i\n" ,tout);
                        end
                        return
                    end
                    
                end
                %end
                
                if bDraw
                    figure(11); clf
                    xt = xh+L3*cos(t3);
                    yt = yh+L3*sin(t3);
                    
                    p1 = plot([x0 xh],[y0 yh],'b-','LineWidth',3); hold on
                    p2 = plot([xh xe],[yh ye],'r-','LineWidth',3);
                    p3 = plot([xh xt],[yh yt],'k-','LineWidth',3);
                    
                    plot(0+[-10 xw],0+[0 0],'k-','LineWidth',1);
                    plot(xw+[0 0], 0+[0 yw],'k-','LineWidth',1);
                    plot(xw+[0 10],yw+[0 0],'k-','LineWidth',1);
                    
                    
                    axis image
                    axis([xh+[-2 2],0+[-.2 2]])
                    
                    drawnow
                    pause(dt*1)
                end
            end
        end
    end
end
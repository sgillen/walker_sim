 classdef CGWalker < matlab.mixin.Copyable
    %% This class encapsulates all the variables and functions needed to run a simulation for a compass gait walker with torso
    %  Sean Gillen 9/14/17
    %
    % 
    % you might be better off just taking a look at cg_walker_sim but I'll provide some basic usage here.
    %
    % generally the workflow I've been using is to first create a walker
    % object with the default values:
    %
    % walker = CGTorsoWalker();
    %
    % and then change any values that you want:
    %
    % walker.L3 = 1;
    % walker.xy_step = [.3,.1]
    %
    % see the properties section for all the stuff you can change
    % note if you want to change the default controller you have to create
    % a controller object first, see CGTorsoController for that.
    %
    % next we run a single step with:
    %
    % [Xnext, flag] = walker.runSim(Xinit)
    %
    % Xinit needs to be a collumn vector for the initial state of all the
    % state variables. 
    %
    % Xnext is the stance of the robot after impacting the ground, flag
    % tells us if we think the robot took a step or fell over, see the code
    % for rumSim for how to interpret those
    %
    % you can now find the results saved in walker.X and walker.t. let's
    % animate it and see how it looked
    %
    %
    % walker.cgTorsoAnimate();
    %
    % we can also try to find the limit cycle for our current set of
    % parameters
    %
    % eival = walker.cgFindLimitCycle
    %
    
    properties
        % these are all DEFAULT values, there's no non default constructor
        % I use the defaults and then set values you want to change piecemeal
        % I found this makes everyone's code more concise than using a
        % "real" constructor
       
        
        g = 9.81; % gravity
        
        m1 = 10; m2 = 10; %total mass for each joint, center of mass defined by Lxc vars
        L1 = 1;  L2 = 1; %length for each joint
        L1c = 0.5; L2c = 0.5;  % location of each joints center of mass wrt hip joint (going down each leg)
        J1 = 10*0.16; J2 = 10*0.16;  
        
        
        xy_start = {[0,0]}; %xy coordinate our stance starts at 
        xy_end = {[0,0]};   %xt cooridnate our stance leg ended up at for the next step
        xy_step = [0,0]; %this is the xy coordinate of our step. y is the step height, positve values is a step up and negative a step down
       
        
        Xinit =[1.9051; 2.4725; -1.2174; 0.5065;]; %state vars at the start of our simulation,  If you call findLimitCycle this will be updated (so usually you want to use that first). default pulled from katie's initial code.
        Tmax  = 3; % maximum length to sim one step before giving up
        
        
        X; %state vars after our simulation (starts as our initial value)
        t = 0; %time vector associated with the state vars
        
        step_num = 1; %how many steps forward have we taken (correpsonds to how many times we call takeStep()
        Xhist = {}; %keeps track of previous steps taken, updated everytime we call takeStep(). This is a cell array, each entry in the array the the X from that indexes step
        thist = {}; %same as Xhist but with t. 
        
        
        %used by the limit cycle function
        eival; %eigen value of the Jacobian at the fixed point we find
        Xfixed; %fixed point of the limit cycle, (or a guess at one I guess)
        Xerr;  %difference between the fixed point and where the sim tells us we end up after starting from the 
        
        bias; %initial DC bias
        noise_const;%sort of the variance for the randn that we use to generate the waveform
        noise = []; %this is the noise at each time point, computed beforehand
        noise_t = [];%time vectore corresponding to the noise var
        
        animation_pause = 1e-6; %seconds to pause between frames when animating  
        
        %default values,
        kp=400;
        kd=40;
        th2_ref = ((360 - 60)*pi)/180; %absolute or relative depending on which controller you choose
        
    end
    
    methods
        %% Constructor
        function obj = CGWalker() 
            obj.X = obj.Xinit; % we start at our initial state...

        end   

        
        %this function will reset the step height, xy_start, xy_end, and
        %Xinit
        function reset(obj)
            obj.xy_start = {[0,0]};
            obj.xy_end = {[0, 0]};
            %obj.xy_step = [0,0];
           
            obj.step_num = 1;
                        
            obj.Xhist = {};
            obj.thist = {};
            
            
            
            
            
        end
        
        function [xy_h, xy_e xy_t] = getXY(obj, X, xy_start)
           
            
            xh = xy_start(1) + obj.L1*cos(X(1));
            yh = xy_start(2) + obj.L1*sin(X(1));
            
            xe = xh+obj.L1*cos(X(2) + X(1));
            ye = yh+obj.L1*sin(X(2) + X(1));
    
            xy_h = [xh, yh];
            xy_e = [xe, ye];

            
            
        end
        

            
            
            
        % this will add noise to the measurement of our th1, this is meant to simulate IMU error
        % if you want to add a constant bias set noise_const to zero and
        % bias to whatever you want.
        
     
        function initSensorNoise(obj, seed, bias, noise_const)
          
            %record this value for later
            obj.bias = bias;
            
            %could be messed with, could make is a property but don't see my
            %self messing it once I find a value I like
            dt = .05;
            obj.noise_t = 0:dt:obj.Tmax;
            
            
            %generate our random waveform
            obj.noise = zeros(1,length(obj.noise_t));
            obj.noise(1) = bias + noise_const*randn;
            for i = 2:length(obj.noise_t)
                obj.noise(i) = obj.noise(i-1) + noise_const*randn;
            end
            
        end
        
   
        %% Run Sim functions
        % Run Simulation and update our foot position so we can step forward through the enviroment
        
        function [Xnext, flag] = takeStep(obj, Xinit)
            
            if nargin < 2
                Xinit = obj.Xinit;
            end
            
            Xnext = Xinit;
            
            %TODO probably pass in options, or better yet have them be additonal parmaters
            options = odeset('AbsTol',1e-4, 'Events' , @(t,y)obj.collisionEvent(t,y)); %,'RelTol',1e-8);
            
            %{
            t and X are the normal solutions to the ODE, te and xe are the
            %time and values for the events that occured (see
            %collisionEvent for more info on that) ie tell us WHICH event
            %occured
            %ie == 1  -> step event
            %ie == 2  -> fall event
            %~ie      -> timeout
            %}
            
            
            obj.xy_start{obj.step_num} = obj.xy_end{obj.step_num};
            
            [t,X,te,xe,flag] = ode45(@(tt,xx)obj.walkerODE(tt,xx), [obj.t(end) obj.t(end) + obj.Tmax], Xnext, options);
            
            obj.Xhist{obj.step_num} = X; %even if we fall we want to see what it looked like
            obj.thist{obj.step_num} = t;
            
            
            if flag == 1 %if we took a step
                Xnext=obj.detectCollision(t,X); %can also get timpact from this..
                obj.Xinit = Xnext;
                
                obj.step_num = obj.step_num + 1;
                
                obj.xy_end{obj.step_num}(1) = obj.xy_start{obj.step_num-1}(1) + (obj.L1*cos(X(end,1)) + obj.L2*cos(X(end,1) + X(end,2)));
                obj.xy_end{obj.step_num}(2) = obj.xy_start{obj.step_num-1}(2) + (obj.L1*sin(X(end,1)) + obj.L2*sin(X(end,1) + X(end,2)));
                
                %obj.xy_end{obj.step_num}(1) = obj.xy_start{obj.step_num-1}(1) + (obj.L1*cos(Xnext(1)) + obj.L2*cos(Xnext(2) + Xnext(1)));
                %obj.xy_end{obj.step_num}(2) = obj.xy_start{obj.step_num-1}(2) + (obj.L1*sin(Xnext(1)) + obj.L2*sin(Xnext(2) + Xnext(1)));
                
            else %if we fell or timed out
                %Xnext = X(end,:).^2'.*1e12; %this is here to discourage the optimizer from choosing solutions where we fall down.
                Xnext = NaN;
                %obj.step_num = obj.step_num - 1;
                return %might need to be changed
            end
           
            obj.Xinit = Xnext;

            
        end
        
        
        
        
       % Run Simulation, but keep us in the same spot for the next run
        function [Xnext, flag] = runSim(obj, Xinit)
            
            orig_step_num = obj.step_num;
            
            if nargin < 2
                Xinit = obj.Xinit;
            end
            
            %obj.xy_start{obj.step_num} = obj.xy_end{obj.step_num};
            
            [Xnext, flag] = obj.takeStep(Xinit);
     
            obj.step_num = orig_step_num;

            

        end
        
        
        %% Collision detection functions
        
        % Collision event functions, we pass this to ode45 , it helps us terminate early so we don't waste a ton of time if the walker falls down
        function [value,isterminal,direction] = collisionEvent(obj,t,y)
            %tol is how far below zero we will allow a leg to go before we consider it below the horizontal
            tol = .01;
            
            %this is where the walker started
            x0 = obj.xy_start{obj.step_num}(1);
            y0 = obj.xy_start{obj.step_num}(2);
            
            %these are the x and y coords of the step
            xw = obj.xy_step(1);
            yw = obj.xy_step(2);
            
            %first calculate all the x and y coordinates doe the hip and
            %end
            yh = y0 + obj.L1*sin(y(1));
            y2 = yh + obj.L1*sin(y(2) + y(1));
            
            xh = x0 + obj.L1*cos(y(1));
            x2 = xh + obj.L1*cos(y(2) + y(1));
            
            %if we are passed the step we need to check the step height ,
            %otherwise we check for the initial height
            if x2 > xw
                yg = yw;
            else
                yg = y0;
            end
            
            % delgo is amt past stance toe, for step checks
            % may want to pass this in as well
            delgo = .3;
            
            %check if we have passed the swing leg and have hit the ground
            if x2>(x0+delgo)
                step_value = y2 - yg;
            else
                step_value = -1;
            end

            fall_value = max(0,min([yh+tol-yg])); %basically this call returns 0 if of yh or y3 is less then 0

      
            
            %if either of the values are 0 then that tells the ode to stop
            value = [step_value, fall_value]; 
        
            %these two don't matter too much, isterminal tells us we need
            isterminal = [1, 1];       % Stop the integration
            direction  =  [-1, 0];         % All direction
        end     
        
        % detectCollision, this we run on the output of our call to ode45,
        % it interpolates back to the "exact" point we hit the ground
        
        function [Xnext, timpact] = detectCollision(obj,t,X)
            
            %we interpolate back to the exact moment we hit zero 
            
            x0 = obj.xy_start{obj.step_num}(1);
            xw = obj.xy_step(1);
            
            xh = x0 + obj.L1*cos(X(end,1));
            x2 = xh + obj.L1*cos(X(end,2) + X(end,1));
            
            %if we are passed the step we need to check the step height ,
            %otherwise we check for the initial height
            
            
            y0 = obj.xy_start{obj.step_num}(2);
            yw = obj.xy_step(2);
            
            if x2 > xw
                yg = yw;
            else
                yg = y0;
            end

            y2_f = y0 + obj.L1*sin(X(end,1)) + obj.L2*sin(X(end,2) + X(end,1));
            y2_p = y0 + obj.L1*sin(X(end-1,1)) +  obj.L2*sin(X(end-1,2) + X(end-1,1));
            
            timpact = nakeinterp1([y2_p; y2_f],[t(end-1); t(end)], yg);
            
            Ximpact = zeros(4,1);
            
            %this can be vectorized
            for i = 1:length(X(end,:))
                Ximpact(i) = nakeinterp1([t(end-1); t(end)], [X(end-1,i); X(end,i)], timpact);
            end
            
            Xnext = obj.cgTorsoImpact(Ximpact);
            
            %if we never actually CROSSED zero (like we started in a
            %fallen state, then the interp1s will return NaN, which can
            %sometimes screw us up
            
            if isnan(Xnext)
                Xnext = zeros(6,1);
            end
            
        end
        
        %% ODE and Impact equation, this is where most of the math is
        
        % Walker ODE, this is the function we will pass to ode45 (or whichever solver we choose)
        function [dX,u] = walkerODE(obj,t,X)
      
            %noise_test = obj.noise
            
            if ~isempty(obj.noise)
                noise = nakeinterp1(obj.noise_t', obj.noise',t);
            else
                noise = 0;
            end
            
             
            %we can remove these, but I think it makes the code more
            %readable and the performance hit is neglible (if not optimized
            %away entirely) 
            
            th1 = X(1);
            th2 = X(2);
            dth1 = X(3);
            dth2 = X(4);
            
            %Inertia matrix (M) and conservative torque terms (C)
            %may be able to save some time by not computing non theta dependent values
            %everyime, but probably not worthwhile.  
            % lc2*m2*sin(th1 - th2)*dth2^2*l1 + d2th1*m2*l1^2 + d2th2*lc2*m2*cos(th1 - th2)*l1 + d2th1*m1*lc1^2 - g*m1*cos(th1)*lc1 + J1*d2th1
            %eq2 = J2*d2th2 + d2th2*lc2^2*m2 - g*lc2*m2*cos(th2) + d2th1*l1*lc2*m2*cos(th1 - th2) - dth1^2*l1*lc2*m2*sin(th1 - th2)

            M11 = obj.J1 + obj.m2*obj.L1^2 + obj.m1*obj.m1*obj.L1c^2;
            M12 = obj.L2c*obj.m2*cos(th1 - th2)*obj.L1 ;
            M21 = obj.L1*obj.L2c*obj.m2*cos(th1 - th2);
            M22 = obj.J2 + obj.L2c^2*obj.m2; 
            
            C1 = obj.g*obj.m1*cos(th1)*obj.L1c + obj.L2c*obj.m2*sin(th1 - th2)*dth2^2*obj.L1 ;
            C2 = obj.g*obj.L2c*obj.m2*cos(th2) +  dth1^2*obj.L1*obj.L2c*obj.m2*sin(th1 - th2);
            
            
            M = [M11, M12; M21, M22];
            C = [C1; C2];
            
            % M*d2th + C = Xi, where Xi are the non-conservative torques, i.e.,
            % Xi = [0; tau2; tau3].
            % Let u = [tau2; tau3], and Xi = [0 0; 1 0; 0 1]*u =
            % So, dX = AX + Bu formulation yields B = [zeros(3,2); inv(M)*[0 0;1 0;0 1]
            
          
            
            % Combine states to define parameters to be directly controlled:
            % TODO, unwrap our angles
            
            th1_cont = th1 + noise; %theta that the controller sees %should I add noise to the derivative too?
            
            th2_abs = th1_cont+th2;
            dth2_abs = dth1+dth2;
            
            % Below is the simple PD control law
            u = obj.kp*(obj.th2_ref - th2_abs) + obj.kd*(0 - dth2_abs);
            
           
            umat = [0; 1]; % Which EOMs does u affect?
            d2th = M \ (-C + umat*u);
%            if(rcond(M) < 1e-15)
%                d2th
%            end
            dth = X(3:4); % velocity states, in order to match positions...
            dX = [dth; d2th];
            
        end 
         % Impact equation, this tells us where our legs are after an impact with the ground. it also switches our stance and swing leg for us
        function Xplus = cgTorsoImpact(obj,Xminus)
            % Katie Byl, UCSB, 7/17/17            
            
            th1 = Xminus(1);
            th2 = Xminus(2);
 
            dth_minus = Xminus(3:4); % pre-impact Angular Velocities
            Qm = [ obj.J1 + obj.J2 + obj.L1c^2*obj.m1 + obj.L1c^2*obj.m2  - obj.L1^2*obj.m1*cos(th2) - obj.L1^2*obj.m2*cos(th2)  - obj.L1*obj.L1c*obj.m1 - obj.L1*obj.L1c*obj.m2 + obj.L1*obj.L1c*obj.m1*cos(th2) + obj.L1*obj.L1c*obj.m2*cos(th2)  ,obj.m2*obj.L1c^2 - obj.L1*obj.m2*obj.L1c + obj.J2
                obj.m1*obj.L1c^2 - obj.L1*obj.m1*obj.L1c + obj.J1,                         0];
            % Now, update "meanings" of all angles, to match post-impact situation.
            % THEN, evaluate Qplus (Qp):
            
            % After impact,
            % 1) th1p = th1m + th2m + pi  -> absolute angle from hip,
            % and then "-pi" to take opposite angle, wrt new toe (instead of wrt
            % hip joint)
            % 2) th2p = (th1m+pi) - th1p = (th1m+pi) - th1m - th2m + pi = -th2m
            % i.e., th2p = -th2m, since 2*pi = 0
            % 3) th3p = (th1m+th3m) - th1p = th1m + th3m - th1m - th2m + pi
            % th3p = th3m - th2m + pi
            
            %Am = [1 1 0; 0 -1 0; 0 -1 1];
            %Bm = [pi; 0; pi];  % [almost, but need 2*pi "tweaks" -- see below
            
            % 1) th1p = (th1m+th2m) - pi
            % 2) th2p = 2*pi - th2m
            % 3) th3p = th1m + th2m + th3m - 2*pi
            Am = [1 1; 0 -1;];
            Bm = [-pi; 2*pi];
            
            th_minus = [th1;th2;];
            th_plus = Am*th_minus + Bm;
            th1 = th_plus(1); th2 = th_plus(2); 
            
            
            Qp = [ obj.J1 + obj.J2  + obj.L1^2*obj.m1 + obj.L1^2*obj.m2 + obj.L1c^2*obj.m1 + obj.L1c^2*obj.m2  - 2*obj.L1*obj.L1c*obj.m1 + 2*obj.L1*obj.L1c*obj.m2*cos(th2) , obj.m2*obj.L1c^2 + obj.L1*obj.m2*cos(th2)*obj.L1c + obj.J2
                obj.m2*obj.L1c^2 + obj.L1*obj.m2*cos(th2)*obj.L1c + obj.J2,                      obj.m2*obj.L1c^2 + obj.J2 ];
       
            
            dth_plus = Qp \ (Qm * dth_minus);
            %dth_plus = (Qp \ Qm) * dth_minus
            
            
            Xplus = [th_plus; dth_plus];

        end  
        
        
        
                
        function [c, ceq] = limitCycleCons(obj, X)
            [xy_h, xy_e] = obj.getXY(X,[0,0]); %could do step num, but fmincon is looking for zero anyway...
            ceq(1)= xy_e(2);
            %c = -xy_h(2);
            [Xnext, flag] = obj.runSim(X);

            if flag == 1
                ceq(2) = 0;
            else
                ceq(2) = 1;
            end
            
            c = [];
        end
            
            
        
        function [Xerr] = findLimitFcn(obj,X)
            [Xnext, flag] = obj.runSim(X);
            
            if flag == 1;
               Xerr = norm(Xnext - X);
            else 
               Xerr = norm(X)^2;
            end
                
        end
                
            
            
            
            
            
     
         %% Find Limit cycle, this used runSim to find a limit cycle for the walker with it's current configuration
        function [eival, Xfixed,flag] = findLimitCycle(obj)
      
            options = optimoptions('fmincon');
            %options = optimoptions('lsqnonlin');
            %options = optimoptions('fminunc');

            
            %options = optimoptions(options, 'Algorithm' , 'sqp');
            
            options = optimoptions(options, 'OptimalityTolerance', 1e-4);
            
            % Set the Display option to 'iter' and StepTolerance to 1e-
            options.Display = 'none';
            options.StepTolerance = 1e-4;
            options.MaxFunctionEvaluations = 300;
            
            %Can use either "fmincon" or "lsqnonlin" -- or another fn
            
            
            
            lb = [-2*pi, -2*pi, -10, -10];
            ub = [2*pi, 2*pi, 10, 10];
            [Xfixed, Xerr2, flag] = fmincon(@(X)obj.findLimitFcn(X),obj.Xinit,[],[],[],[],lb,ub, @(X)obj.limitCycleCons(X),options); %,);
            %Xfixed = fmincon(@(X)1e2*norm(obj.runSim(X) - X),obj.Xinit,[],[],[],[],[],[],[],options); %,);
            %Xfixed = lsqnonlin(@(X)obj.findLimitFcn(X),obj.Xinit,[],[],[],[],[],[], @(X)obj.limitCycleCons(X),options); %,);
            %Xfixed = fminunc(@(X)1e2*norm(obj.runSim(X) - X), obj.Xinit,options);
            %catch
                        
            %end

            obj.Xfixed = Xfixed;          
           % Xerr = max(abs(Xfixed - obj.runSim(Xfixed)));
            
           % Xerr
           % Xerr2
            
            damt = 1e-4;
            J = zeros(6,6);
            
            for n=1:6
                d = zeros(6,1); d(n)=damt;
                xtemp = obj.runSim(Xfixed + d);
                xtemp2 = obj.runSim(Xfixed - d);
                
                J(:,n) = (1/(2*damt))*(xtemp-xtemp2);
            end
            
            [eivec,eival] = eig(J);
            obj.eival = diag(eival);
            eival = diag(eival);
        end
        
        %% animate the walker
        
        function animate(obj, t_list, x_list, xy_list)
           if nargin < 3
               t_list = obj.thist;
               x_list = obj.Xhist;
               xy_list = obj.xy_start;
           end
 
           for i = 1:size(x_list,2) 
               obj.animateStep(t_list{i}, x_list{i}, xy_list{i})     
           end
           
        end
        
        function animateStep(obj, tout,xout,xy_start)
                   
            if nargin < 4
               tout = obj.t;
               xout = obj.X;
               xy_start = obj.xy_start{end};
            end
            
            % Below, absolute angles
            th1a = xout(:,1);
            th2a = xout(:,1)+xout(:,2);
            
            %intial location of stance leg
            x0 = xy_start(1);
            y0 = xy_start(2);
            
            %location and height of the step
            xw = obj.xy_step(1);
            yw = obj.xy_step(2);
            
            % look and draw...
            dt = (1/100);
            tu = 0:dt:max(tout);
            
            for n=1:length(tu);
                t1 = interp1(tout,th1a,tu(n));
                t2 = interp1(tout,th2a,tu(n));
                xh = x0+obj.L1*cos(t1);
                yh = y0+obj.L1*sin(t1);
                xe = xh+obj.L1*cos(t2);
                ye = yh+obj.L1*sin(t2);
                
                figure(11); clf
               
                
                
                p1 = plot([x0 xh],[y0 yh],'b-','LineWidth',3); hold on
                p2 = plot([xh xe],[yh ye],'r-','LineWidth',3);
                
                plot(0+[-10 xw],0+[0 0],'k-','LineWidth',1);
                plot(xw+[0 0], 0+[0 yw],'k-','LineWidth',1);
                plot(xw+[0 10],yw+[0 0],'k-','LineWidth',1);
                
                
                axis image
                axis([xh+[-2 2],0+[-2 2]])
                
                drawnow
                pause(obj.animation_pause)
             
            end
        end
        
        %animates a single frame..
        function animateFrame(obj,X,xy_start)
            % Below, absolute angles
            th1a = X(1);
            th2a = X(1)+X(2);
            
            %th1a = X(1);
            %th2a = X(2);
            %th3a = X(3);
            
            %intial location of stance leg
            x0 = xy_start(1);
            y0 = xy_start(2);
            
            %location and height of the step
            xw = obj.xy_step(1);
            yw = obj.xy_step(2);
            
            xh = x0+obj.L1*cos(th1a);
            yh = y0+obj.L1*sin(th1a);
            xe = xh+obj.L1*cos(th2a);
            ye = yh+obj.L1*sin(th2a);
            
            plot([x0 xh],[y0 yh],'b-','LineWidth',3); hold on
            plot([xh xe],[yh ye],'r-','LineWidth',3);
                   
            plot(0+[-10 xw],0+[0 0],'k-','LineWidth',1);
            plot(xw+[0 0], 0+[0 yw],'k-','LineWidth',1);
            plot(xw+[0 10],yw+[0 0],'k-','LineWidth',1);
        
            axis image
            axis([xh+[-2 2],0+[-2 2]])
            
        end
            
            
            
    end
end
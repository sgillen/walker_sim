 classdef CGTorsoWalker < matlab.mixin.Copyable
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
        
        m1 = 10; m2 = 10; m3 = 10;  %total mass for each joint, center of mass defined by Lxc vars
        L1 = 1;  L2 = 1;  L3 = 0.8; %length for each joint
        L1c = 0.5; L2c = 0.5; L3c = 0.4; % location of each joints center of mass wrt hip joint (going down each leg)
        J1 = 10*0.16; J2 = 10*0.16; J3 = 10*.25 % m*r^2, approx guess, J3 to be played with
        
        %this is a controller object to passed in to serve as the walkers controller
        controller;
       
        
        xy_start = {[0,0]}; %xy coordinate our stance starts at 
        xy_end = {[0,0]};   %xt cooridnate our stance leg ended up at for the next step
        xy_step = [0,0]; %this is the xy coordinate of our step. y is the step height, positve values is a step up and negative a step down
       
        
        Xinit =[1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184]; %state vars at the start of our simulation,  If you call findLimitCycle this will be updated (so usually you want to use that first). default pulled from katie's initial code.
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
        
        
        seed;      %seed for the rng 
        bias; %initial DC bias
        noise_const;%sort of the variance for the randn that we use to generate the waveform
        noise = []; %this is the noise at each time point, computed beforehand
        noise_t = [];%time vectore corresponding to the noise var
        
        animation_pause = 1e-6; %seconds to pause between frames when animating

        git_hash; %the current git hash, useful if you find one of these objects in old data (really this should just be stuck in the script that calls this class..)
  
        
    end
    
    methods
        %% Constructor
        function obj = CGTorsoWalker(controller)
            if nargin > 0 %if the user passed in something use it
                obj.controller = controller;
            else % otherwise the default controller constructor, the mass/geometric properties already have default values
                obj.controller = CGTorsoController();
            end
            
            obj.X = obj.Xinit % we start at our initial state...

            
            %this will get the current git hash (which tells you which
            %version of the code we are running) 
            [~,obj.git_hash] = system('git rev-parse HEAD');
       
        end    
        
        
        %this function will reset the step height, xy_start, xy_end, and
        %Xinit
        function reset(obj)
            obj.xy_start = {[0,0]};
            obj.xy_end = {[0, 0]};
            obj.xy_step = [0,0];
           
            obj.step_num = 1;
            obj.controller.step_num = 0;
            
            obj.Xinit =[1.9051; 2.4725; -0.8654; -1.2174; 0.5065; 0.2184];
            
            obj.Xhist = {};
            obj.thist = {};
            
            
                
            
            
        end
        % this will add noise to the measurement of our th1, this is meant to simulate IMU error
        % if you want to add a constant bias set noise_const to zero and
        % bias to whatever you want.
        
        %!!! warning this reseeds the global rng TODO: find a better
        %way to do this 
        function initSensorNoise(obj, seed, bias, noise_const)
            % seed is the seed for the rng , bias is how bad the error is
            % during the first step (you could also make this ranodom), the
            % noise_const is a const we multiply the output of randn by
            
            %The basic idea here is that we generate a predetermined waveform
            %And then add it to our th1 measurment.
           obj.seed = seed;
           rng(seed); 
           
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
        
        % Run Simulation, but keep us in the same spot for the next run
        function [Xnext, flag] = runSim(obj, Xinit)
            
            if nargin < 2
                Xinit = obj.Xinit;
            end
            %TODO probably pass in options, or better yet have them be additonal parmaters
            options = odeset('AbsTol',1e-8, 'Events' , @(t,y)obj.collisionEvent(t,y)); %,'RelTol',1e-8);
        
            %{
            t and X are the normal solutions to the ODE, te and xe are the
            %time and values for the events that occured (see
            %collisionEvent for more info on that) ie tell us WHICH event
            %occured
            %ie == 1  -> step event
            %ie == 2  -> fall event
            %~ie      -> timeout
            %}
            
            while true %we check obj.controller.cont every loop and break if it tells us we are done
           
                obj.xy_start{obj.step_num} = obj.xy_end{obj.step_num};
                
                [t,X,te,xe,flag] = ode45(@(tt,xx)obj.walkerODE(tt,xx), [obj.t(end) obj.t(end) + obj.Tmax], Xinit, options);
     
                
                obj.Xhist{obj.step_num} = X; %even if we fall we want to see what it looked like
                obj.thist{obj.step_num} = t;
                
                obj.step_num = obj.step_num + 1;
                obj.controller.step_num = obj.controller.step_num + 1;
            
                if flag == 1 %if we took a step
                    Xnext=obj.detectCollision(t,X); %can also get timpact from this..
                    Xinit = Xnext; 
                    
                    obj.xy_end{obj.step_num}(1) = obj.xy_start{obj.step_num-1}(1) + obj.L1*cos(X(end,1)) + obj.L2*cos(X(end,2) + X(end,1));
                    obj.xy_end{obj.step_num}(2) = obj.xy_start{obj.step_num-1}(2) + obj.L1*sin(X(end,1)) + obj.L2*sin(X(end,2) + X(end,1));
                    
                else %if we fell or timed out
                    Xnext = X(end,:)'.*100000; %this is here to discourage the optimizer from choosing solutions where we fall down.
                    obj.step_num = 1;
                    
                    return %might need to be changed
                end
                
               if ~obj.controller.cont 
                    break 
                end 
                
            end
            
            obj.step_num = 1;
            obj.controller.step_num = 1; 

        end 
        
        % Run Simulation and update our foot position so we can step forward through the enviroment
        function [Xnext, flag] = takeStep(obj, Xinit)
            
            if nargin < 2
                Xinit = obj.Xinit;
            end
            
            obj.xy_start{obj.step_num} = obj.xy_end{obj.step_num};
            
            [Xnext, flag] = obj.runSim(Xinit);
            
            
            
            if flag == 1
                
                obj.step_num = obj.step_num + 1;
                %obj.Xhist{obj.step_num} = obj.X;
                %obj.thist{obj.step_num} = obj.t;
           
                obj.Xinit = Xnext;
                
                %These lines got convoluted but all it's doing is
                %calculating where the next xy_end is. Xhist was updated in
                %our call to runSim.
                obj.xy_end{obj.step_num}(1) = obj.xy_start{obj.step_num-1}(1) + obj.L1*cos(obj.Xhist{obj.step_num}(end,1)) + obj.L2*cos(obj.Xhist{obj.step_num}(end,2) + obj.Xhist{obj.step_num}(end,1));
                obj.xy_end{obj.step_num}(2) = obj.xy_start{obj.step_num-1}(2) + obj.L1*sin(obj.Xhist{obj.step_num}(end,2)) + obj.L2*sin(obj.Xhist{obj.step_num}(end,2) + obj.Xhist{obj.step_num}(end,2));
            end
            
            

        end
        
        
        %% Collision detection functions
        
        % Collision event functions, we pass this to ode45 , it helps us terminate early so we don't waste a ton of time if the walker falls down
        function [value,isterminal,direction] = collisionEvent(obj,t,y)
            %tol is how far below zero we will allow a leg to go before we consider it below the horizontal
            tol = 0;
            
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
            y3 = yh + obj.L3*sin(y(3) + y(1));
            
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

            fall_value = max(0,min([yh+tol-y0,y3+tol-y0])); %basically this call returns 0 if of yh or y3 is less then 0

      
            
            %if either of the values are 0 then that tells the ode to stop
            value = [step_value, fall_value]; 
        
            %these two don't matter too much, isterminal tells us we need
            isterminal = [1, 1];       % Stop the integration
            direction  =  [-1, 0];         % All direction
        end     
        
        % detectCollision, this we run on the output of our call to ode45,
        % it interpolates back to the "exact" point we hit zero
        % I've found that usually X(end,:) is so close to the zero crossing
        % we find that this function does not really do anything. but it's
        % cheap to do and gives us peace of mind in cases where ODE45
        % overshoots the zero significantly
        function [Xnext, timpact] = detectCollision(obj,t,X)
            
            %we interpolate back to the exact moment we hit zero (I
            %have found this does not make much of a difference)
            y2_f = obj.L1*sin(X(end,1)) + obj.L2*sin(X(end,2) + X(end,1));
            y2_p = obj.L1*sin(X(end-1,1)) +  obj.L2*sin(X(end-1,2) + X(end-1,1));
            
            timpact = nakeinterp1([y2_p; y2_f],[t(end-1); t(end)], 0);
            
            Ximpact = zeros(6,1);
            
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
            th3 = X(3);
            dth1 = X(4);
            dth2 = X(5);
            dth3 = X(6);
            
            %Inertia matrix (M) and conservative torque terms (C)
            %may be able to save some time by not computing non theta dependent values
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
            
            %the controller object is created when we create the walker
            %object, the class can be found in GCTorsoController
            u = obj.controller.calculateControlEfforts([X(1) + noise; X(2:6)],M,C);
            
            umat = [0 0; 1 0; 0 1]; % Which EOMs does u affect?
            d2th = M \ (-C + umat*u);
            dth = X(4:6); % velocity states, in order to match positions...
            dX = [dth; d2th];
            
        end 
         % Impact equation, this tells us where our legs are after an impact with the ground. it also switches our stance and swing leg for us
        function Xplus = cgTorsoImpact(obj,Xminus)
            % Katie Byl, UCSB, 7/17/17            
            
            th1 = Xminus(1);
            th2 = Xminus(2);
            th3 = Xminus(3);
            dth_minus = Xminus(4:6); % pre-impact Angular Velocities
            Qm = [ obj.J1 + obj.J2 + obj.J3 + obj.L1c^2*obj.m1 + obj.L1c^2*obj.m2 + obj.L3c^2*obj.m3 - obj.L1^2*obj.m1*cos(th2) - obj.L1^2*obj.m2*cos(th2) - obj.L1^2*obj.m3*cos(th2) - obj.L1*obj.L1c*obj.m1 - obj.L1*obj.L1c*obj.m2 + obj.L1*obj.L1c*obj.m1*cos(th2) + obj.L1*obj.L1c*obj.m2*cos(th2) + obj.L1*obj.L3c*obj.m3*cos(th3) - obj.L1*obj.L3c*obj.m3*cos(th2 - th3), obj.m2*obj.L1c^2 - obj.L1*obj.m2*obj.L1c + obj.J2, obj.m3*obj.L3c^2 - obj.L1*obj.m3*cos(th2 - th3)*obj.L3c + obj.J3
                obj.m1*obj.L1c^2 - obj.L1*obj.m1*obj.L1c + obj.J1,                         0,                                        0
                obj.m3*obj.L3c^2 + obj.L1*obj.m3*cos(th3)*obj.L3c + obj.J3,                         0,                            obj.m3*obj.L3c^2 + obj.J3];
            
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
            Am = [1 1 0; 0 -1 0; 0 -1 1];
            Bm = [-pi; 2*pi; pi];
            
            th_minus = [th1;th2;th3];
            th_plus = Am*th_minus + Bm;
            th1 = th_plus(1); th2 = th_plus(2); th3 = th_plus(3);
            
            
            
            Qp = [ obj.J1 + obj.J2 + obj.J3 + obj.L1^2*obj.m1 + obj.L1^2*obj.m2 + obj.L1^2*obj.m3 + obj.L1c^2*obj.m1 + obj.L1c^2*obj.m2 + obj.L3c^2*obj.m3 - 2*obj.L1*obj.L1c*obj.m1 + 2*obj.L1*obj.L1c*obj.m2*cos(th2) + 2*obj.L1*obj.L3c*obj.m3*cos(th3), obj.m2*obj.L1c^2 + obj.L1*obj.m2*cos(th2)*obj.L1c + obj.J2, obj.m3*obj.L3c^2 + obj.L1*obj.m3*cos(th3)*obj.L3c + obj.J3
                obj.m2*obj.L1c^2 + obj.L1*obj.m2*cos(th2)*obj.L1c + obj.J2,                      obj.m2*obj.L1c^2 + obj.J2,                                  0
                obj.m3*obj.L3c^2 + obj.L1*obj.m3*cos(th3)*obj.L3c + obj.J3,                                  0,                      obj.m3*obj.L3c^2 + obj.J3];
            
            dth_plus = Qp \ (Qm * dth_minus);
            %dth_plus = (Qp \ Qm) * dth_minus
            
            
            Xplus = [th_plus; dth_plus];

        end  
     
         %% Find Limit cycle, this used runSim to find a limit cycle for the walker with it's current configuration
        function [eival] = findLimitCycle(obj)
      
            options = optimoptions('fmincon');
            %options = optimoptions('lsqnonlin');
            
            %options = optimoptions(options, 'Algorithm' , 'sqp');
            
            options = optimoptions(options, 'OptimalityTolerance', 1e-7);
            
            % Set the Display option to 'iter' and StepTolerance to 1e-
            %options.Display = 'iter';
            options.StepTolerance = 1e-7;
            options.MaxFunctionEvaluations = 1e4;
            
            %Can use either "fmincon" or "lsqnonlin" -- or another fn
            Xfixed = fmincon(@(X)1e2*norm(obj.runSim(X) - X),obj.Xinit,[],[],[],[],[],[],[],options); %,);
            %Xfixed = lsqnonlin(@(X)1e2*norm(obj.runSim(X) - X),obj.Xinit,[],[],options); %,[],[],[],[],[],[],[],options);
            %Xfixed = fminunc(@(X)1e2*norm(obj.runSim(X) - X), obj.Xinit,options);
            
            obj.Xfixed = Xfixed;
            
            obj.Xerr = max(abs(Xfixed - obj.runSim(Xfixed)));
            
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
            th3a = xout(:,1)+xout(:,3);
            
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
                t3 = interp1(tout,th3a,tu(n));
                xh = x0+obj.L1*cos(t1);
                yh = y0+obj.L1*sin(t1);
                xe = xh+obj.L1*cos(t2);
                ye = yh+obj.L1*sin(t2);
                
                figure(11); clf
                xt = xh+obj.L3*cos(t3);
                yt = yh+obj.L3*sin(t3);
                
                
                p1 = plot([x0 xh],[y0 yh],'b-','LineWidth',3); hold on
                p2 = plot([xh xe],[yh ye],'r-','LineWidth',3);
                p3 = plot([xh xt],[yh yt],'k-','LineWidth',3);
                
                plot(0+[-10 xw],0+[0 0],'k-','LineWidth',1);
                plot(xw+[0 0], 0+[0 yw],'k-','LineWidth',1);
                plot(xw+[0 10],yw+[0 0],'k-','LineWidth',1);
                
                
                axis image
                axis([xh+[-2 2],0+[-2 2]])
                
                drawnow
                pause(obj.animation_pause)
             
            end
        end
    end
end
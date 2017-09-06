clear all % clears persistent variables in cg_torso_controller.m
format compact


% sgillen, these were not doing anything when I got here by virtue of the
% hard coded xpost below
%{ 
th1 = 80*pi/180; %73*pi/180
th2 = 200*pi/180; %214*pi/180;
th3 = 10*pi/180;

Xpre = [th1; th2; th3; -1;2;1.5];

Xpost = cg_torso_impact(Xpre)
%}

%sgillen - not too sure where this Xpost came from, but seems to result in
%a cleaner first step.. so maybe empirically placed here?
Xpost = [ 1.9051
    2.4725
   -0.8654
   -1.2174
    0.5065
    0.2184];

Xinit = Xpost;

Tmax = 3;

xy_start = [0,0]; % where stance toe starts on the ground...
bDraw = true;
Xlist = Xinit; % list all post-impact states over time
%{
%% simulate 10 steps forward with default control values
for n=1:10
    [tout,xout] = ode45(@cg_torso_ode,[0 Tmax],Xinit);
    [thit,Xhit] = cg_torso_animate(tout,xout,xy_start,bDraw)
    
    %get the position we ended up at and start a new cycle there
    Xlist = [Xlist, cg_torso_impact(Xhit)];
    
    Xinit = Xlist(:,end);
    [tout,xout] = ode45(@cg_torso_ode,[0 Tmax],Xinit);
end

% Find a limit cycle, which may be either stable or unstable,
% and determine a numeric approximation of its Jacobian, to estimate
% eigenvalues of this limit cycle:
cg_torso_find_limit_cycle
%}


options = odeset('Events', @fall_event);


%% Now, redefine the CONTROL params, for PD controller:

for n = 0;
    n
    fprintf('Now, we will reset the control setpoints...\n')
    bReset = true;
    torso_abs_des = 40*pi/180; % wrt x axis...
    swing_rel_des = (180+30)*pi/180; % interleg angle (swing, wrt stance leg)
    wn = 30; zeta = 1;
    Params.Ctype = 1; % make sure this fits within cg_torso_controller.m
    Params.Kp2 = max(0,wn*wn + n*10);
    Params.Kd2 = max(0,2*zeta*wn + n);
    Params.Kp3 = max(0,wn*wn + n*10);
    Params.Kd3 = max(0,2*zeta*wn + n)
    Params.th3_ref = torso_abs_des;
    Params.th2_ref = swing_rel_des;

    %this call will actually update the parameters 
    cg_torso_controller([],[],bReset,Params);

    %simulate another 10 steps with our new controller
%     fell = false;
%     for n=1:10
%         [tout,xout] = ode45(@cg_torso_ode,[0 Tmax],Xinit,options);
%      
%         [thit,Xhit] = cg_torso_animate(tout,xout,xy_start,bDraw)
%         if tout(end) ~= Tmax
%             fprintf("ode terminated early!\n")
%             tout
%             fell = true;
%             break;
%         end
%         
%         [thit,Xhit] = cg_torso_animate(tout,xout,xy_start,bDraw)
%         Xlist = [Xlist, cg_torso_impact(Xhit)];
%         Xinit = Xlist(:,end);
%         
%         [tout,xout] = ode45(@cg_torso_ode,[0 Tmax],Xinit,options);
%     end
%     
%     if fell == true
%         fprintf("flag says robot fell\n");
%         continue
%     end

    % New limit cycle, with new controller defined
    cg_torso_find_limit_cycle

end 
%% Next, we could solve for the nominal u and d2X associated with the
% limit cycle, as input parameters for PFL control:
% Tend = thit + 0.5;
% fi = find(tout<=Tend);
% Xnom = xout(fi,:)';
% tnom = tout(fi)';
% d2Xnom = zeros(5,length(tnom));
% unom = zeros(2,length(tnom));
% 
% for n=1:length(tnom)
%     [d2xnom(:,n),unom(:,n)] = cg_torso_ode(tnom(n),Xnom(:,n));
% end

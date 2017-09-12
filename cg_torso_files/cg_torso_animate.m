function [thit,Xhit,xy_end] = cg_torso_animate(tout,xout,xy_start, bDraw ,xy_wall)
%% Animate cg with torso data
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


%% Below, absolute angles
th1a = xout(:,1);
th2a = xout(:,1)+xout(:,2);
th3a = xout(:,1)+xout(:,3);
x0 = xy_start(1);
y0 = xy_start(2);

xw = xy_wall(1);
yw = xy_wall(2);

%% delgo is amt past stance toe, for xhit checks 
% may want to pass this in as well
delgo = 1e-3;

%% Look up parameters from a separate (single) file
P = cg_torso_params;
%J1=P.J1; J2=P.J2; J3=P.J3;
L1=P.L1; L1c=P.L1c; L3c=P.L3c;
L3 = P.L3;
%m1=P.m1; m2=P.m2; m3=P.m3;
%g = P.g; % gravity

%% look and draw...
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





function [thit,Xhit] = cg_torso_animate(tout,xout,xy_start,bDraw)

%% Animate cg with torso data
thit = []; Xhit = []; % stays empty, if not "hit" with ground is detected

if ~exist('bDraw','var')
    bDraw = true;
end
if ~exist('xy_start','var')
    xy_start = [0,0];
end


%% Below, absolute angles
th1a = xout(:,1);
th2a = xout(:,1)+xout(:,2);
th3a = xout(:,1)+xout(:,3);
x0 = xy_start(1);
y0 = xy_start(2);

%% delgo is amt past stance toe, for xhit checks
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
for n=1:length(tu);
    t1 = interp1(tout,th1a,tu(n));
    t2 = interp1(tout,th2a,tu(n));
    t3 = interp1(tout,th3a,tu(n));
    xh = x0+L1*cos(t1);
    yh = x0+L1*sin(t1);
    xe = xh+L1*cos(t2);
    ye = yh+L1*sin(t2);
    %if ~bDidHit
    if xe>(x0+delgo) && ye<=y0 && n>1
        %keyboard
        % Check preview time
        t1m = interp1(tout,th1a,tu(n-1));
        t2m = interp1(tout,th2a,tu(n-1));
        t3m = interp1(tout,th3a,tu(n-1));
        xhm = x0+L1*cos(t1m);
        yhm = x0+L1*sin(t1m);
        xem = xhm+L1*cos(t2m);
        yem = yhm+L1*sin(t2m);
        if yem>y0
            
            bDidHit = true;
            nu = n+[-1 0];
            thit = interp1([yem, ye],[tu(nu)],0);
            Xhit = zeros(6,1);
            for n2=1:6
                Xhit(n2) = interp1(tout,xout(:,n2),thit);
            end
            %keyboard
            if bDraw
                title('HIT DETECTED!');
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
        plot(x0+[-10 10],y0+[0 0],'k-','LineWidth',2);
        axis image
        axis([xh+[-2 2],y0+[-.2 2]])
        
        drawnow
        pause(dt*1)
    end
end

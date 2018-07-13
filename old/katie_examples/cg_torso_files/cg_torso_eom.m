%% Code to derive EOM and impact equations, for compass gait (CG)
% with a torso.  Stance leg (th1) is absolute. Swing (th2) and
% torso (th3) angles are relative to th1.
% passive ankle, so tau1=0, and
% Xi1 = 0; Xi2 = tau2; Xi3 = tau3;
clear
format compact

syms L1 L1c m1 J1 L2 L2c m2 J2 L3 L3c m3 J3 g
syms th1 th2 th3 x1 x2 x3 y1 y2 y3 xe ye xh yh
syms k2 k3
syms dth1 dth2 dth3 d2th1 d2th2 d2th3 

GC = {th1,th2,th3};
for n=1:length(GC)
    dGC{n} = fulldiff(GC{n},GC);
    d2GC{n} = fulldiff(dGC{n},GC);
end

L2 = L1;
L2c = L1c; % measured down from hip

x1 = (L1-L1c)*cos(th1);
y1 = (L1-L1c)*sin(th1);
x2 = L1*cos(th1) + (L2c)*cos(th1+th2);
y2 = L1*sin(th1) + (L2c)*sin(th1+th2);
x3 = L1*cos(th1) + (L3c)*cos(th1+th3);
y3 = L1*sin(th1) + (L3c)*sin(th1+th3);
xe = L1*cos(th1) + L2*cos(th1+th2); % end of toe
ye = L1*sin(th1) + L2*sin(th1+th2);
xh = L1*cos(th1); % hip joint, where all links meet
yh = L1*sin(th1); 

dth1 = fulldiff(th1,GC);
dth2 = fulldiff(th2,GC);
dth3 = fulldiff(th3,GC);
dx1 = fulldiff(x1,GC);
dx2 = fulldiff(x2,GC);
dx3 = fulldiff(x3,GC);
dy1 = fulldiff(y1,GC);
dy2 = fulldiff(y2,GC);
dy3 = fulldiff(y3,GC);

d2th1 = fulldiff(dth1,GC);
d2th2 = fulldiff(dth2,GC);
d2th3 = fulldiff(dth3,GC);
d2x1 = fulldiff(dx1,GC);
d2x2 = fulldiff(dx2,GC);
d2x3 = fulldiff(dx3,GC);
d2y1 = fulldiff(dy1,GC);
d2y2 = fulldiff(dy2,GC);
d2y3 = fulldiff(dy3,GC);

%% Derive EOM, via Lagrangian:

T = (1/2)*m1*(dx1^2+dy1^2) + (1/2)*m2*(dx2^2+dy2^2) + (1/2)*m3*(dx3^2+dy3^2) + ...
    (1/2)*(J1*dth1^2 + J2*(dth1+dth2)^2 + J3*(dth1+dth3)^2);
V = m1*g*y1 + m2*g*y2 + m3*g*y3 + (1/2)*k2*th2^2 + (1/2)*k3*th3^2;
Lag = T-V;

eq1 = fulldiff(diff(Lag,dth1),GC) - diff(Lag,th1);
eq2 = fulldiff(diff(Lag,dth2),GC) - diff(Lag,th2);
eq3 = fulldiff(diff(Lag,dth3),GC) - diff(Lag,th3);

for n1=1:3
    ctemp = eval(['eq',num2str(n1)]);
    for n2=1:3
        mtemp = eval(['diff(eq',num2str(n1),',d2th',num2str(n2),')']);
        mtemp = simplify(mtemp);
        eval(['M',num2str(n1),num2str(n2),' = mtemp;']);
        M(n1,n2) = mtemp;
        ctemp = ctemp - mtemp*eval(['d2th',num2str(n2)]);
    end
    ctemp = simplify(ctemp);
    eval(['C',num2str(n1),' = ctemp;'])
    C(n1) = ctemp;
end

for n1=1:3
    for n2=1:3
        fprintf('M%d%d = %s;\n',n1,n2,M(n1,n2));
    end
end
for n1=1:3
    fprintf('C%d = %s;\n',n1,C(n1));
end

% Bpre = [0 0; 1 0; 0 1]; % to be pre-multiplied by inv(M),
% to get bottom 3 rows of B matrix

    
%% Impact equations.
% Before impact, look at angular momentum wrt "end" of swing leg,
% at (xe,ye).  After impact, this point on the ground is now
% the contact for the (new) stance leg, so everything is referenced
% wrt this new contact (impact) point.

% angmom_a tracks ALL (a:All) masses and inertia, wrt upcoming impact
% pt, at (xe,ye)
angmom_ae = 0;
dth1a = dth1; dth2a = dth1+dth2; dth3a = dth1+dth3; % ABSOLUTE ang vel's
for n1=1:3
    s = num2str(n1);
    % for i = 1:3, add these 3 terms, to get angular momentum contribution
    % + mi * dyi * (xi-xe)
    % - mi * dxi * (yi-ye)
    % + J *dthia 
    % 3rd term (line above) is an ABSOLUTE ang vel
    angmom_ae = angmom_ae + eval(['m',s,' * dy',s,' * (x',s,'-xe)']);
    angmom_ae = angmom_ae - eval(['m',s,' * dx',s,' * (y',s,'-ye)']);
    angmom_ae = angmom_ae + eval(['J',s,' * dth',s,'a']);
end

%% angmom_st tracks STANCE (s:STance) leg mass and inertia, wrt HIP
angmom_sth = 0;
s = num2str(1); % Stance leg has ID "1"...
angmom_sth = angmom_sth + eval(['m',s,' * dy',s,' * (x',s,'-xh)']);
angmom_sth = angmom_sth - eval(['m',s,' * dx',s,' * (y',s,'-yh)']);
angmom_sth = angmom_sth + eval(['J',s,' * dth',s,'a']);

%% angmom_sw tracks SWING (s:SWing) leg mass and inertia, wrt HIP
angmom_swh = 0;
s = num2str(2); % Stance leg has ID "1"...
angmom_swh = angmom_swh + eval(['m',s,' * dy',s,' * (x',s,'-xh)']);
angmom_swh = angmom_swh - eval(['m',s,' * dx',s,' * (y',s,'-yh)']);
angmom_swh = angmom_swh + eval(['J',s,' * dth',s,'a']);

%% angmom_t tracks TORSO (t:Torso) leg mass and inertia, wrt HIP
angmom_th = 0;
s = num2str(3); % Torso has ID "3"...
angmom_th = angmom_th + eval(['m',s,' * dy',s,' * (x',s,'-xh)']);
angmom_th = angmom_th - eval(['m',s,' * dx',s,' * (y',s,'-yh)']);
angmom_th = angmom_th + eval(['J',s,' * dth',s,'a']);


%% Now, consider post-impact velocities, dth1+, etc., each defined
% wrt to NEW stance toe location.
% We will write geometries in terms of the post-impact equations,
% and the meaning of post-impact velocities is that dth1 is
% absolute, wrt new stance toe. dth2 is relative to dth1 (for
% new swing leg), and dth3 is relative to dth1 (for torso).

angmom_be = 0; % Using "b" instead of "a" here...
dth1a = dth1; dth2a = dth1+dth2; dth3a = dth1+dth3; % ABSOLUTE ang vel's
x0=0; y0=0; % NEW stance toe is at zero
for n1=1:3
    s = num2str(n1);
    % for i = 1:3, add these 3 terms, to get angular momentum contribution
    % + mi * dyi * (xi-xe)
    % - mi * dxi * (yi-ye)
    % + J *dthia 
    % 3rd term (line above) is an ABSOLUTE ang vel
    % NOTE: (x0,y0) below, instead of (xe,ye)...[as used in angmom_ae]
    angmom_be = angmom_be + eval(['m',s,' * dy',s,' * (x',s,'-x0)']);
    angmom_be = angmom_be - eval(['m',s,' * dx',s,' * (y',s,'-y0)']);
    angmom_be = angmom_be + eval(['J',s,' * dth',s,'a']);
end

% %% angmom_s tracks STANCE (s:Stance) leg mass and inertia, wrt HIP
% angmom_sh = 0;
% s = num2str(1); % Stance leg has ID "1"...
% angmom_sh = angmom_sh + eval(['m',s,' * dy',s,' * (x',s,'-xh)']);
% angmom_sh = angmom_sh - eval(['m',s,' * dx',s,' * (y',s,'-yh)']);
% angmom_sh = angmom_sh + eval(['J',s,' * dth',s,'a']);
% 
% %% angmom_t tracks TORSO (t:Torso) leg mass and inertia, wrt HIP
% angmom_th = 0;
% s = num2str(3); % Torso has ID "3"...
% angmom_th = angmom_th + eval(['m',s,' * dy',s,' * (x',s,'-xh)']);
% angmom_th = angmom_th - eval(['m',s,' * dx',s,' * (y',s,'-yh)']);
% angmom_th = angmom_th + eval(['J',s,' * dth',s,'a']);


%% Group all terms, to find Q- and Q+, such that
% (Q-)*dth- = (Q+)*dth+
AngMom_minus = [angmom_ae; angmom_sth; angmom_th];
dth1=0; dth2=0; dth3=0; % to find matrix values, below
syms Qminus
for n1=1:3
    for n2=1:3
        % Below, "diff" wrt whichever dthi value should also work...
        eval(['dth',num2str(n2),' = 1;']); % Set appropriate ang vel to unity
        Qminus(n1,n2) = eval(AngMom_minus(n1,:));
        eval(['dth',num2str(n2),' = 0;']);
    end
end


%% Group all terms, to find Q- and Q+, such that
% (Q-)*dth- = (Q+)*dth+
AngMom_plus = [angmom_be; angmom_swh; angmom_th];
dth1=0; dth2=0; dth3=0; % to find matrix values, below
syms Qplus
for n1=1:3
    for n2=1:3
        % Below, "diff" wrt whichever dthi value should also work...
        eval(['dth',num2str(n2),' = 1;']); % Set appropriate ang vel to unity
        Qplus(n1,n2) = eval(AngMom_plus(n1,:));
        eval(['dth',num2str(n2),' = 0;']);
    end
end


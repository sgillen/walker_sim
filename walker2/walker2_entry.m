%entry point for walker simultion developed as basically a tutorial during
%my first week with UCSB

X0 = [pi/2+ .1 ; -pi/2;-pi/2;0;0;0];


%ode 45 was freezing up, apparently we might have a stuff system, ode15s
%seems to work much faster

options = odeset('RelTol', 1e-8, 'AbsTol', 1e-9);
[t,y] = ode15s(@walker2_noncollocated,[0 10],X0,options);

figure(1);

walker2_animate(t,y);

tau1 = 0*t;
tau2 = 0*t;

TAU = [tau1';tau2'];



for n=1:length(t)
    [dx,TAU(:,n)] = walker2_noncollocated(t(n),y(n,:));
end

figure(2); subplot(211); plot(t,TAU(1,:));
figure(3); subplot(211); plot(t,TAU(2,:));


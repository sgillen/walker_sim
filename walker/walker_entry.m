%entry point for walker simultion developed as basically a tutorial during
%my first week with UCSB

X0 = [0;0;0;0;0;0];

[t,y] = ode45(@walker2_noncollocated,[0 5],X0);

figure(1);

M = walker_animate(t,y);

tau1 = 0*t;
tau2 = 0*t;

TAU = [tau1';tau2'];



for n=1:length(t)
    [dx,TAU(:,n)] = walker_noncollocated(t(n),y(n,:));
end

figure(2); subplot(211); plot(t,TAU(1,:));
figure(3); subplot(211); plot(t,TAU(2,:));


% run acrobot examples...
% 
X0 = [-pi/2+.1;0;0;0];
[t,y] = ode45(@acrobot_collocated_linearization,[0 20],X0);
figure(1);
M = acrobot_animate(t,y);
tau = 0*t;
for n=1:length(t)
    [dx,tau(n)] = acrobot_collocated_linearization(t(n),y(n,:));
end
figure(2); subplot(211); plot(t,tau);

% 
% X0 = [-pi/2+.1;0;0;0];
% [t,y] = ode45(@acrobot_noncollocated_linearization,[0 20],X0);
% acrobot_animate(t,y)
% tau = 0*t;
% for n=1:length(t)
%     [dx,tau(n)] = acrobot_noncollocated_linearization(t(n),y(n,:));
% end
% figure(2); subplot(212); plot(t,tau);

X0 = [-pi/2+.1;0;0;0];
[t,y] = ode45(@acrobot_eom,[0 20],X0);
figure(1);
M = acrobot_animate(t,y);


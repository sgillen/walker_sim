function M = walker_animate(t_ode,X_ode)

M=[];
dt = .05;
tlist = min(t_ode):dt:max(t_ode);
Xlist = zeros(6,length(tlist));
for i=1:4
    Xlist(i,:) = interp1(t_ode,X_ode(:,i),tlist);
end
clear M
for n=1:length(tlist)
    walker2_draw(Xlist(:,n)); drawnow; pause(dt*.005);
    %M(n) = getframe(gca);
end

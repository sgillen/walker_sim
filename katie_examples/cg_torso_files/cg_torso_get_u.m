cg_torso_find_limit_cycle % to get "Xfixed"...

S = odeset('AbsTol',1e-6); %,'RelTol',1e-8);
[tout,xout] = ode45(@cg_torso_ode,[0 Tmax],Xfixed,S);
[thit,xhit] = cg_torso_animate(tout,xout,[0 0],0);
fi = find(tout<thit);
tset = [tout(fi,:); thit];
xset = [xout(fi,:); xhit'];
uset = zeros(length(tset),2);
for n=1:length(tset)
    [a,b] = cg_torso_ode(tset(n),xset(n,:)');
    uset(n,:) = b;
end



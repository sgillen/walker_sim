function cost = cg_torso_LCcost(X1,xy_start)

X2 = cg_torso_step(X1,false,xy_start);
cost = 1e2*norm(X2-X1);
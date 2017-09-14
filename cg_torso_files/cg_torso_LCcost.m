function cost = cg_torso_LCcost(X1)

X2 = cg_torso_step(X1);
cost = 1e2*norm(X2-X1);
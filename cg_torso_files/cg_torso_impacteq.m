% derive the impact equation of the compass gait with a torso
% 

syms mh m1 m2 md
syms g l d a b
syms Os_p Os_n Ons_p Ons_n Od_p Od_n
syms dOs_p dOs_n dOns_p dOns_n dOd_p dOd_n

vh_n = [-l*dOs_n*cos(Os_n) -l*dOs_n*sin(Os_n) 0];
vh_p = [-l*dOs_p*cos(Os_p) -l*dOs_p*sin(Os_p) 0];

vs_n = [-a*dOs_n*cos(Os_n) -a*dOs_n*sin(Os_n) 0];
vs_p = [-a*dOs_p*cos(Os_p) -a*dOs_p*sin(Os_p) 0];

vns_n =[b*dOns_n*cos(Ons_n)-l*dOs_n*cos(Os_n) b*dOns_n*sin(Ons_n)-l*dOs_n*sin(Os_n) 0];
vns_p =[b*dOns_p*cos(Ons_p)-l*dOs_p*cos(Os_p) b*dOns_p*sin(Ons_p)-l*dOs_p*sin(Os_p) 0];

vd_n = [-l*dOs_n*cos(Os_n)-d*dOd_n*cos(Od_n) -l*dOs_n*sin(Os_n)-d*dOd_n*sin(Od_n) 0];
vd_p = [-l*dOs_p*cos(Os_p)-d*dOd_p*cos(Od_p) -l*dOs_p*sin(Os_p)-d*dOd_p*sin(Od_p) 0];

eq2 =m2*cross([b*sin(Os_n) -b*cos(Os_n) 0],vs_n) -m2*cross([b*sin(Ons_p) -b*cos(Ons_p) 0],vns_p);
eq1 =mh*cross([-l*sin(Os_n) l*cos(Os_n) 0],vh_n)+m2*cross([-a*sin(Os_n) a*cos(Os_n) 0],vs_n)+m1*cross([b*sin(Ons_n)-l*sin(Os_n) l*cos(Os_n)-b*cos(Ons_n) 0],vns_n) + md*cross([-l*sin(Os_n)-d*sin(Od_n) l*cos(Os_n)+d*cos(Od_n) 0],vd_n)-(mh*cross([-l*sin(Os_p) l*cos(Os_p) 0],vh_p)+m2*cross([b*sin(Ons_p)-l*sin(Os_p) l*cos(Os_p)-b*cos(Ons_p) 0],vns_p)+m1*cross([-a*sin(Os_p) a*cos(Os_p) 0],vs_p)+ md*cross([-l*sin(Os_p)-d*sin(Od_p) l*cos(Os_p)+d*cos(Od_p) 0],vd_p));
eq3 =md*cross([-d*sin(Od_n) d*cos(Od_n) 0],vd_n)-md*cross([-d*sin(Od_p) d*cos(Od_p) 0],vd_p);

EQ1 = simplify(eq1)
EQ2 = simplify(eq2)
EQ3 = simplify(eq3)
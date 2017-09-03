% playing around with two link manipulator
clear all

%length of the two arms
l1 = 5; 
l2 = 4;

%functions derived in mathematica, they tell you what thetas you need to
%reach a given x,y value
theta2 = @(x,y) atan2((1+(-1/2).*l1.^(-1).*l2.^(-1).*((-1).*l1.^2+(-1).*l2.^2+x.^2+y.^2)).^(1/2),(1/2).*l1.^(-1).*l2.^(-1).*((-1).*l1.^2+(-1).*l2.^2+x.^2+y.^2))
theta1 = @(x,y,theta2) atan2(y,x)+(-1).*atan2(l2.*sin(theta2),l1+l2.*cos(theta2))
  
% desired x and y
xd = 0 
yd = 8

%compute our thetas by pluggin in the desired values to the functions we
%defined
t2 = theta2(xd,yd)
t1 = theta1(xd,yd,t2)

%draw two lines that represent the two arms we have
line([0, l1*cos(t1)], [0, l1*sin(t1)])
line([l1*cos(t1), l1*cos(t1) + l2*cos(t1 + t2)] ,[l1*sin(t1), l1*sin(t1) + l2*sin(t1 + t2)], 'Color' ,'red' )
 

%the jacobian for our system
J = [-l1*sin(t1) - l2*sin(t1 + t2), -l2*sin(t1 + t2); l1*cos(t1) - l2*cos(t1 + t2), l2*cos(t1 + t2)]
 



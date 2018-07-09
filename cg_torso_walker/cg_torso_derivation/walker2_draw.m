function walker_draw(X)

q1 = X(1); q2 = X(2); q3 = X(3);

%feel like these should be global
l1 = 2; l2 = 2; l3 = 2;

figure(1); clf
rectangle('Position',[-.1,-.1,.2,.2])

line([0, l1*cos(q1)], [0, l1*sin(q1)], 'Color', 'black'); hold on

line([l1*cos(q1), l1*cos(q1)+ l2*cos(q2)], [l1*sin(q1), l1*sin(q1) + l2*sin(q2)],'Color','red' );

line([l1*cos(q1), l1*cos(q1) + l3*cos(q3)], [l1*sin(q1), l1*sin(q1) + l3*sin(q3)], 'Color' ,'blue');


axis equal; axis off
W = l1+l2+l3;
axis([-W W -W W]*1.1)




% th = .5*.1; % half-THickness of arm
% c1 = [0 0 1];  % Color for link 1
% c2 = [1 0 0];  % Color for link 2
% c3 = [0 1 1];
% 
% avals = pi*[0:.05:1];
% x1 = [0 L1 L1+th*cos(avals-pi/2) L1 0 th*cos(avals+pi/2)];
% y1 = [-th -th th*sin(avals-pi/2) th th th*sin(avals+pi/2)];
% r1 = (x1.^2 + y1.^2).^.5;
% a1 = atan2(y1,x1);
% x1draw = r1.*cos(a1+q1);  % x pts to plot, for Link 1
% y1draw = r1.*sin(a1+q1);  % y pts to plot, for Link 1
% x1end = L1*cos(q1);  % "elbow" at end of Link 1, x
% y1end = L1*sin(q1);  % "elbow" at end of Link 1, x
% 
% x2 = [0 L2 L2+th*cos(avals-pi/2) L2 0 th*cos(avals+pi/2)];
% y2 = [-th -th th*sin(avals-pi/2) th th th*sin(avals+pi/2)];
% r2 = (x2.^2 + y2.^2).^.5;
% a2 = atan2(y2,x2);
% x2draw = x1end+r2.*cos(a2+q1+q2);  %TODO x pts to plot, for Link 1
% y2draw = y1end+r2.*sin(a2+q1+q2);  %TODO y pts to plot, for Link 1
% 
% x3 = [0, L3, L3+th*cos(avals - pi/2), L3, 0, th*cos(avals + pi/2)];
% y3 = [-th, -th, th*sin(avals-pi/2), th, th, th*sin(avals+pi/2)];
% r3 = (x3.^2 + y3.^2).^.5;
% a3 = atan2(y3,x3);
% x3draw = x1end+r3.*cos(a3+q1+q3); %TODO
% y3draw = y1end+r3.*cos(a3+q1+q3); %TODO
% 
% % now, draw the acrobot:
% figure(1); clf
% p1 = patch(x1draw,y1draw,'b','FaceColor',c1); hold on
% p2 = patch(x2draw,y2draw,'r','FaceColor',c2);
% p3 = patch(x3draw,y3draw,'g','FaceColor',c3);
% axis equal; axis off
% W = L1+L2+L3;
% axis([-W W -W W]*1.1)





 

function [value,isterminal,direction] = fall_event(t,y)
% this function attempts to find out if the walker has fallen down, you are
% supposed to pass this is as an event function to odexx via the option
% parameter

P = cg_torso_params;

%is is how far below zero we will allow a leg to go before we consider it
%below the horizontal
tol = .2;


yh = P.L1*sin(y(1));
%y2 = yh + P.L1*sin(y(2) + y(1));
y3 = yh + P.L3*sin(y(3) + y(1));


q1 = y(1); q2 = y(2); q3 = y(3);


value = max(0,min([yh+tol,y3+tol])); %basically this call returns 0 if any of yh y2 y3 is less then 0.

% if value == 0
%     fprintf("the walker fell down!\n");
%    %feel like these should be global
%     figure(1); clf
%     rectangle('Position',[-.1,-.1,.2,.2])
% 
%     line([0, P.L1*cos(q1)], [0, P.L1*sin(q1)], 'Color', 'black'); hold on
% 
%     line([P.L1*cos(q1), P.L1*cos(q1)+ P.L1*cos(q2+q1)], [P.L1*sin(q1), P.L1*sin(q1) + P.L1*sin(q2+q1)],'Color','red' );
% 
%     line([P.L1*cos(q1), P.L1*cos(q1) + P.L3*cos(q3+q1)], [P.L1*sin(q1), P.L1*sin(q1) + P.L3*sin(q3+q1)], 'Color' ,'blue');
% 
%     axis equal; axis off
%     W = 2*P.L1+P.L3;
%     axis([-W W -W W]*1.1)
%     
%     pause(.05);
% 
% end



isterminal = 1;        % Stop the integration
direction = 0;         % All direction
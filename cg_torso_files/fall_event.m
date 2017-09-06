function [value,isterminal,direction] = fall_event(t,y)
% this function attempts to find out if the walker has fallen down, you are
% supposed to pass this is as an event function to odexx via the option
% parameter

P = cg_torso_params;

%this is how far below zero we will allow a leg to go before we consider it
%below the horizontal
tol = .2;


yh = P.L1*sin(y(1));
%y2 = yh + P.L1*sin(y(2) + y(1));
y3 = yh + P.L3*sin(y(3) + y(1));

value = max(0,min([yh+tol,y3+tol])); %basically this call returns 0 if any of yh y2 y3 is less then 0.

% if value == 0
%     fprintf("the walker fell down!\n");
%     t
%     yh
%     %y2
%     y3
% end



isterminal = 1;        % Stop the integration
direction = 0;         % All direction
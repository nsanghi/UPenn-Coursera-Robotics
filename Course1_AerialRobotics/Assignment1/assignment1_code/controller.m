function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;

% keyboard
% FILL IN YOUR CODE HERE
Kd = 10;
Kp = 60;
e = s_des(1) - s(1);
e_dot = s_des(2) - s(2);
u = params.mass*(params.gravity+Kp*e+Kd*e_dot);

end


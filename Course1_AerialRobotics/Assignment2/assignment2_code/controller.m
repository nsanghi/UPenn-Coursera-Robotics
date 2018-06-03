function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
kp_z = 25.0;
kv_z = 105.0; 
kp_phi = 100.0*120;
kv_phi = 120*60;
kp_y = 15.0;
kv_y = 100.0;

pos_error = des_state.pos - state.pos;
vel_error = des_state.vel - state.vel;
des_accel = des_state.acc;

phi = state.rot;
phi_dot = state.omega;

phi_commanded = -1/params.gravity*(des_accel(1)+kv_y*vel_error(1)+kp_y*pos_error(1));
u1 = params.mass*(params.gravity+des_accel(2)+kv_z*vel_error(2)+kp_z*pos_error(2));
u2 = params.Ixx*(kv_phi*(-phi_dot)+kp_phi*(phi_commanded-phi));
end

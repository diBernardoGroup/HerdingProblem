%% return angular accelleration and velocity of the herder

function [theta_ddot, theta_dot] = HerderDynamics_angular(theta_vel, T)

global b_theta_S 

theta_dot = theta_vel; 
theta_ddot = - (b_theta_S * theta_vel +  T); % + 0.1*rand(1); 
%% return the radial accelleration and velocity of the herder 

function [yr_ddot,yr_dot] = HerderDynamics_radial(yrvel,F)

global br

yr_dot = yrvel;

yr_ddot = - (br * yrvel + F); % + rand(1);

%% return collision avoidance effect 

function [xddot, xdot] = TargetDynamicsCollision(x,x_neigh,prov,xvel) 

xdot = xvel ;

xddot = prov + (x - x_neigh)  / (norm(x - x_neigh).^3); 



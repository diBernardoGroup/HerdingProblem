%% return the velocity in cartesian coordinates of the target agents in oresence of a herder


function xdot = TargetDynamics(x,y)   

global RepulMag

xdot = RepulMag * (x - y ) / (norm(x - y) .^3) ; 


%% return target brownian accelleration and velocity 

function xdot = TargetDynamicsBrownian_EulerMaruyama()

global BrownDist BrownMag

xdot = BrownMag .* [random(BrownDist); random(BrownDist)]; 



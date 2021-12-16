function [Target] = Target_Initialization(TargetNumber, TotalTimesteps)
% Initialize an array of "TargetNumber + 1" struct
%
% The first "TargetNumber" struct have fields :
%
% x  cartesian position
% x_dot   cartesian total velocity
% x_dot1   cartesian brownian velocity component
% x_dot2   cartesian collision velocity component
% x_dot3  cartesian repulsive velocity component
%
%
% The "TargetNumber + 1" has fields :
%
% gcm   herd global centre of mass position
% col_temp   temp collision avoidance position
%
%
% and assign initial cartesian positions

global rstar xstar

for q = 1 : TargetNumber
    Target(q).x = zeros(2,TotalTimesteps);                                           % cartesian position
    Target(q).x_dot = zeros(2,TotalTimesteps);                                       % cartesian velocity
    Target(q).x_dot1 = zeros(2,TotalTimesteps);
    Target(q).x_dot2 = zeros(2,TotalTimesteps);
    Target(q).x_dot3 = zeros(2,TotalTimesteps);
    
    [Target(q).x(1,1),Target(q).x(2,1)] = pol2cart(2 * pi * rand(1,1) - pi, 2*rstar + 0.5 * rand(1,1) - norm(xstar));       % initial random position around a fixed radial distance from the centre of the environment
    
end

Target(TargetNumber+1).gcm = zeros(2,TotalTimesteps);               % global center of mass
Target(TargetNumber+1).coll_temp = zeros(2,TotalTimesteps);         % temp storage for collisions

end
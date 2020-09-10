
function Herder = Herder_Initialization(HerderNumber, TotalTimesteps)

% Initialize an array of "HerderNumber" struct of fields :
%
% y             cartesian position
% r             radial position
% theta         angular position
% r_dot         radial velocity
% theta_dot     angular velocity
% r_ddot         radial acceleration
% theta_ddot     angular acceleration
%
%
% and assign initial polar (and cartesian) initial around a fixed radial distance from the centre of the environment

global rstar

for p = 1 : HerderNumber
    
    Herder(p).y = zeros(2,TotalTimesteps);
    Herder(p).r = zeros(1,TotalTimesteps);
    Herder(p).theta = zeros(1,TotalTimesteps);
    Herder(p).r_dot = zeros(1,TotalTimesteps);          % radial velocity
    Herder(p).theta_dot = zeros(1,TotalTimesteps);      % angular velocity
    Herder(p).r_ddot = zeros(1,TotalTimesteps);          % radial acceleration
    Herder(p).theta_ddot = zeros(1,TotalTimesteps);      % angular acceleration
    
    
end

Herder(HerderNumber+1).rps(:,1) = 4 * rstar * ones(HerderNumber,1);
Herder(HerderNumber+1).thetaps(:,1) = zeros(HerderNumber,1);


for p = 1 : HerderNumber
    Herder(p).theta(1,1) = p * 2 * pi / HerderNumber ;
    Herder(p).r(1,1) = 4 * rstar;
    [Herder(p).y(1,1), Herder(p).y(2,1)] = pol2cart(Herder(p).theta(1,1), Herder(p).r(1,1));
end


end
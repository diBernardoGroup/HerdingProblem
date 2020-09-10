function Func_NovelModel_B_INIT(TargetNumber,RepulMultiplier)
 
% Assign system parameters and save them on file in "Parameters\" folder 
%
% Simulation parameters : T N dt
% Environment parameters : xstar rstar delta_rmin Q P
% Target's parameters : BrownDist BrownMag RepulMag
% Herder's parameters : br b_theta_S eps_r eps_theta

global T N dt

dt = 0.006;         % fixedDeltaTime
T = 30 * 2;         % simulation total time
N = T / dt;         % number of steps

global xstar rstar delta_rmin Q P

Q = TargetNumber;                      % number of target
P = 2;                      % number of herder
xstar = zeros(2,1);        % goal position initialization
rstar = 1;                  % radius of goal region
delta_rmin = rstar + (.062 - .061539); %

global BrownDist BrownMag RepulMag

BrownDist = makedist('Normal',0,sqrt(dt)); % Initialize the probability distribution for our random variable with mean 0 and stdev of sqrt(dt)
BrownMag = 0.05; % original value 0.05
RepulMag = RepulMultiplier * BrownMag; % nominal value 20*0.05


global br b_theta_S eps_r eps_theta

or = 10;           
eps_r = or^2; % radial spring constant
otheta = 7.85;
eps_theta = otheta^2; 
zeta_r = 0.5;   
zeta_theta = 0.7;
br = 2 * zeta_r * or;            % radial damping term
b_theta_S = 2 * zeta_theta * otheta;     % angular damping term

save(['Parameters\param_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat']); 


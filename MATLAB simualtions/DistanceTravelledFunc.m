function LenghtTrajectory = DistanceTravelledFunc(AgentPos, steadystate, T_interval)

% steadystate = 0   if containment rate over full trial 
% steadystate = 1   if containment rate over last T_interval second of trial

global N T 

if steadystate == 0 
    N_start = 1; 
else
    N_start = N * (T - T_interval) / T + 1;
end 

    
AgentPosShifted = circshift(AgentPos(:,N_start:N), -1, 2); 
TrajectoryPieces = vecnorm(AgentPos(:,N_start:N) - AgentPosShifted, 2, 1);
TrajectoryPieces = TrajectoryPieces(1,1:length(TrajectoryPieces)-1);
LenghtTrajectory = sum(TrajectoryPieces); 
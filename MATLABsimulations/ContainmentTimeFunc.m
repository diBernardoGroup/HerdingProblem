function [TimeFirstIn, TimeFirstOut, DeltaTimeIn] = ContainmentTimeFunc(Target_norm, steadystate, T_interval)

% steadystate = 0   if containment rate over full trial 
% steadystate = 1   if containment rate over last T_interval second of trial

global N T rstar

if steadystate == 0 
    N_start = 1; 
else
    N_start = N * (T - T_interval) / T + 1;
end 


cont_in = 0; 
cont_out = 0; 

dt = T / N; 

for t = N_start : N


    if Target_norm(:,t) < rstar 
        if cont_in == 0
            TimeFirstIn = t * dt; 
        end 
        cont_in = cont_in + 1; 
    else 
        if cont_in > 1 
            TimeFirstOut = t * dt; 
        end 
        cont_out = cont_out + 1; 
    end 
 
end 

DeltaTimeIn = cont_in * dt; 

if ~exist('TimeFirstIn','var')
    DeltaTimeIn = 0;
    TimeFirstIn = T; 
    TimeFirstOut = T; 
end
if ~exist('TimeFirstOut','var')
TimeFirstOut = T; 
end 


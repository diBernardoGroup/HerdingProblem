function [rps_P, thetaps_P, Bounds, chsd] = planeSearch_peer2peer(t, Herder, Target)

global P Q xstar 

ChasedIndex = (Q+1)*ones(Q,P); 

for p = 1 : P
    Herder_angle(p) = wrapTo2Pi(Herder(p).theta(1,t));
end

for p = 1 : P
    
    index_prev = p - 1; 
    index_next = p + 1;  
    
    if p == 1 , index_prev = P; end 
    if (p == P) , index_next = 1; end 
    
    Angle_with_prev(p) = abs(Herder_angle(p) - Herder_angle(index_prev)); 
    Angle_with_next(p) = abs(Herder_angle(p) - Herder_angle(index_next)); 
    
    LB(p) = Herder_angle(p) - Angle_with_prev(p) / 2; 
    UB(p) = Herder_angle(p) + Angle_with_next(p) / 2; 
    
    if LB(p) == UB(p)
        UB(p) = LB(p) + 2 * pi / P; 
    end 
   
end 

[LB,index_temp] = sort(LB); 
UB = UB(index_temp); 

Bounds = [LB(1:P); UB]; 

% % get Target polar positions and angles wrapped in [0,2pi] 
for q = 1 : Q
    Target_pos(:,q) = Target(q).x(:,t) - xstar;
    [Target_angle(q), Target_norm(q)] = cart2pol(Target_pos(1,q), Target_pos(2,q));
    Target_angle_wrapped(q) = wrapTo2Pi(Target_angle(q));     
end

[Target_norm_sorted, index_sorted] = sort(Target_norm,'descend'); 
Target_angle_wrapped = Target_angle_wrapped(index_sorted); 

for p = 1 : P
    index_second = find(Target_angle_wrapped < Bounds(2,p));
    index_first = find(Target_angle_wrapped(index_second) >=Bounds(1,p));
    ChasedIndex(index_first,p) = index_sorted(index_second(1:length(index_first)));
    Target_angle_wrapped(index_second) = 10;
    index_second = [];
    index_first = [];
end

TargetGCM = mean(Target_pos,2); 
[Target_angle(Q+1), Target_norm(Q+1)] = cart2pol(TargetGCM(1), TargetGCM(2));

chsd = ChasedIndex(1,:); 
rps_P = Target_norm(chsd);
thetaps_P = Target_angle(chsd); 
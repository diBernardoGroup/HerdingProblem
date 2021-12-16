function [rps_P, thetaps_P, chsd] = planeSearch_global(Target_pos)

global Q 

Chased = []; 

for q = 1 : Q 
    [Target_angle(q), Target_norm(q)] = cart2pol(Target_pos(1,q), Target_pos(2,q)); 
end 

[Chased, index] = sort(Target_norm, 'descend');

for q = 1 : Q 
    if Target_norm(q) == Chased(1)
        rps_P(1) = Target_norm(q); 
        thetaps_P(1) = Target_angle(q); 
        chsd(1) = q;
    end
    
    if Target_norm(q) == Chased(2)
        rps_P(2) = Target_norm(q); 
        thetaps_P(2) = Target_angle(q); 
        chsd(2) = q;
    end
    
end 

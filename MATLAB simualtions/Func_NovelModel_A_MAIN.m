function Func_NovelModel_A_MAIN(TrialNumber,TargetNumber,RepulMultiplier,how,howSearch)

load(['Parameters\param_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat']);

Target = Target_Initialization(Q,N);
Herder = Herder_Initialization(P,N);

LowerBound_temp = [];
UpperBound_temp = [];
t_search = 0;

switch how
    case 1  % search strategy GLOBAL %
        howSearch = 'Global';
    case 2  % search strategy STATIC %
        howSearch = 'Static';
    case 3  % search strategy INDIVIDUAL %
        howSearch = 'LeaderFollower';
    case 4  % search strategy COOPERATIVE %
        howSearch = 'PeerToPeer';
end


%% Simulation loop
for t = 1 : N
    
    
    %% evaluate far agents
    for q = 1 : Q
        target_pos(:,q) = Target(q).x(:,t) - xstar;
    end
    
    if t==1 || mod(t,50) == 0
        t_search = t_search + 1;
        switch how
            case 1   % global plane search
                [rps_temp(:,t_search), thetaps_temp(:,t_search),chased(:,t_search)] = planeSearch_global(target_pos);
                LowerBound_temp(:,t_search) = zeros(2,1);
                UpperBound_temp(:,t_search) = zeros(2,1);
            case 2 % partial plane search according to static
                [rps_temp(:,t_search), thetaps_temp(:,t_search), Bounds_temp(:,:,t_search),chased(:,t_search)] = planeSearch_static(t, Target);
            case 3 % partial plane search according to leaderfollower
                [rps_temp(:,t_search), thetaps_temp(:,t_search), Bounds_temp(:,:,t_search),chased(:,t_search)] = planeSearch_LeaderFollower(t, Herder, Target);
            case 4   % partial plane search according to peer2peer
                [rps_temp(:,t_search), thetaps_temp(:,t_search), Bounds_temp(:,:,t_search),chased(:,t_search)] = planeSearch_peer2peer(t, Herder, Target);
        end
    end
    
    rps(:,t) = rps_temp(:,t_search);
    thetaps(:,t) = thetaps_temp(:,t_search);
    %     LowerBound(:,t) = Bounds_temp(1,:,t_search);
    %     UpperBound(:,t) = Bounds_temp(2,:,t_search);
    
    
    target_pos = [];
    
    %% Herders dynamics
    for p = 1 : P
        
        if Herder(p).theta(1,t) >= 0
            if abs(Herder(p).theta(1,t) - thetaps(:,t)) > pi
                if Herder(p).theta(1,t) - thetaps(:,t) >= 0
                    thetaps(:,t) =  thetaps(:,t) + 2 * pi;
                else
                    thetaps(:,t) - 2 * pi;
                end
            end
        else
            if abs(Herder(p).theta(1,t) - thetaps(:,t)) < pi
                if Herder(p).theta(1,t) - thetaps(:,t) >= 0
                    thetaps(:,t) =  thetaps(:,t) - pi;
                else
                    thetaps(:,t) + pi;
                end
            end
        end
        
        xsi(p,t) = getXsi(rps(p,t));
        
        % radial and angular force
        Force(p,t) = ForceRadial(Herder(p).r(1,t), xsi(p,t), rps(p,t));
        Torque(p,t) = ForceAngular(Herder(p).theta(1,t), xsi(p,t), thetaps(p,t));
        
        [Herder(p).r_ddot(1,t), Herder(p).r_dot(1,t)] = HerderDynamics_radial(Herder(p).r_dot(1,t), Force(p,t));     %  radial dynamics
        [Herder(p).theta_ddot(1,t), Herder(p).theta_dot(1,t)] = HerderDynamics_angular(Herder(p).theta_dot(1,t), Torque(p,t));   %  angular dynamics
        
        % Herder state update
        Herder(p).r_dot(1,t+1) = Herder(p).r_dot(1,t) + Herder(p).r_ddot(1,t) * dt;
        Herder(p).r(1,t+1) = Herder(p).r(1,t) + Herder(p).r_dot(1,t) * dt;
        
        Herder(p).theta_dot(1,t+1) = Herder(p).theta_dot(1,t) + Herder(p).theta_ddot(1,t) * dt;
        Herder(p).theta(1,t+1) = Herder(p).theta(1,t) + Herder(p).theta_dot(1,t) * dt;
        
        [Herder(p).y(1,t+1), Herder(p).y(2,t+1)] = pol2cart(Herder(p).theta(1,t+1), Herder(p).r(1,t+1));
        
        
    end
    
    %% Targets dynamics
    for q = 1 : Q
        
        % x_dot = f(x,y)  -- Diffusive motion
        Target(q).x_dot1(:,t) =  TargetDynamicsBrownian_EulerMaruyama();
        
        
        for p = 1 : P
            % x_dot = f(x,y)  -- Escaping the herder
            Target(q).x_dot3_temp = TargetDynamics(Target(q).x(:,t),Herder(p).y(:,t));
            Target(q).x_dot3(:,t)  = Target(q).x_dot3(:,t)  + Target(q).x_dot3_temp;
        end
        
        
        % x_dot = v , v_dot = f(x,v) -- Collision avoidance b/t targets
        for q1 = 1 : Q
            if (q1 ~= q)
                if (norm(Target(q).x(:,t) - Target(q1).x(:,t)) < 0.01)
                    [Target(q).x_ddot2(:,t), Target(q).x_dot2(:,t)] = TargetDynamicsCollision(Target(q).x(:,t), Target(q1).x(:,t), Target(Q+1).coll_temp(:,t),Target(q).x_dot(:,t));
                    Target(Q+1).coll_temp(:,t) = Target(q).x_ddot2(:,t);
                end
            end
        end
        
        % state update
        Target(q).x_dot(:,t) = Target(q).x_dot2(:,t) + Target(q).x_dot3(:,t);
        Target(q).x(:,t+1) = Target(q).x(:,t) + Target(q).x_dot(:,t) * dt + Target(q).x_dot1(:,t);
        
    end
    
    
end

for q = 1 : Q
    TargetPos(:,:,q) = Target(q).x;
end

for q = 1 : P
    HerderPos(:,:,q) = Herder(q).y;
end

file_nameTrial = ['Trials\',howSearch,'\','trial_',num2str(howSearch),'_',num2str(TrialNumber),'_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat'];
save(file_nameTrial, 'TargetPos','HerderPos');

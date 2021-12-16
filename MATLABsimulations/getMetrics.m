function MetricsMatrix = getMetrics(TargetNumber,RepulMultiplier,TrialNumberTot,HowDivisionTot,paramTrue)

if paramTrue == 1
    load(['Parameters\param_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat']);
else
    
    global T N dt xstar rstar Q P
    
    dt = 0.006;         % fixedDeltaTime
    T = 30 * 2;         % simulation total time
    N = T / dt;         % number of steps
    
    Q = TargetNumber;                      % number of target
    P = 2;                      % number of herder
    xstar = zeros(2,1);        % goal position initialization
    rstar = 1;                  % radius of goal region
    
end

T_interval = T; % 45 for T = 60 | 84 for T = 120
steadystate = 1;

for how = HowDivisionTot(1) : HowDivisionTot(2)
    
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
    
    cont_successfull_trials = 0;
    metrics_deltaTime = [];
    metrics_containmentRate = [];
    metrics_spreadNormalized = [];
    metrics_spread = [];
    metrics_distance = [];
    metrics_distanceTravelled = [];
    successfull_trials = [];
    
    for TrialNumber = TrialNumberTot(1) : TrialNumberTot(2)
        
        % load data from trialfile 'file_name' %
        if paramTrue == 1
            file_name = ['Trials\',howSearch,'\','trial_',num2str(howSearch),'_',num2str(TrialNumber),'_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat'];
        else
            file_name = ['C:\Users\fa17936\OneDrive - University of Bristol\Trials\',howSearch,'\','trial_',num2str(howSearch),'_',num2str(TrialNumber),'_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat'];
            
        end
        
        load(file_name);
        
        if exist('Target','var')
            
            for q = 1 : Q
                TargetPos(:,:,q) = Target(q).x;
            end
            
            for q = 1 : P
                HerderPos(:,:,q) = Herder(q).y;
            end
        end
        
        % calculate function's arguments
        for q = 1 : Q
            
            for t = 1 : N
                Target_norm(q,t) = norm(TargetPos(:,t,q) - xstar);
                Target_norm_abs(q,t) = norm(TargetPos(:,t,q));
            end
            
            TargetPosX(q,:) = TargetPos(1,:,q);
            TargetPosY(q,:) = TargetPos(2,:,q);
            
        end
        
        % discard unsuccessfull trials %
        % a "T" long trial is considered succsessfull only if targets are contained for
        % the "percentage" of last "T_interval" of the trial duration
        
        
        [timeIN, timeOUT, deltaTimeIN] = ContainmentTimeFunc(Target_norm, steadystate, T_interval);
        
        
        %         if deltaTimeIN >= T_interval * percentage
        
        cont_successfull_trials = cont_successfull_trials + 1;
        successfull_trials = [successfull_trials, TrialNumber];
        
        % containment time
        metrics_deltaTime = [metrics_deltaTime; deltaTimeIN];
        
        % containment rate
        [contRate, escapeRate] = ContainmentRateFunc(Target_norm, steadystate, T_interval);
        metrics_containmentRate = [metrics_containmentRate; contRate];
        
        % heard spread
        herdSpread = HerdSpreadFunc(TargetPosX, TargetPosY, steadystate, T_interval);
        herdSpreadNormalized = herdSpread / (pi * rstar^2);
        metrics_spreadNormalized = [metrics_spreadNormalized; herdSpreadNormalized];
        metrics_spread = [metrics_spread; herdSpread];
        
        % mean distance
        TargetGoalAvgDistance = MeanDistanceFunc(TargetPosX, TargetPosY,steadystate,T_interval);
        metrics_distance = [metrics_distance; TargetGoalAvgDistance];
        
        % Distance travelled by each herder
        
        for p = 1 : P
            LenghtTrajectory(p) = DistanceTravelledFunc(HerderPos(:,:,p), steadystate, T_interval);
        end
        metrics_distanceTravelled = [metrics_distanceTravelled; LenghtTrajectory];
        
        %         end
        
    end
    
    
    MetricsMatrix(:,1,how) = metrics_deltaTime;
    MetricsMatrix(:,2,how) = metrics_containmentRate;
    MetricsMatrix(:,3,how) = metrics_spreadNormalized;
    MetricsMatrix(:,4,how) = metrics_spread;
    MetricsMatrix(:,5,how) = metrics_distance;
    MetricsMatrix(:,6,how) = metrics_distanceTravelled(:,1);
    MetricsMatrix(:,7,how) = metrics_distanceTravelled(:,2);
    
    metrics(how).table = table(metrics_deltaTime, metrics_containmentRate, metrics_spreadNormalized, metrics_spread, metrics_distance, metrics_distanceTravelled);
    metrics(how).name = howSearch;
    metrics(how).cont_successfullTrials = cont_successfull_trials;
    metrics(how).successfullTrials = successfull_trials;
    
end




save(['Metrics\Metrics_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat'],'metrics');


end
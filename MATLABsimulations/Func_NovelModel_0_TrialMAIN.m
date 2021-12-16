function Func_NovelModel_0_TrialMAIN(TrialNumberTot,TargetNumberTot,ParamTot,HowDivisionTot)

% TrialNumberTot(2) = 10;
% TargetNumberTot(2) = 120;
% ParamTot(2) = 50;
% HowDivisionTot(2) = 4;


TotParam = ParamTot(2) - ParamTot(1) + 1;
TotTargets = TargetNumberTot(2) - TargetNumberTot(1) + 1;
TotTrials = TotParam * TotTargets;
TargetNumberCont = 1;

for TargetNumber = TargetNumberTot(1) : 1 : TargetNumberTot(2)
    
    for RepulMultiplier = ParamTot(1) : ParamTot(2)
        
        % initialize parameters for all division and trials repetition
        Func_NovelModel_B_INIT(TargetNumber,RepulMultiplier);
        
        parfor how = HowDivisionTot(1) : HowDivisionTot(2)
            
            for TrialNumber = TrialNumberTot(1) : TrialNumberTot(2)
                
                Func_NovelModel_A_MAIN(TrialNumber,TargetNumber,RepulMultiplier,how);
                
            end
            
        end
        
        % get average metrics
        MetricsMatrix = getMetrics(TargetNumber,RepulMultiplier,TrialNumberTot,HowDivisionTot,1);
        
        % add average metrics to a metrics
        AverageMetrics_ContTime_Global(TargetNumberCont,RepulMultiplier) = mean(MetricsMatrix(:,1,1));
        AverageMetrics_ContTime_Static(TargetNumberCont,RepulMultiplier) = mean(MetricsMatrix(:,1,2));
        AverageMetrics_ContTime_LeaderFollower(TargetNumberCont,RepulMultiplier) = mean(MetricsMatrix(:,1,3));
        AverageMetrics_ContTime_PeerToPeer(TargetNumberCont,RepulMultiplier) = mean(MetricsMatrix(:,1,4));
        
        delete(['Parameters\param_',num2str(TargetNumber),'_',num2str(RepulMultiplier),'.mat']);
        
        % delete trials used
        delete('Trials\Global\*.mat');
        delete('Trials\Static\*.mat');
        delete('Trials\PeerToPeer\*.mat');
        delete('Trials\LeaderFollower\*.mat');
        %
    end
    
    TargetNumberCont = TargetNumberCont + 1;
    
    
end

save('AverageContainmentTime.mat','AverageMetrics_ContTime_Global','AverageMetrics_ContTime_LeaderFollower','AverageMetrics_ContTime_PeerToPeer','AverageMetrics_ContTime_Static');
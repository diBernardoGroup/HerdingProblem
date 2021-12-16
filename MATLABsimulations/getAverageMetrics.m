function AverageMetrics_ContTime = getAverageMetrics(MetricsMatrix,TargetNumberCont,RepulMultiplier,HowDivisionTot)

% change the output of the function to have the average of different
% metrics

for how = HowDivisionTot(1) : HowDivisionTot(2)
    
    AverageMetrics_ContTime(TargetNumberCont,RepulMultiplier,how) = mean(MetricsMatrix(:,1,how));
    
    AverageMetrics_ContRate(TargetNumberCont,RepulMultiplier,how) = mean(MetricsMatrix(:,2,how));
    
    AverageMetrics_Spread(TargetNumberCont,RepulMultiplier,how) = mean(MetricsMatrix(:,4,how));
    
    AverageMetrics_Distance(TargetNumberCont,RepulMultiplier,how) = mean(MetricsMatrix(:,5,how));
    
    
end



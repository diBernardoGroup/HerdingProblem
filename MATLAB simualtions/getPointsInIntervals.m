function points = getPointsInIntervals(TargetNumberTot,ParamTot,HowDivisionTot,fileName)

load(fileName);

topParam1 = TargetNumberTot(2)/1; 
topParam2 = ParamTot(2); 

points_00_30 = zeros(1,HowDivisionTot(2)); 
points_30_60 = zeros(1,HowDivisionTot(2)); 
points_60_75 = zeros(1,HowDivisionTot(2)); 
points_75_100 = zeros(1,HowDivisionTot(2)); 

for how = HowDivisionTot(1) : HowDivisionTot(2)
    
    val_3 = AveragesTaskDivisions.AverageMetrics_ContTime(:,:,how);
    val_3 = val_3 * 100 / 60;

    for rows = 1 : 50
        for cols = 1 : 4 
            
            if val_3(rows,cols) <= 30
                
                points_00_30(1,how) = points_00_30(1,how) + 1 ;
                
            elseif val_3(rows,cols) > 30 && val_3(rows,cols)<= 60
                
                points_30_60(1,how) =  points_30_60(1,how) + 1 ;
                
                
            elseif val_3(rows,cols) > 60 && val_3(rows,cols)<= 75
                
                points_60_75(1,how) = points_60_75(1,how) + 1 ;
            
            elseif val_3(rows,cols) > 75
                
                points_75_100(1,how) = points_75_100(1,how) + 1 ;

            end
            
            
        end
    end
    
   points.below30percent = points_00_30/(topParam1*topParam2) * 100; 
   points.between30and60percent = points_30_60/(topParam1*topParam2) * 100; 
   points.between60and75percent = points_60_75/(topParam1*topParam2) * 100; 
   points.above75percent = points_75_100/(topParam1*topParam2) * 100; 
    
    
    
end
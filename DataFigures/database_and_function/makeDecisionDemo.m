function index = makeDecisionDemo(pcData_ins1, pcData_ins2, pcData_obs, threshold,pos1,pos2)
% revised at 3/27/2022
% return which sensor is not under adversary
% && && && better method?
% have to use score to value it
% scanIMU = pcData_imu;
% scanGPS = pcData_gps;
% scanLiDAR = pcLidar;
% threshold = [2 1.5 0.5];
%     maxrange = 90;



    threshold_score = 375;
    index = 0; cellsizes = 5;
    diff = pos1 - pos2;       
    if abs(diff(1)) <= threshold(1) && abs(diff(2)) <= threshold(2)
    
        index = 3; % [1 2] are all correct
        
        return
    
    end
%     [pose21,stats1] = matchScans(pcData_ins1,pcData_obs,'CellSize',cellsizes) ;
    [~, pose22, score1,~] = kickLidar(pcData_obs,pcData_ins1);
    score1
    
%     [pose31,stats2] = matchScans(pcData_ins2,pcData_obs,'CellSize',cellsizes);
    [~, pose32, score2,~] = kickLidar(pcData_obs,pcData_ins2);
    score2
    
    %
%     INS1 =  threshold(1)>=abs(pose22(1)) && threshold(2)>=abs(pose22(2)) && ...
%         threshold(3)>= abs(pose22(3));
    if score1 >= threshold_score && score2 >= threshold_score
        disp('LiDAR is under attack')
        threshold_score = 400;    
    else
        disp('LiDAR is not under attack')
    end

    INS1 = threshold(1)>=abs(pose22(1)) && threshold(2)>=abs(pose22(2)) && ...
        threshold(3)>= abs(pose22(3)) && score1 <= threshold_score;
    

    if INS1 == 1
        index = 1;
    else
        index = index;
    end
    
    INS2 = threshold(1)>=abs(pose32(1)) && threshold(2)>=abs(pose32(2)) && ...
        threshold(3)>= abs(pose32(3)) && score2 <= threshold_score;
    if INS2 == 1
        index = 2;
    else
        index = index;
    end
    if index == 0
        if score1 < score2
            index = 1;
        else
            index = 2;
        end
    end



end
function index = makeDecisionDemo(pcData_ins1, pcData_ins2, pcData_obs, threshold)
% return which sensor is not under adversary
% && && && better method?
% have to use score to value it
% scanIMU = pcData_imu;
% scanGPS = pcData_gps;
% scanLiDAR = pcLidar;
% threshold = [0.5 0.5 0.5];
    maxrange = 90;
    range_raw = pcData_ins1.Ranges;
    angle_raw = pcData_ins1.Angles;
    lenr = length(pcData_ins1.Ranges);
    for i = 1:lenr
        if range_raw(i) >= maxrange
            range_raw(i) = NaN;
        end
    end
    pcData_ins1 = lidarScan(range_raw,angle_raw );
    
    range_raw = pcData_ins2.Ranges;
    angle_raw = pcData_ins2.Angles;
    lenr = length(pcData_ins2.Ranges);
    for i = 1:lenr
        if range_raw(i) >= maxrange
            range_raw(i) = NaN;
        end
    end
    pcData_ins2 = lidarScan(range_raw,angle_raw );
    
    range_raw = pcData_obs.Ranges;
    angle_raw = pcData_obs.Angles;
    lenr = length(pcData_obs.Ranges);
    for i = 1:lenr
        if range_raw(i) >= maxrange
            range_raw(i) = NaN;
        end
    end
    pcData_obs = lidarScan(range_raw,angle_raw );



    index = 0; cellsizes = 6;
    pose1 = matchScans(pcData_ins1,pcData_ins2,'CellSize',cellsizes);
    crit = sqrt(pose1*pose1');
    % disp(pose1)
    if threshold(1)>=abs(pose1(1)) && threshold(2)>=abs(pose1(2))&& threshold(3)>= abs(pose1(3)) && crit > 0.0001
        index = 3; % INS-1 and INS-2 work well
        return
    end
    % disp('point')
    pose21 = matchScans(pcData_ins1,pcData_obs,'CellSize',cellsizes) ;
    [~, pose22, ~,~, ~,~,~] = kickLidar(pcData_obs,pcData_ins1);
  
    
    pose31 = matchScans(pcData_ins2,pcData_obs,'CellSize',cellsizes);
    [~, pose32, ~,~, ~,~,~] = kickLidar(pcData_obs,pcData_ins2);
    
    %
    IMU =  threshold(1)>=abs(pose21(1)) && threshold(2)>=abs(pose21(2)) && ...
        threshold(3)>= abs(pose21(3));
    IMU_tilde = threshold(1)>=abs(pose22(1)) && threshold(2)>=abs(pose22(2)) && ...
        threshold(3)>= abs(pose22(3));
    if IMU_tilde == 1
        index = 1;
    else
        index = index;
    end
    GPS =  threshold(1)>=abs(pose31(1)) && threshold(2)>=abs(pose31(2)) && ...
        threshold(3)>= abs(pose31(3));
    GPS_tilde = threshold(1)>=abs(pose32(1)) && threshold(2)>=abs(pose32(2)) && ...
        threshold(3)>= abs(pose32(3));
    if GPS_tilde == 1
        index = 2;
    else
        index = index;
    end




end
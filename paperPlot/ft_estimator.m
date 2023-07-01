function index = ft_estimator(scanIMU, scanGPS, scanLiDAR, threshold)
% return which sensor is not under adversary
% && && && better method?
% have to use score to value it
% scanIMU = pcData_imu;
% scanGPS = pcData_gps;
% scanLiDAR = pcLidar;
% threshold = [0.5 0.5 0.5];

index = [];
pose1 = matchScans(scanGPS,scanIMU)

% disp(pose1)
if threshold(1)>=abs(pose1(1)) && threshold(2)>=abs(pose1(2))&& threshold(3)>= abs(pose1(3))
    index = [1,2];
    return
end
% disp('point')
pose21 = matchScans(scanIMU,scanLiDAR) 
[angle2, pose22, ~,~, ~,~] = kickLidar(scanLiDAR, scanIMU)
% pose22
disp(pose21)
pose31 = matchScans(scanGPS,scanLiDAR)
[angle3, pose32, ~,~, ~,~] = kickLidar(scanLiDAR, scanGPS)
% pose32
% 
IMU =  threshold(1)>=abs(pose21(1)) && threshold(2)>=abs(pose21(2)) && ...
    threshold(3)>= abs(pose21(3));
IMU_tilde = threshold(1)>=abs(pose22(1)) && threshold(2)>=abs(pose22(2)) && ...
    threshold(3)>= abs(pose22(3));
if IMU_tilde == 1
    index = [index 1];
else
    index = [];
end
GPS =  threshold(1)>=abs(pose31(1)) && threshold(2)>=abs(pose31(2)) && ...
    threshold(3)>= abs(pose31(3));
GPS_tilde = threshold(1)>=abs(pose32(1)) && threshold(2)>=abs(pose32(2)) && ...
    threshold(3)>= abs(pose32(3));
if GPS_tilde == 1
    index = [index 2];
else
    index = index;
end
  



% end
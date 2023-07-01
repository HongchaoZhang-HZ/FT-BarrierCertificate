clear; close all; clc

load pointCloud.mat pointCloud
load location.mat location_log_radar
load orientation.mat ori_log_radar
load mapdataDemo.mat mapmatrix % waypoint = [80 100];

% real position
waypoint = 90; maxrange = 30; advinfo = [10 15 -100 -90];noise = 0.5;%
cellsizes = 5; iterations = 30;

pose_waypoint = location_log_radar.signals.values(:,:,waypoint);
ori = ori_log_radar.signals.values(:,:,waypoint);
pose_waypoint(1,3) =  ori(1,3);
pose_waypoint
pcDataRaw = pointCloud.signals.values(:,:,:,waypoint);
xlist = pcDataRaw(17,:,1);
ylist = pcDataRaw(17,:,2);
pcData = lidarScan([xlist', ylist']);
lenr = length(pcData.Ranges);

range_raw = pcData.Ranges + (-noise/2) + noise*rand(lenr,1);
for i = 1:lenr
    if range_raw(i) >= maxrange
        range_raw(i) = NaN;
    end
end
angle_raw = pcData.Angles;
pcData = lidarScan(range_raw,angle_raw );



[pcData2] = adversaryDemo(pcData, advinfo);




[pose_1, score1] = matchScans(pcData,pcData,'CellSize',cellsizes,'MaxIterations',iterations);

[pose_2, score2] = matchScans(pcData2,pcData,'CellSize',cellsizes,'MaxIterations',iterations);
disp('-------------------------adversary test-------------------------------')
disp(score1.Score)
disp(score2.Score)
disp(pose_1)
disp(pose_2)

%-----------------
pcDataRaw3 = pointCloud.signals.values(:,:,:,waypoint-1);
xlist3 = pcDataRaw3(17,:,1);
ylist3 = pcDataRaw3(17,:,2);
pcData3 = lidarScan([xlist3', ylist3']);
lenr = length(pcData.Ranges);

range_raw2 = pcData3.Ranges + (-0/2) + 0*rand(lenr,1);
for i = 1:lenr
    if range_raw2(i) >= maxrange
        range_raw2(i) = NaN;
    end
end
angle_raw2 = pcData3.Angles;
pcData3 = lidarScan(range_raw2,angle_raw2 );


[pose_3, score3] = matchScans(pcData3,pcData,'CellSize',cellsizes,'MaxIterations',iterations);
[pose_4, score4] = matchScans(pcData3,pcData2,'CellSize',cellsizes,'MaxIterations',iterations);
disp(score3.Score)
disp(score4.Score)
disp(pose_3)
disp(pose_4)
disp('-------------------------kick-------------------------------')
angle_step = 30;


% [angle, pose, score,pcd1, pcd2, scores] = kickLidar(pcData, pcData2,false);
% disp(angle)
% disp(pose)
% disp(score)





disp('-------------------------TABLE-------------------------------')


% real pcData
% lidar pcData2
% transScan = transformScan(scan,relPose)
relPose1 = [ 1.5 0 0];
relPose2 = [ 0.3 0 0];
% pcData_imu = transformScan(pcData,relPose1);
% pcData_gps = transformScan(pcData,relPose2);
% change to estimate
pose_imu = pose_waypoint + relPose1;
pose_gps = pose_waypoint + relPose2;
maxlidarrange = 30;
pcData_imu = pcEst(pose_imu,mapmatrix,maxlidarrange);
pcData_gps = pcEst(pose_gps,mapmatrix,maxlidarrange);
pcData_est = pcEst(pose_waypoint,mapmatrix,maxlidarrange);






pcLidar = pcData2;
threshold = [0.4 0.4 0.4];
TrustSensor = makeDecisionDemo(pcData_imu, pcData_gps, pcLidar, threshold)
if TrustSensor == 1
    disp('Trust IMU')
elseif TrustSensor == 2
    disp('Trust GPS')
elseif TrustSensor == [1 2]
    disp('Trust IMU and GPS')
else
    disp('Both IMU and GPS are under attack.')
end

disp('finish')

% compare two scan
figure(1)
plot(pcData.Cartesian(:,1),pcData.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
title('LiDAR Scan without Adversary')


figure(2)
hold on
plot(pcData.Cartesian(:,1),pcData.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
plot(pcData2.Cartesian(:,1),pcData2.Cartesian(:,2),...
    '.','MarkerSize',3,'color','b')
legend('Original Scan','Under Adversary Scan')
title('LiDAR with and without Adversary')
hold off

figure(3)
hold on
plot(pcd1.Cartesian(:,1),pcd1.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
plot(pcd2.Cartesian(:,1),pcd2.Cartesian(:,2),...
    '.','MarkerSize',3,'color','b')
legend('Original Scan','Under Adversary Scan')
title('LiDAR Scan Reconstruction')
hold off

figure(4)
hold on
plot(pcData_imu.Cartesian(:,1),pcData_imu.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
plot(pcData_gps.Cartesian(:,1),pcData_gps.Cartesian(:,2),...
    '.','MarkerSize',3,'color','b')
legend('imu LiDAR scan','GPS LiDAR scan')
hold off

figure(5)
hold on
plot(pcData_est.Cartesian(:,1),pcData_est.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
title('Estimated LiDAR scan Based on Position')
hold off

figure(6)
hold on
plot(pcData_imu.Cartesian(:,1),pcData_imu.Cartesian(:,2),...
    '.','MarkerSize',3,'color','b')
plot(pcData_gps.Cartesian(:,1),pcData_gps.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
plot(pcLidar.Cartesian(:,1),pcLidar.Cartesian(:,2),...
    '.','MarkerSize',3,'color','g')
legend('IMU estimation','GPS estimation', 'LiDAR')
title('IMU and LiDAR are under attack')

hold off
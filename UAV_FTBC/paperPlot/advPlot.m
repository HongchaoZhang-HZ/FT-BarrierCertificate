% This script is used to plot figures in section 1;
% This section has two scenarios
% Scenario 1: one INS is under attack; Another one INS and LiDAR work well.
% Scenario 2: one INS and LiDAR are under attack; Another one INS works
% well.
clear; close all; clc
% UAVstate time interval is 0.01s, it has 5601 points;
% LiDar data time interval is 0.5s, it has 113 points;
% select the 90th points in LiDAR log as a reference;
% the related points in UAVstate should be 1+89*50 = 4501 
load uavState.mat UAVstate
load pointCloud.mat
load mapdataDemo.mat mapmatrix
maxlidarrange = 90; % lidar parameter
posInfo = UAVstate{1}.Values.pos_vel;
pos_x = posInfo.x.Data; pos_y = -posInfo.y.Data; pos_z = -posInfo.z.Data;
v_x = posInfo.vx.Data; v_y = posInfo.vy.Data; v_z = posInfo.vz.Data;
trueOri = UAVstate{1}.Values.attitude;
trueOrien = [trueOri.roll.Data -trueOri.pitch.Data -trueOri.yaw.Data];
% when select point #4451
pointNumber = 4451; 
pointNumber2 = (pointNumber-1)/50+1;
truePosition = [pos_x(pointNumber) pos_y(pointNumber) pos_z(pointNumber)];
trueVelocity = [v_x(pointNumber) v_y(pointNumber) v_z(pointNumber)];
trueOrientation = trueOrien(pointNumber,:);
% [position, velocity, orientation] = simINS(truePosition,trueVelocity,trueOrientation);
disp("The real position is:")
disp(truePosition)
%--------------------------------------------------------------------------
% Scenario 1: INS-1 is under attack; INS-2 and LiDAR work well.
%--------------------------------------------------------------------------
% real LiDAR scan 
pcDataRaw = pointCloud.signals.values(:,:,:,pointNumber2);
xlist = pcDataRaw(17,:,1);
ylist = pcDataRaw(17,:,2);
pcData = lidarScan([xlist', ylist']);
% Position1 under attack + [0 10 0]
[position1, velocity1, orientation1] = simINS(...
    truePosition,trueVelocity,trueOrientation);
position1 = position1 + [0 10 0]; % position of INS1
% get pos1 and pos2 [x, y, yaw]
pose_ins1 = [position1(1) position1(2) orientation1(3)];
[position2, velocity2, orientation2] = simINS(...
    truePosition,trueVelocity,trueOrientation);% position of INS2
pose_ins2 = [position2(1) position2(2) orientation2(3)];
% estimate LiDAR scan based on INS 1
pcData_ins1 = pcEst(pose_ins1,mapmatrix,maxlidarrange);
% estimate LiDAR scan based on INS 2
pcData_ins2 = pcEst(pose_ins2,mapmatrix,maxlidarrange);

figure(1)
hold on
plot(pcData.Cartesian(:,1),pcData.Cartesian(:,2),...
    '.','MarkerSize',3,'color','m')
plot(pcData_ins1.Cartesian(:,1),pcData_ins1.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
plot(pcData_ins2.Cartesian(:,1),pcData_ins2.Cartesian(:,2),...
    '.','MarkerSize',3,'color','b')
scatter(0,0,'filled','MarkerFaceColor','b')
hold off
legend({'LiDAR','INS-1',...
    'INS-2'})
title('Scenario 1: INS-1 is under attack')
threshold = [2 2 2];
index = makeDecisionDemo(pcData_ins1, pcData_ins2, pcData, threshold);
disp('In Scenario 1: ')
if index == 1
    disp('The INS-1 works')
elseif index == 2
    disp('The INS-2 works')
elseif index == 3
    disp('INS-1 and INS-2 work')
else
    disp('No INS sensor works')
end

%--------------------------------------------------------------------------
% Scenario 2: INS-1 and Lidar are under attack; INS-2 works well.
%--------------------------------------------------------------------------
% LiDAR scan under attack
pcDataRaw = pointCloud.signals.values(:,:,:,pointNumber2);
xlist = pcDataRaw(17,:,1);
ylist = pcDataRaw(17,:,2);
pcData = lidarScan([xlist', ylist']);
% random attack generates obstacle
advinfo = [10 15 -90 -80];
% generate obstacle in range of 10 to 15 meters, 
% in angle of -100 to -90 degrees.
[pcData_obs] = adversaryDemo(pcData, advinfo);
figure(2)
set(gcf,'position',[200 200 1000 400])
subplot(1,2,1);
hold on
scatter(0,0,'filled','MarkerFaceColor','b')
plot(pcData_obs.Cartesian(:,1),pcData_obs.Cartesian(:,2),...
    '.','MarkerSize',3,'color','m')
plot(pcData_ins1.Cartesian(:,1),pcData_ins1.Cartesian(:,2),...
    '.','MarkerSize',3,'color','r')
plot(pcData_ins2.Cartesian(:,1),pcData_ins2.Cartesian(:,2),...
    '.','MarkerSize',3,'color','b')
scatter(0,0,'filled','MarkerFaceColor','b')
hold off
legend({'LiDAR','INS-1',...
    'INS-2'})
title('Scenario 2: INS-1 and LiDAR are under attack')

% eliminate LiDAR obstacle
% [angle, pose, score,pcd1, pcd2, scores] = kickLidar(pcData_obs, pcData_ins1,false);

% make decision about which sensor does not work
% index = 1 for INS-1 ;2 for INS-2; 3 for both

index = makeDecisionDemo(pcData_ins1, pcData_ins2, pcData_obs, threshold);
disp('In Scenario 2: ')
if index == 1
    disp('The INS-1 works')
    pcDataSensor = pcData_ins1;
elseif index == 2
    disp('The INS-2 works')
    pcDataSensor = pcData_ins2;
elseif index == 3
    disp('INS-1 and INS-2 work')
    pcDataSensor = pcData_ins1;
else
    disp('No INS sensor works')
end

 
% show how to elimate the obstacle in LiDAR 
[angle, pose, score,pcd1, pcd2,scores,bestLabel] = kickLidar(pcDataSensor, pcData_obs);

subplot(1,2,2);

    pcDataSensor_range = pcDataSensor.Ranges;
    pcDataSensor_angle = pcDataSensor.Angles;
    pcData_obs_range = pcData_obs.Ranges;
    pcData_obs_angle = pcData_obs.Angles;
    num_step = floor(30/360*1083);
    label = 1: num_step: 1083;
    num_label = length(label); % num_label = 13;
    if bestLabel ~= num_label
        for j= num_step*(bestLabel-1)+1: num_step*bestLabel
            pcDataSensor_range(j) = NaN;
            pcData_obs_range(j) = NaN;
        end
    else
        for j = num_step*(bestLabel-1)+1: label(end)
            pcDataSensor_range(j) = NaN;
            pcData_obs_range(j) = NaN;
        end
    end
    
    pcList1 = lidarScan(pcDataSensor_range, pcDataSensor_angle);
    pcList2 = lidarScan(pcData_obs_range, pcData_obs_angle);
    hold on
    plot(pcList1.Cartesian(:,1),pcList1.Cartesian(:,2),...
        '.','MarkerSize',3,'color','b')
    plot(pcList2.Cartesian(:,1),pcList2.Cartesian(:,2),...
        '.','MarkerSize',3,'color','m')
    legend('Original Scan','Under Adversary Scan')
    title('Scenario 2: Eliminate LiDAR obstacle')
    hold off







 
function [position, velocity, orientation] = simINS(truePosition,trueVelocity,trueOrientation)
gTruth = struct('Position',truePosition,'Velocity',trueVelocity,'Orientation',trueOrientation);
INS = insSensor("PositionAccuracy",0.1,'YawAccuracy',0);
insMeas = INS(gTruth);
position = insMeas.Position;
velocity = insMeas.Velocity;
orientation = insMeas.Orientation;
end

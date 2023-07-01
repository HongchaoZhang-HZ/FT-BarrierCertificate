clear; close all; clc
% this script was wroten at 3/27/2022
% score_delta_threshold = 350
% maxlidarrange = 90 is better than 80
addpath database_and_function\
load pointCloud_ins2.mat pointcloud_ins2
load worldPosition_ins_unattack.mat worldPosition_ins_unattack
load worldVelocity_ins_unattack.mat worldVelocity_ins_unattack
load worldEule_ins_unattack.mat worldEuler_ins_unattack
load mapdataDemo.mat mapmatrix % waypoint = [80 100];

% when y <= 55, INS sensor will get attack, x - 20 bias
% real position 76 to 113
% Lidar step interval 0.5s
pc_data_list = pointcloud_ins2.Data;
timestep = ([76:112]'-1)*0.5;
time_ins = worldPosition_ins_unattack.Time;
pos_ins = worldPosition_ins_unattack.Data;
ori_ins = worldEuler_ins_unattack.Data;
vel_ins = worldVelocity_ins_unattack.Data;
maxlidarrange = 90; cellsizes = 5; threshold = [2 1.5 0.5];
num = length(timestep);
score1s = zeros(num,1);
score2s = zeros(num,1);
score1_anchor = zeros(num,1); score1_match = zeros(num,1); 
score1_delta = zeros(num,1); score1_delta_kick = zeros(num,1);
score2_anchor = zeros(num,1); score2_match = zeros(num,1); 
score2_delta = zeros(num,1); score2_delta_kick = zeros(num,1);
waypoint = 76;
pose1 = zeros(num,3); pose2 = zeros(num,3);


% for i = 1:num
for i = 10:10
    waypoint = 75 + i;
    % ground truth LiDAR scan
    pc_data_xyz = pc_data_list(:,:,:, waypoint);
    pc_x = pc_data_xyz(17,:,1);
    pc_y = pc_data_xyz(17,:,2);
    pcLidar = lidarScan([pc_x', pc_y']);
%     % test maxlidarrange module
%     tmp = pcLidar.Ranges;
%     for j = 1: length(tmp)
%         if tmp(j) >= maxlidarrange
%             tmp(j) = NaN;
%         end
%     end
%     pcLidar = lidarScan(tmp,pcLidar.Angles);
    % ground truth pose [x y theta]
    time_index = find(time_ins == timestep(i));
    pos_tmp = pos_ins(:,:,time_index)';
    ori_tmp = ori_ins(:,:,time_index)';
    vel_tmp = vel_ins(:,:,time_index)';
    
%     % test module
%     pose_ins(1) = pose_tmp(1);
%     pose_ins(2) = - pose_tmp(2);
%     pose_ins(3) = - ori_tmp(1);
%     pc1 = pcEst(pose_ins,mapmatrix,maxlidarrange);
%     hold on 
%     plot(pc_x,pc_y,'.','Color','r')
%     plot(pc1.Cartesian(:,1),pc1.Cartesian(:,2),'.','Color','b')
    % position of ins1 (attacked)
    [pos1, vel1, ori1] = simINS(pos_tmp,vel_tmp,ori_tmp,1);
    pos1(2) = -pos1(2);
    pos1(3) = - ori1(1);
    % estimated Lidar scan based on ins1
    pcData1 = pcEst(pos1,mapmatrix,maxlidarrange);
    % position of ins2 (unattacked)
    [pos2, vel2, ori2] = simINS(pos_tmp,vel_tmp,ori_tmp,2);
    pos2(2) = - pos2(2);
    pos2(3) = - ori2(1);
    % estimated Lidar scan based on ins2
    pcData2 = pcEst(pos2,mapmatrix,maxlidarrange);
    
%     % test module 2
%     hold on 
%     plot(pc_x,pc_y,'.','Color','r')
%     plot(pcData1.Cartesian(:,1),pcData1.Cartesian(:,2),'.','Color','b')
%     plot(pcData2.Cartesian(:,1),pcData2.Cartesian(:,2),'.','Color','g')
    % delta score without kick sector
    % --------------plot Scenario 1: INS-1 is under attack----------------
    figure(1)
%     set(gcf,'position',[200 200 1000 400])
%     subplot(1,2,1);
    hold on
    p1 = plot(pcLidar.Cartesian(:,1),pcLidar.Cartesian(:,2),'.','Color','m');
    p2 = plot(pcData1.Cartesian(:,1),pcData1.Cartesian(:,2),'.','Color','b');
    p3 = scatter(0,0,90,'filled','d','MarkerFaceColor','[0.9290 0.6940 0.1250]');
%     legend({'Lidar Scan','INS1 Estimate','UAV position'},)

    [~, objh] = legend({'LiDAR Scan','INS1 Estimate','UAV Position'},'FontSize',15,...
        'FontName','Times New Roman','Location','southwest');
    %// set font size as desired
    % note that even if you plot(x,y,'.') it's a "line" plot
    objhl = findobj(objh, 'type', 'line'); %// objects of legend of type line
    set(objhl, 'Markersize', 25); %// set marker size as desired
    xlabel('[x]_1','FontSize',15,'FontName','Times New Roman')
    set(gca,'FontSize',12,'FontName','Times New Roman')
    ylabel('[x]_2','FontSize',15,'FontName','Times New Roman')
    title('LiDAR Scan and Compromised INS1 Estimate',...
        'FontSize',13,'FontName','Times New Roman')
%     title
    %     xlabel({'(a)'});

%     title('Scenario 1: INS1 is under attack')
    hold off
    figure(2)
%     subplot(1,2,2);
    hold on
    plot(pcLidar.Cartesian(:,1),pcLidar.Cartesian(:,2),'.','Color','m')
    plot(pcData2.Cartesian(:,1),pcData2.Cartesian(:,2),'.','Color','b')
%     scatter(0,0,'filled','d','MarkerFaceColor','g')
    scatter(0,0,90,'filled','d','MarkerFaceColor','[0.9290 0.6940 0.1250]');
%     [~, objh2] = legend('LiDAR Scan','INS2 Estimate');
    [~, objh2] = legend({'LiDAR Scan','INS2 Estimate','UAV Position'},...
        'FontSize',15,'FontName','Times New Roman','Location','southwest');
    %// set font size as desired
    % note that even if you plot(x,y,'.') it's a "line" plot
    objhl2 = findobj(objh2, 'type', 'line'); %// objects of legend of type line
    set(objhl2, 'Markersize', 25); %// set marker size as desired
    xlabel('[x]_1','FontSize',15,'FontName','Times New Roman')
    ylabel('[x]_2','FontSize',15,'FontName','Times New Roman')
    set(gca,'FontSize',12,'FontName','Times New Roman')
    title('LiDAR Scan and INS2 Estimate',...
        'FontSize',13,'FontName','Times New Roman')
%     title('Scenario 1: INS2 is not under attack')
    hold off
    % check which sensor is not under attack
    disp('In Scenario 1: ')
    index = makeDecisionDemo(pcData1, pcData2, pcLidar, threshold,pos1,pos2);
    
    if index == 1
        disp('The INS-1 works')
    elseif index == 2
        disp('The INS-2 works')
    elseif index == 3
        disp('INS-1 and INS-2 work')
    else
        disp('No INS sensor works')
    end


    % add random attack to LiDAR generate obstacle
    advinfo = [10 15 -70 -60]; % 10 to 15  m, -100 to -90 degree
    [pcLidar_obs] = adversaryDemo(pcLidar, advinfo);
    %     figure(2)
    figure(3)
%     set(gcf,'position',[100 100 800 400])
%     subplot(2,2,1);
    hold on
    plot(pcLidar_obs.Cartesian(:,1),pcLidar_obs.Cartesian(:,2),'.','Color','m')
    plot(pcData1.Cartesian(:,1),pcData1.Cartesian(:,2),'.','Color','b')
%     scatter(0,0,'filled','d','MarkerFaceColor','g')
    scatter(0,0,90,'filled','d','MarkerFaceColor','[0.9290 0.6940 0.1250]');
    [~, objh3] = legend({'LiDAR Scan','INS1 Estimate','UAV Position'},...
        'FontSize',15,'FontName','Times New Roman','Location','southwest');
    objhl3 = findobj(objh3, 'type', 'line'); %// objects of legend of type line
    set(objhl3, 'Markersize', 25); %// set marker size as desired
    xlabel('[x]_1','FontSize',15,'FontName','Times New Roman')
    ylabel('[x]_2','FontSize',15,'FontName','Times New Roman')
    set(gca,'FontSize',12,'FontName','Times New Roman')
    title('Spoofed LiDAR Scan and Compromised INS1 Estimate',...
        'FontSize',13,'FontName','Times New Roman')
%     xlabel({'(a)'});
%     title('Scenario 2: INS1 and LiDAR are under attack')
    hold off  
%     figure(4)
% %     subplot(2,2,3);
%     hold on
%     plot(pcLidar_obs.Cartesian(:,1),pcLidar_obs.Cartesian(:,2),'.','Color','m')
%     plot(pcData2.Cartesian(:,1),pcData2.Cartesian(:,2),'.','Color','b')
%     scatter(0,0,'filled','d','MarkerFaceColor','g')
%     legend('Lidar Scan','INS2 Estimate')
% %     xlabel({'(c)'});
%     title('Scenario 2: INS2 is not under attack')
%     hold off
    % plot kick sector result
    [~, ~, ~,~,pcData1_new,pcLidar1_new] = kickLidar_pcdata(pcData1, pcLidar_obs);
    [~, ~, ~,~,pcData2_new,pcLidar2_new] = kickLidar_pcdata(pcData2, pcLidar_obs);

    %     set(gcf,'position',[200 200 1000 400])
    %     subplot(2,2,2);
%     figure(5)
%     hold on
%     plot(pcLidar1_new.Cartesian(:,1),pcLidar1_new.Cartesian(:,2),'.','Color','m')
%     plot(pcData1_new.Cartesian(:,1),pcData1_new.Cartesian(:,2),'.','Color','b')
%     scatter(0,0,'filled','d','MarkerFaceColor','g')
%     legend('Lidar Scan','INS1 Estimate')
% %     xlabel({'(b)'});
%     title('FT Estimation based on INS1 and LiDAR')
%     hold off
    figure(6)
%     subplot(2,2,4);
    hold on
    plot(pcLidar2_new.Cartesian(:,1),pcLidar2_new.Cartesian(:,2),'.','Color','m')
    plot(pcData2_new.Cartesian(:,1),pcData2_new.Cartesian(:,2),'.','Color','b')
%     scatter(0,0,'filled','d','MarkerFaceColor','g')
    scatter(0,0,90,'filled','d','MarkerFaceColor','[0.9290 0.6940 0.1250]');
    [~, objh4] = legend({'LiDAR Scan','INS2 Estimate','UAV Position'},...
        'FontSize',15,'FontName','Times New Roman','Location','southwest');
  
    objhl4 = findobj(objh4, 'type', 'line'); %// objects of legend of type line
    set(objhl4, 'Markersize', 25); %// set marker size as desired
    xlabel('[x]_1','FontSize',15,'FontName','Times New Roman')
    ylabel('[x]_2','FontSize',15,'FontName','Times New Roman')
    set(gca,'FontSize',12,'FontName','Times New Roman')
    title('Spoofed LiDAR Scan and INS2 Estimate',...
        'FontSize',13,'FontName','Times New Roman')
%     xlabel({'(d)'});
%     title('FT Estimation based on INS2 and LiDAR')
    hold off
    disp('In Scenario 2: ')
    index = makeDecisionDemo(pcData1, pcData2, pcLidar_obs, threshold,pos1,pos2);
    
    if index == 1
        disp('The INS-1 works')
    elseif index == 2
        disp('The INS-2 works')
    elseif index == 3
        disp('INS-1 and INS-2 work')
    else
        disp('No INS sensor works')
    end











% 
%     [~, stats] = matchScans(pcData1,pcData1);
%     score1_anchor(i) = stats.Score;
%     [~, stats] = matchScans(pcLidar, pcData1);
%     score1_match(i) = stats.Score;
%     [~, stats] = matchScans(pcData2,pcData2);
%     score2_anchor(i) = stats.Score;
%     [~, stats] = matchScans(pcLidar, pcData2);
%     score2_match(i) = stats.Score;
%     score1_delta(i) = abs(score1_anchor(i) - score1_match(i));
%     score2_delta(i) = abs(score2_anchor(i) - score2_match(i));
%     % delta score with kick sector
%     [~, stats] = matchScans(pcData1,pcData1);
%     score1_anchor(i) = stats.Score;
%     [~, stats] = matchScans(pcLidar, pcData1);
%     score1_match(i) = stats.Score;
%     [~, stats] = matchScans(pcData2,pcData2);
%     score2_anchor(i) = stats.Score;
%     [~, stats] = matchScans(pcLidar, pcData2);
%     [~, pose1(i,:), score1_delta_kick(i),~] = kickLidar(pcData1, pcLidar);
%     [~, pose2(i,:), score2_delta_kick(i),~] = kickLidar(pcData2, pcLidar);
    



    













end
% time = 1:num;
% figure(1)
% plot(time,score1_delta,'r',time,score2_delta,'b')
% legend('INS1','INS2')
% title('Delta Score without kick sector')
% figure(2)
% hold on
% plot(time,score1_delta,'r',time,score1_anchor,'--r',time,score1_match,'.r')
% 
% plot(time,score2_delta,'b',time,score2_anchor,'--b',time,score2_match,'.b')
% legend('deltaScore1','score1Anchor','score1Match',...
%     'deltaScore2','score2Anchor','score2Match')
% hold off
% figure(3)
% plot(time,score1_delta_kick,'r',time,score2_delta_kick,'b')
% max(abs(pose1))
% max(abs(pose2))








function [position, velocity, orientation] = simINS(truePosition,trueVelocity,trueOrientation,ins_index)
gTruth = struct('Position',truePosition,'Velocity',trueVelocity,'Orientation',trueOrientation);
INS = insSensor('RollAccuracy',0,'PitchAccuracy',0,'YawAccuracy',0,'PositionAccuracy',[0.001 0.001 0.001],...
    'VelocityAccuracy',0,'AccelerationAccuracy',0);
insMeas = INS(gTruth);
position = insMeas.Position;
if ins_index == 1
    x =  position(1);
    y = -position(2);
    if y <= 55
        x = x - 20;
    end
    position(1) = x;
end
velocity = insMeas.Velocity;
orientation = insMeas.Orientation;
end
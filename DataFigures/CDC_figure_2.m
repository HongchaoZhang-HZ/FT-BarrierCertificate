% plot 2D map
% 03-30-2022
clear; close all; clc
addpath database_and_function\
load pointCloud.mat pointCloud
load location.mat location_log_radar
load orientation.mat ori_log_radar
load uavtrj_ft_noekf_xyz_v1.mat uavtrj_ft_noEKF_V1
load baseline_trj_attacked_x20.mat baseline_trj_attacked_x20
maxLidarRange = 90; downsample = 1;
global mapmatrix

poses = location_log_radar.signals.values;
ori = ori_log_radar.signals.values;
poses(:,3) = ori(:,3);

% set position matrix from waypoint 16 to 100
waypoint1 = 16;
waypoint2 =100;
position = zeros(waypoint2-waypoint1+1,3);
for i = waypoint1:100
    position(i-waypoint1+1,:) = location_log_radar.signals.values(:,:,i);
end
% set rotation in radians for relPose
ori = ori_log_radar.signals.values;
ori1 = zeros(waypoint2-waypoint1+1,3);
for i = waypoint1: waypoint2
    ori1(i-waypoint1+1,:) = ori(:,:,i);
end
% 
% 
position(:,3) = ori1(:,3);
% set angle matrix from back to back
angles = zeros(113,1083);
for i = 1:113
    angles(i,:) = linspace(pi, -pi, 1083);
end

% set range matrix
% for j = 17:17 % use j instead of 17
xlist = zeros(113, 1083);
ylist = zeros(113, 1083);
for i = waypoint1: 100
    test = pointCloud.signals.values(:,:,3,i);
    test = abs(test);
    test(find(isnan(test)==1)) = 0;
    test_sum = sum(test,2);
    [~,row] = min(test_sum);
    
    disp(row)

    xlist(i,:) = pointCloud.signals.values(row,:,1,i);
    ylist(i,:) = pointCloud.signals.values(row,:,2,i);
end
ranges = sqrt(xlist.^2 + ylist.^2);
scans = cell(1,85);
% set scans 1*85cell 
[~,num] = size(xlist);

for i =1:num
    b= mod(i,downsample);
    if b ~= 0
        xlist(:,i) = NaN;
        ylist(:,i) = NaN;
    end
end




for i = waypoint1: waypoint2
    scans{i-waypoint1+1} = lidarScan([xlist(i,:)',ylist(i,:)']);
end

% occMap = buildMap(scans,position,maxLidarRange,20);
% figure
% show(occMap)
% title('Occupancy Map of Garage')
hold on
trj1 = baseline_trj_attacked_x20.Data;
[~,~,num1] = size(trj1);
x1 = zeros(num1,1); y1 = zeros(num1,1);
for i = 1: num1
    pos = trj1(:,:,i);
    x1(i) = pos(1);
    y1(i) = -pos(2);
end

trj2 = uavtrj_ft_noEKF_V1.Data;
[~,~,num2] = size(trj2);
x2 = zeros(num2,1); y2 = zeros(num2,1);
for i = 1: num2
    pos = trj2(:,:,i);
    x2(i) = pos(1);
    y2(i) = -pos(2);
end

p1 = plot(x1,y1,'m--','linewidth',2,'DisplayName','Baseline');
p2 = plot(x2,y2,'b-','linewidth',2,'DisplayName','Proposed Method');
% legend('Baseline','Proposed Method','')
% legend({x1,y1,x2,y2},'DisplayName','Baseline','DisplayName','Proposed Method')
% legend('off')
% legend([p1 p2 p3])


mapmatrix = [];
helperShow(scans,position,maxLidarRange,mapmatrix,waypoint1,waypoint2,p1,p2);
% hold on 
% plot(position(:,1),position(:,2))
% hold off
% trajectory with attack, no FT
% ---------------------plot trajectory----------------------------------
% hold on
% trj1 = baseline_trj_attacked_x20.Data;
% [~,~,num1] = size(trj1);
% x1 = zeros(num1,1); y1 = zeros(num1,1);
% for i = 1: num1
%     pos = trj1(:,:,i);
%     x1(i) = pos(1);
%     y1(i) = -pos(2);
% end

% trajectory with attack, with FT
trj2 = uavtrj_ft_noEKF_V1.Data;
[~,~,num2] = size(trj2);
x2 = zeros(num2,1); y2 = zeros(num2,1);
for i = 1: num2
    pos = trj2(:,:,i);
    x2(i) = pos(1);
    y2(i) = -pos(2);
end

% plot(x1,y1,'m--','linewidth',2,'DisplayName','Baseline')
% plot(x2,y2,'b-','linewidth',2,'DisplayName','Proposed Method')
% legend('Baseline','Proposed Method')
% legend()


rectangle('Position',[-101,-100,50,300],'FaceColor','red') 
axis([[-190 -85 0 130]])
txt = ['Unsafe Area'];
text(-92,95,txt,'FontSize',25,'Color','w','Rotation',270,'FontWeight','bold','FontName','Times New Roman')
hold off



function helperShow(scans1,position,maxRange,mapmatrix,waypoint1,waypoint2,p1,p2)
global mapmatrix    
hold on
%     for i = 1:numel(scans1)
    for i = 1:waypoint2-waypoint1+1
   
        
        sc = transformScan(scans1{i}.removeInvalidData('RangeLimits',[0.02 maxRange]), ...
            position(i,:));
        scPoints = sc.Cartesian;
        map_append = [scPoints(:,1),scPoints(:,2)];
        mapmatrix = [mapmatrix;map_append];
%         scatter(scPoints(:,1),scPoints(:,2),'.','black')
        if i ~= waypoint2-waypoint1+1
            p3 = plot(scPoints(:,1),scPoints(:,2),'.','MarkerSize',3,'color','k','HandleVisibility','off');
        else
            p3 = plot(scPoints(:,1),scPoints(:,2),'.','MarkerSize',3,'color','k',...
                'DisplayName','Map');

            [~, objh3] = legend([p1 p2 p3],'FontSize',15,'FontName','Times New Roman');
            objhl3 = findobj(objh3, 'type', 'line'); %// objects of legend of type line
            set(objhl3, 'Markersize', 25); %// set marker size as desired
            title('Comparison of UAV Trajectories Under Attack','FontSize',15,'FontName','Times New Roman')
            
            
        end
        
    end
    hold off
    xlabel('[x]_1','FontSize',15,'FontName','Times New Roman')
    ylabel('[x]_2','FontSize',15,'FontName','Times New Roman')
%     set(gca,'FontSize',11,'FontName','Times New Roman')
    
   
end


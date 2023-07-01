clear; close all; clc
addpath database_and_function\
load pointCloud.mat pointCloud
load location.mat location_log_radar
load orientation.mat ori_log_radar
load uavtrj_ft_noekf_xyz_v1.mat uavtrj_ft_noEKF_V1
% baseline 
time = uavtrj_ft_noEKF_V1.Time;
data = uavtrj_ft_noEKF_V1.Data;
[~,~,num] = size(data);
time1 = 0;
time2 = 0;
for i = 1:num
    pos = data(:,:,i);
    y = -pos(2);
    if y<= 55
        disp(i)
        time1 = i;
        break

    end
end
time1 = time(time1)

% get attack 37.2550 s
% for j = 1:num
%     pos = data(:,:,j);
%     x = pos(1);
%     if x>= -101
%         disp(j)
%         time2 = j;
%         break
%     end
% end
% % enter unsafe area 46.955 s
% time2 = time(time2)
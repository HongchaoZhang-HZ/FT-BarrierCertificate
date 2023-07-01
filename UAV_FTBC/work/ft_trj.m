% plot uav trj under attack, using FT algorithm
load uavtrj_ft_noekf_xyz_v1.mat uavtrj_ft_noEKF_V1

trj_data = uavtrj_ft_noEKF_V1;
[~,~,num] = size(trj_data.Data);
x = zeros(num,1); y = zeros(num,1);
for i = 1: num
    pos = trj_data.Data(:,:,i);
    x(i) = pos(1);
    y(i) = - pos(2);
end

plot(x,y)
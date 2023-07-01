load uavtrj_ft_noekf_xyz_v1.mat uavtrj_ft_noEKF_V1
load baseline_trj_attacked_x20.mat baseline_trj_attacked_x20
% trajectory with attack, no FT
trj1 = baseline_trj_attacked_x20.Data;
[~,~,num1] = size(trj1);
x1 = zeros(num1,1); y1 = zeros(num1,1);
for i = 1: num1
    pos = trj1(:,:,i);
    x1(i) = pos(1);
    y1(i) = -pos(2);
end

% trajectory with attack, with FT
trj2 = uavtrj_ft_noEKF_V1.Data;
[~,~,num2] = size(trj2);
x2 = zeros(num2,1); y2 = zeros(num2,1);
for i = 1: num2
    pos = trj2(:,:,i);
    x2(i) = pos(1);
    y2(i) = -pos(2);
end

plot(x1,y1,'r',x2,y2,'b','linewidth',2)
legend('Without FT Control','With FT Control')
hold on

rectangle('Position',[-101,10,20,100],'FaceColor','red') 
axis([[-190 -91 10 110]])
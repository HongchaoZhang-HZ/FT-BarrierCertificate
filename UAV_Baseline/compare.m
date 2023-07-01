
true_location = location_log_radar.signals.values;
gps_location = final_lla.signals.values;
x1 = zeros(1,113); y1 = zeros(1,113); z1 = zeros(1,113);%true
x2 = zeros(1,113); y2 = zeros(1,113); z2 = zeros(1,113);%gps
for i = 1: 113
    a = true_location(:,:,i);
    x1(i) = a(1);
    y1(i) = a(2);
    z1(i) = a(3);
    b = gps_location(:,:,i);
    x2(i) = b(1);
    y2(i) = b(2);
    z2(i) = b(3);
end

figure(1)
plot3(x1,y1,z1)
hold on
plot3(x2,y2,z2)
legend('Ground Truth','GPS Location')
hold off


load('uav_p_2ddt.mat')
A = uav_p_2ddt.A;
B = uav_p_2ddt.B;
C = uav_p_2ddt.C;

A = C*A*inv(C);
B = C*B;
C = eye(2);

I = eye(2);
Kp = [0.25,0.35]';
Kd = [0.8,0.8]';

Abar = inv(I-B*Kd)*A;
Bbar = inv(I-B*Kd)*(B*Kp-B*Kd);
% Abar+Bbar

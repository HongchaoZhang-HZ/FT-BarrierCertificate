
load('uav_p_2ddt.mat');
sys = ss(uav_p_2ddt);
sys = ss(sys.C*sys.A*inv(sys.C), sys.C*sys.B, eye(2), zeros(2),0.005);
% global A
A = sys.A;
% global B 
B = sys.B;
num_steps = 6781;
% global timestep
timestep = 1;
n = 2;
m = 2;

% global P 
P = zeros([n,n,num_steps]);
% s = zeros(n, num_steps);
% global R
R = 1e-2*eye(2);


for i = num_steps-1:-1:1
%     P(:,:,i) = P(:,:,i+1) + dt*(A'*P(:,:,i+1) + P(:,:,i+1)*A - P(:,:,i+1)*B*R_inv*B'*P(:,:,i+1) + Q); % negative
    P(:,:,i) = A'*P(:,:,i+1)*A - A'*P(:,:,i+1)*B* inv(R+B'*P(:,:,i+1)*B)*B'*P(:,:,i+1)*A;

%     dsdt = (A' - P(:,:,i)*B*R_inv*B')'*s(:,i+1) - Q*ref_traj(:,i+1);
    %dsdt = (A - B*R_inv*B'*P(:,:,i))'*s(:,i+1) - Q*ref_traj(:,i+1);
%     s(:,i) = s(:,i+1) + dsdt*dt;
end

r = [r_X.Data'; r_Y.Data'];
size(r)
% global ud
ud = zeros([m,num_steps]);
size(ud)
for i = 2:1:num_steps
    ud(:,i) = r(:,i)-r(:,i-1);
end
Gain = zeros([2,2,num_steps]);
for i = 2:1:num_steps
    disp(i)
    Gain(:,:,i) = -inv(R+B'*P(:,:,timestep)*B)*B'*P(:,:,timestep)*A;
end
% u_out = -inv(R+B'*P(:,:,timestep)*B)*B'*P(:,:,timestep)*A*e+ud(:,timestep);

u_1 = [0.5,1 1];
u_2 = [0,1 1];
u1 = u_1(1:2)';
u2 = u_2(1:2)';
index = 2;
xi = 0.6;
m = 2;
R = eye(m);
% u_alpha = u1; 
options = optimoptions(@fmincon,'Display','off','Algorithm','sqp',...
    'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);

% constraint_matrix{1} = 2*eye(m);
% constraint_coefficient{1} = [0, 0]';
% constraint_constant{1} = [0, 0]';
% acc_gps1>acc_gps2

constraint_matrix = cell([1,2]);
constraint_matrix1 = cell(1);
constraint_matrix2 = cell(1);
constraint_coefficient =cell([1,2]);
constraint_coefficient1=cell(1);
constraint_coefficient2=cell(1);
constraint_constant = cell([1,2]);
constraint_constant1 = cell(1);
constraint_constant2 = cell(1);
constraint_matrix{1} = 2*eye(m);
constraint_coefficient{1} = -2*u1;
constraint_constant{1} = u1'*u1-xi^2;
constraint_matrix{2} = 2*eye(m);
constraint_coefficient{2} = -2*u1;
constraint_constant{2} = u1'*u1-xi^2;

% constraint_matrix1{1} = 2*eye(m);
% constraint_coefficient1{1} = -2*u1;
% constraint_constant1{1} = u1'*u1-xi^2;
% constraint_matrix2{1} = 2*eye(m);
% constraint_coefficient2{1} = -2*u2;
% constraint_constant2{1} = u2'*u2-xi^2;
% constr_mat = [constraint_matrix1,constraint_matrix2];
% constr_coeff = [constraint_coefficient1, constraint_coefficient1];
% constr_const = [constraint_constant1, constraint_constant2];
% 
% constraint_matrix{1} = constr_mat(1);
% constraint_coefficient{1} = constr_coeff(1);
% constraint_constant{1} = constr_const(1);
% constraint_matrix{2} = constr_mat[2];
% constraint_coefficient{2} = constr_coeff(2);
% constraint_constant{2} = constr_const(2);

fun1 = @(z) quadobj(z,2*R,-2*u1,u1'*u1);
fun2 = @(z) quadobj(z,2*R,-2*u2,u2'*u2);
nonlconstr = @(z) quadconstr(z,constraint_matrix,constraint_coefficient,constraint_constant);
x0 = [0;0]; % column vector
% x0 = start_point;
[u_ast,fval,eflag,output,lambda] = fmincon(fun1,x0,...
    [],[],[],[],[],[],nonlconstr,options);

% u_out = u_ast;

if eflag == 1 && index == 3
    u_out = [u_ast',u_1(3)]';
elseif index == 1
    constraint_matrix1{1} = 2*eye(m);
    constraint_coefficient1{1} = -2*u1;
    constraint_constant1{1} = u1'*u1-xi^2;
    nonlconstr = @(z) quadconstr(z,constraint_matrix1,constraint_coefficient1,constraint_constant1);
    [u_ast,fval,eflag,output,lambda] = fmincon(fun1,x0,...
        [],[],[],[],[],[],nonlconstr,options);
    u_out = [u_ast',u_1(3)]';
elseif index == 2
    constraint_matrix2{1} = 2*eye(m);
    constraint_coefficient2{1} = -2*u2;
    constraint_constant2{1} = u2'*u2-xi^2;
    nonlconstr = @(z) quadconstr(z,constraint_matrix2,constraint_coefficient2,constraint_constant2);
    [u_ast,fval,eflag,output,lambda] = fmincon(fun2,x0,...
        [],[],[],[],[],[],nonlconstr,options);
    u_out = [u_ast',u_2(3)]';
end
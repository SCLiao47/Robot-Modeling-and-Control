% model for 2DOF planer rovolute joints robot
%
% x - right 
% y - up
%
% Reference configuration is aline to right!
clear;
addpath(genpath('Utils\'));


%% gravity
syms g real
% Gvec = g*[0 0 1 0]';
Gvec = g*[0 1 0 0]'; % 

%% links parameter setup
syms m1 m2 real 
syms I1 I2 real 
syms l1 l2 real
syms lc1 lc2 real   % center of mass
syms b c real

% link dimension
b = b;        % width
c = c;          % depth

% b = 0.2;
% c = 0;
% m1 = 2;
% m2 = 2;
% l1 = 1;
% l2 = 1;

Link1.l = l1;
Link1.lc = lc1;

Link2.l = l2;
Link2.lc = lc2;

% mass and Inertia
Link1.m = m1;
Link1.Ixx = Link1.m/12*(b^2+c^2);
Link1.Iyy = Link1.m/12*(Link1.l^2+c^2);
% Link1.Izz = Link1.m/12*(Link1.l^2+b^2); 
Link1.Izz = I1;

Link1.M = [Link1.m*eye(3), zeros(3);
           zeros(3), diag([Link1.Ixx, Link1.Iyy, Link1.Izz])];

Link2.m = m2;
Link2.I = I2;
Link2.Ixx = Link2.m/12*(b^2+c^2);
Link2.Iyy = Link2.m/12*(Link2.l^2+c^2);
% Link2.Izz = Link2.m/12*(Link2.l^2+b^2);
Link2.Izz = I2;

Link2.M = [Link2.m*eye(3), zeros(3);
           zeros(3), diag([Link2.Ixx, Link2.Iyy, Link2.Izz])];


%% forward-kinematics
syms th1 th2 real
syms thd1 thd2 real
syms thdd1 thdd2 real
th = [th1, th2]';
thd = [thd1, thd2]';
thdd = [thdd1, thdd2]';

Link1.q = [0 0 0]';
Link1.w = [0 0 1]';
Link1.P_l = [Link1.l 0 0 1]';
Link1.P_lc = [Link1.lc 0 0 1]';

Link2.q = [Link1.l 0 0]';
Link2.w = [0 0 1]';
Link2.P_l = [Link1.l+Link2.l 0 0 1]';
Link2.P_lc = [Link1.l+Link2.lc 0 0 1]';

xi1 = Revolute2Xi(Link1.q, Link1.w);
xi2 = Revolute2Xi(Link2.q, Link2.w);

e1 = TwistExp(xi1, th1);
e2 = TwistExp(xi2, th2);

% position of the tip for each link
q_sp1 = e1*Link1.P_l;
q_sp2 = simplify(e1*e2*Link2.P_l);

% position of the center of mass for each link
q_sp1c = e1*Link1.P_lc;
q_sp2c = simplify(e1*e2*Link2.P_lc);

g_st_ref = [1 0 0 Link1.l+Link2.l;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];

% Jacobian
g_sp10 = [1 0 0 Link1.lc;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
g_sp20 = [1 0 0 Link1.l+Link2.lc;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
     
Jb1 = BodyJacobian({xi1, th1}, g_sp10);
Jb1 = [Jb1, zeros(6,1)];

Jb2 = BodyJacobian({xi1, th1; xi2, th2}, g_sp20);

%% Energy
% Kinematic Energy
M = simplify(Jb1'*Link1.M*Jb1 + Jb2'*Link2.M*Jb2);
KE = 1/2*thd'*M*thd;

% Potential Energy
PE = Link1.m*Gvec'*q_sp1c + Link2.m*Gvec'*q_sp2c;

% Lagrangian Mechanics
L = simplify(KE - PE);

%% Manipulation equation
% Chris = @(i,j,k) 1/2*(diff(M(i,j),th(k))+diff(M(i,k),th(j))-diff(M(k,j),th(i)));
Chris = @(i,j,k) ChristoffelSymbols(i,j,k,M,th);

C11 = Chris(1,1,1)*thd1 + Chris(1,1,2)*thd2;
C12 = Chris(1,2,1)*thd1 + Chris(1,2,2)*thd2;
C21 = Chris(2,1,1)*thd1 + Chris(2,1,2)*thd2;
C22 = Chris(2,2,1)*thd1 + Chris(2,2,2)*thd2;

C = [C11, C12; C21, C22];

P = [diff(PE,th1); diff(PE,th2)];

%% EOM
EOM = simplify(Lagrange(L, [th1 thd1 thdd1 th2 thd2 thdd2]))

% To verify 
fprintf('Difference between MyLagrange and coefficient: [%d, %d]^T\n',simplify(EOM - M*thdd - C*thd - P));


%% State Space representation
syms tau [2 1] real

Sol = solve(tau == EOM, thdd);

%% Output thdd1, thdd2, and [M,C,P] as matlabFunction
% thdd1_fn = matlabFunction(Sol.thdd1, 'File', 'RR_thdd1');
% thdd2_fn = matlabFunction(Sol.thdd2, 'File', 'RR_thdd2');
% 
% matlabFunction(M,C,P,'File','RR_ManEqnCoeff');
% matlabFunction(q_sp1,q_sp2,'File','RR_EndEffector');
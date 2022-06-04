clear; close all; clc;
% lqrscript
[x_trim,u_trim,y_trim,dx] = trim('UAV', [0;0;0;0;0;0;0;0;pi/6;0;0;0], [0;0;0;0]);
[Ag,Bg,C_full,D_full] = linmod('UAV', x_trim, u_trim);
% We need to specify A and B matrices first
% A is of 12 by 12 and B is of 12 by 4
Ag(abs(Ag) < 1e-5) = 0;
Bg(abs(Bg) < 1e-5) = 0;
Cg = C_full([1, 2, 3, 9], :);
Dg = zeros(4, 4);
Gss = ss(Ag, Bg, Cg, Dg);

% We need to specify Q and R matrice next.
r = 10;
Qxu = diag([r,r,r,0,0,0,0,0,r,0,0,0,1,1,1,1]);
Qwv = eye(16);
% R = r*diag([1,1,1,1]);
% delta_A = 0.1*eye(12);
% delta_B = zeros(12, 4);
% delta_B(12, :) = 10*[1,1,1,1];

% lqr
K = lqg(Gss,Qxu,Qwv);
K = -K;
% save
save('lqggains.mat','Ag','Bg','K')

disp(K);
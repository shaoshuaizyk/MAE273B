clear; close all; clc;
% lqrscript
[x_trim,u_trim,y_trim,dx] = trim('UAV', [0;0;0;0;0;0;0;0;pi/6;0;0;0], [0;0;0;0]);
[Ag,Bg,C_full,D_full] = linmod('UAV', x_trim, u_trim);

Ag(abs(Ag) < 1e-5) = 0;
Bg(abs(Bg) < 1e-5) = 0;
Cg = C_full([1, 2, 3, 9], :);
Dg = zeros(4, 4);
Gss = ss(Ag,Bg,Cg,Dg);
Gtf = tf(Gss);
% Gss = minreal(Gtf);

% We need to specify Q and R matrice next.  
r = 10;
Q = diag([1,1,1,0,0,0,0,0,1,0,0,0]);
R = r*diag([1,1,1,1]);
delta_A = 0.1*eye(12);
delta_B = zeros(12, 4);
delta_B(12, :) = 10*[1,1,1,1];

% lqr 
[K,S,e] = lqr(Gss,Q,R);
Ag = K;
K = ss(eye(4));
% save
save('lqrgains.mat','Ag','Bg','K')

disp(Ag);
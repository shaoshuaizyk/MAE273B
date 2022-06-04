clear all;
clc; 
close all;

[x_trim,u_trim,y_trim,dx] = trim('UAV', [0;0;0;0;0;0;0;0;pi/6;0;0;0], [0;0;0;0]);
[A,B,Cfull,Dfull] = linmod('UAV', x_trim, u_trim);

C = Cfull([1, 2, 3, 9],:);
D = zeros([4 4]);

A(abs(A) < 1e-3) = 0;
B(abs(B) < 1e-3) = 0;
C(abs(C) < 1e-3) = 0;
D(abs(D) < 1e-3) = 0;

sys = ss(A, B, C, D);

Gptf = tf(sys);
Gp = minreal(Gptf)

syms s;
P = [-0.6408, 0.37, 0.6408, -0.37; -0.37, -0.6408, 0.37, 0.6408; 0.002*((s+0.1)^2), 0.002*((s+0.1)^2), 0.002*((s+0.1)^2), 0.002*((s+0.1)^2); -0.25*((s+0.1)^2), 0.25*((s+0.1)^2), -0.25*((s+0.1)^2), 0.25*((s+0.1)^2)]; 

[Ul,Ur,S] = smithForm(P)

clear; close all; clc;
%% Linearize the model, fidn Ag, Bg, Cg, Dg
[x_trim,u_trim,y_trim,dx] = trim('UAV', [0;0;0;0;0;0;0;0;pi/6;0;0;0], [0;0;0;0]);
[Ag,Bg,C_full,D_full] = linmod('UAV', x_trim, u_trim);

Ag(abs(Ag) < 1e-5) = 0;
Bg(abs(Bg) < 1e-5) = 0;

%% Choice of controller
% Number of controller outputs, 1 means only control the yaw, 2 means
% control the x and yaw, 4 means control the x, y, z and yaw 
NUMOUT = 4; %1 2, 4
% Weights: 'static' means identity weights, 'tune' means diagonal transfer
% functions
WEIGHTS = 'tune'; % static tune

% Adding poles
ADDINGPOLES = 0; % 0, 1

w=logspace(-2,3);

if NUMOUT == 1 % Control only the yaw
    Cg = C_full(9, :);
    Dg = zeros(1, 4);
elseif NUMOUT == 4 % Control x, y, z and yaw
    Cg = C_full([1, 2, 3, 9], :);
    Dg = zeros(4, 4);
elseif NUMOUT == 2
    Cg = C_full([1, 9], :);
    Dg = zeros(2, 4);
else
    error("ERROR! Wrong number of outputs!");
end

% Find the minimum realization
% [Ag,Bg,Cg,Dg] = minreal(Ag,Bg,Cg,Dg);
if ADDINGPOLES
    Ag = Ag - 0.1*eye(size(Ag));
end
Gss = ss(Ag,Bg,Cg,Dg);
Gtf = tf(Gss);

[GS, GNS] = stabsep(Gss);
assert(rank(ctrb(GNS.A, GNS.B)) >= size(GNS.A, 1), "The system is instabilizable!");
disp(Gtf)

%% Define Wp
if NUMOUT == 1 % Control only the yaw
    if strcmp(WEIGHTS,'static')
        Wptf = tf(1);
    else
        Wp1 = makeweight(100, 2, 0.25);
        Wptf = tf(Wp1);
    end
elseif NUMOUT == 4 % Control x, y, z and yaw
    if strcmp(WEIGHTS,'static')
        Wptf = tf(eye(4));
    else
        Wp1 = makeweight(100, 2, 0.25);
        Wp2 = makeweight(100, 2, 0.25);
        Wp3 = makeweight(100, 2, 0.25);
        Wp4 = makeweight(100, 2, 0.25);
        Wptf = tf([Wp1 0 0 0; 0 Wp2 0 0; 0 0 Wp3 0; 0 0 0 Wp4]);
    end
elseif NUMOUT == 2
    if strcmp(WEIGHTS,'static')
        Wptf = tf(eye(2));
    else
        Wp1 = makeweight(100, 2, 0.25);
        Wp2 = makeweight(100, 2, 0.25);
        Wptf = tf([Wp1 0; 0 Wp2]);
    end
else
    error("ERROR! Wrong number of outputs!");
end

Wpss = ss(Wptf);
[Ap,Bp,Cp,Dp] = ssdata(Wpss);
[Ap,Bp,Cp,Dp] = minreal(Ap,Bp,Cp,Dp);

%% Define Wu
Wu = [];

Wutf = tf(Wu);
Wuss = ss(Wutf);
[Au,Bu,Cu,Du] = ssdata(Wuss);
[Au,Bu,Cu,Du] = minreal(Au,Bu,Cu,Du);

%% Define Wd
if NUMOUT == 1 % Control only the yaw
    if strcmp(WEIGHTS,'static')
        Wdtf = tf(1);
    else
        Wd1 = makeweight(0.25, 2, 100);
        Wdtf = tf(Wd1);
    end
elseif NUMOUT == 4 % Control x, y, z and yaw
    if strcmp(WEIGHTS,'static')
        Wdtf = tf(eye(4));
    else
        Wd1 = makeweight(0.25, 2, 100);
        Wd2 = makeweight(0.25, 2, 100);
        Wd3 = makeweight(0.25, 2, 100);
        Wd4 = makeweight(0.25, 2, 100);
        Wdtf = tf([Wd1 0 0 0; 0 Wd2 0 0; 0 0 Wd3 0; 0 0 0 Wd4]);
    end
elseif NUMOUT == 2
    if strcmp(WEIGHTS,'static')
        Wdtf = tf(eye(2));
    else
        Wd1 = makeweight(100, 2, 0.25);
        Wd2 = makeweight(100, 2, 0.25);
        Wdtf = tf([Wd1 0; 0 Wd2]);
    end
else
    error("ERROR! Wrong number of outputs!");
end

Wdss = ss(Wdtf);
[Ad,Bd,Cd,Dd] = ssdata(Wdss);
[Ad,Bd,Cd,Dd] = minreal(Ad,Bd,Cd,Dd);

%% Compute the augmented plant
[A,B1,B2,C1,C2,D11,D12,D21,D22]=augss(Ag,Bg,Cg,Dg, Ap,Bp,Cp,Dp,...
                                      Au,Bu,Cu,Du, Ad,Bd,Cd,Dd);
Plant = augw(Gtf,Wptf,Wutf,Wdtf);
[K, CL, Gamma] = hinfsyn(Plant, NUMOUT, 4);
if isempty(K)
    error("Could not find the solution!");
end
% K.A(K.A < 1e-10) = 0;
% K.B(K.B < 1e-10) = 0;
% K.C(K.C < 1e-10) = 0;
% K.D(K.D < 1e-10) = 0;
[acp,bcp,ccp,dcp]=ssdata(K);
[acl,bcl,ccl,dcl]=ssdata(CL);

%controller
[acp,bcp,ccp,dcp]=minreal(acp,bcp,ccp,dcp);
% open loop
[aly,bly,cly,dly]=series(acp,bcp,ccp,dcp,Ag,Bg,Cg,Dg);
[aly,bly,cly,dly]=minreal(aly,bly,cly,dly);
[alu,blu,clu,dlu]=series(Ag,Bg,Cg,Dg,acp,bcp,ccp,dcp);
[alu,blu,clu,dlu]=minreal(alu,blu,clu,dlu);
%closed loop
[at,bt,ct,dt]=cloop(aly,bly,cly,dly,-1);
[at,bt,ct,dt]=minreal(at,bt,ct,dt);
as=at;
bs=bt;
cs=-ct;
ds=eye(size(dt))-dt;
[atu,btu,ctu,dtu]=cloop(alu,blu,clu,dlu,-1);
[atu,btu,ctu,dtu]=minreal(atu,btu,ctu,dtu);
asu=atu;
bsu=btu;
csu=-ctu;
dsu=eye(size(dtu))-dtu;
[ay,by,cy,dy]=series(as,bs,cs,ds,acp,bcp,ccp,dcp);
[ay,by,cy,dy]=minreal(ay,by,cy,dy);

%% figure 1
figure(1);
sigma(aly,bly,cly,dly)
hold on
sigma(Gamma.*inv(Wdtf(NUMOUT, NUMOUT)),w)
hold on     
sigma(Gamma.*Wptf(1,1),w)
legend('Ly sigular values','1/Wd','Wp')

%% figure 2
figure(2)
sigma(at,bt,ct,dt,w)
hold on
sigma(as,bs,cs,ds,w)
% hold on
% sigma(asu,bsu,csu,dsu,w)
hold on
sigma(Gamma.*inv(Wdtf(NUMOUT,NUMOUT)),w)
hold on
sigma(Gamma.*inv(Wptf(1,1)),w)
legend('T','S','1/Wd','1/Wp')
hold off

%% figure Youla singular values
figure(3)
sigma(ay,by,cy,dy,w)
legend('Youla singular values')


%% Closed loop time response
t=0:0.01:10;
figure(4)
step(ss(at,bt,ct,dt),t)
title('step responses to the input')

%% Plant singular values
figure(5)
sigma(Gtf)
legend('Plant singular values')

%% Controller singular values
figure(6)
sigma(K)
legend('Controller singular values')

%% By students, compute the maximum sigular value of Tzw from lft computation
figure (7)
Tzw=lft(Plant,K);
Kinf=norm(Tzw,inf);
Kinflog=20*log10(Kinf);
Kinflog=Kinflog*ones(size(w));
sigma(Tzw)
hold on
semilogx(w,Kinflog)
hold on
sigma(acl,bcl,ccl,dcl)
legend('Inf norm of Tzw')


%% Save the A, B, K
% save('hinfgains.mat','Ag','Bg','K')
% disp(K);
%% Definition od quadcopter model's parameters
clear; close all; clc;

g = 9.81;

Ub = 12;             % napon baterije
Omega_b = 0;         % pomjeraj (bias) linearizovane karakteristike motora
Kv = 980;         % nagib linearizovane karakteristike motora
cT = 3.13*1e-4;      % konstanta sile potiska
cM = 7.5*1e-6;    % konstanta momenta reakcije
Tm = 0.02;            % vremenska konstanta modela motora
d = 0.225;               % udaljenost ose rotacije motora od centra mase
m = 0.6;

JR = 1e-5;
Jxx = 0.0075;
Jyy = 0.0075;
Jzz = 0.0075;
J = diag([Jxx, Jyy, Jzz]);

S = [cT cT cT cT;
     d*cT*sqrt(2)/2 -d*cT*sqrt(2)/2 -d*cT*sqrt(2)/2 d*cT*sqrt(2)/2;
     -d*cT*sqrt(2)/2 -d*cT*sqrt(2)/2 d*cT*sqrt(2)/2 d*cT*sqrt(2)/2;
     -cM cM -cM cM];
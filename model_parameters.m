%% Definition of quadcopter model's parameters
clear; close all; clc;

g = 9.81;

Ub = 12;             % baterry voltage
Omega_b = 0;         % motor speed bias
Kv = 980;            % motor speed coefficient
cT = 3.13*1e-4;      % thrust coefficient
cM = 7.5*1e-6;       % torque coefficient
Tm = 0.02;           % motor time constant
d = 0.225;           % distance netween frame COM and rotor axis
m = 0.6;             % mass

JR = 1e-5;           % propeller moment of inertia   
Jxx = 0.0075;
Jyy = 0.0075;
Jzz = 0.0075;
J = diag([Jxx, Jyy, Jzz]);  % frame inertia matrix

S = [     cT               cT              cT               cT
     d*cT*sqrt(2)/2  -d*cT*sqrt(2)/2 -d*cT*sqrt(2)/2  d*cT*sqrt(2)/2
     -d*cT*sqrt(2)/2 -d*cT*sqrt(2)/2  d*cT*sqrt(2)/2  d*cT*sqrt(2)/2
         -cM               cM             -cM               cM];
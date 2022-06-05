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
 
%%
syms x y z xd yd zd phi theta psi wx wy wz
syms Ft Mx My Mz

p = [x; y; z];
v = [xd; yd; zd];
eta = [phi; theta; psi];
omega = [wx; wy; wz];

X = [p; v; eta; omega];
U = [Ft; Mx; My; Mz];

Rx = [1     0         0
      0  cos(phi)  -sin(phi)
      0  sin(phi)   cos(phi)];
  
Ry = [cos(theta)  0   sin(theta)
          0       1       0
      -sin(theta) 0   cos(theta)];

Rz = [cos(psi)  -sin(psi)  0
      sin(psi)   cos(psi)  0
         0          0      1];
  
R = Rz*Ry*Rx;

T = [1  tan(theta)*sin(phi)  tan(theta)*cos(phi)
     0       cos(phi)             -sin(phi)
     0  sin(phi)/cos(theta)  cos(phi)/cos(theta)];

f(1:3) = v;
f(4:6) = [0; 0; -g] + 1/m*R*[0; 0; Ft];
f(7:9) = T*omega;
f(10:12) = J^(-1)*(-cross(omega, J*omega) + [Mx; My; Mz]);

f = simplify(transpose(f));

A = jacobian(f, X);
B = jacobian(f, U);

matlabFunction(f, 'File', 'state_fcn', 'Vars', {X, U});
matlabFunction(A, B, 'File', 'state_jacob_fcn', 'Vars', {X, U});
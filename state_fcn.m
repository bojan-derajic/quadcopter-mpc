function f = state_fcn(in1,in2)
%STATE_FCN
%    F = STATE_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    05-Jun-2022 22:38:27

Ft = in2(1,:);
Mx = in2(2,:);
My = in2(3,:);
Mz = in2(4,:);
phi = in1(7,:);
psi = in1(9,:);
theta = in1(8,:);
wx = in1(10,:);
wy = in1(11,:);
wz = in1(12,:);
xd = in1(4,:);
yd = in1(5,:);
zd = in1(6,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = tan(theta);
f = [xd;yd;zd;Ft.*(t5.*t6.*(5.0./3.0)+t2.*t3.*t7.*(5.0./3.0));Ft.*t3.*t5.*(-5.0./3.0)+Ft.*t2.*t6.*t7.*(5.0./3.0);Ft.*t2.*t4.*(5.0./3.0)-9.81e+2./1.0e+2;wx+t5.*t8.*wy+t2.*t8.*wz;t2.*wy-t5.*wz;(t5.*wy+t2.*wz)./t4;Mx.*(4.0e+2./3.0);My.*(4.0e+2./3.0);Mz.*(4.0e+2./3.0)];

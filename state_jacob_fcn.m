function [A,B] = state_jacob_fcn(in1,in2)
%STATE_JACOB_FCN
%    [A,B] = STATE_JACOB_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    05-Jun-2022 22:38:27

Ft = in2(1,:);
phi = in1(7,:);
psi = in1(9,:);
theta = in1(8,:);
wy = in1(11,:);
wz = in1(12,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = tan(theta);
t9 = t8.^2;
t10 = t2.*wz;
t11 = t5.*wy;
t12 = 1.0./t4;
t14 = t3.*t5.*(5.0./3.0);
t15 = t2.*t6.*t7.*(5.0./3.0);
t13 = t9+1.0;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ft.*(t2.*t6.*(5.0./3.0)-t3.*t5.*t7.*(5.0./3.0)),Ft.*t2.*t3.*(-5.0./3.0)-Ft.*t5.*t6.*t7.*(5.0./3.0),Ft.*t4.*t5.*(-5.0./3.0),t2.*t8.*wy-t5.*t8.*wz,-t10-t11,t12.*(t2.*wy-t5.*wz),0.0,0.0,0.0,0.0,0.0,0.0,Ft.*t2.*t3.*t4.*(5.0./3.0),Ft.*t2.*t4.*t6.*(5.0./3.0),Ft.*t2.*t7.*(-5.0./3.0),t10.*t13+t11.*t13,0.0,t7.*t12.^2.*(t10+t11),0.0,0.0,0.0,0.0,0.0,0.0,Ft.*(t14-t15),Ft.*t5.*t6.*(5.0./3.0)+Ft.*t2.*t3.*t7.*(5.0./3.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t5.*t8,t2,t5.*t12,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2.*t8,-t5,t2.*t12,0.0,0.0,0.0],[12,12]);
if nargout > 1
    B = reshape([0.0,0.0,0.0,t5.*t6.*(5.0./3.0)+t2.*t3.*t7.*(5.0./3.0),-t14+t15,t2.*t4.*(5.0./3.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,4.0e+2./3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,4.0e+2./3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,4.0e+2./3.0],[12,4]);
end
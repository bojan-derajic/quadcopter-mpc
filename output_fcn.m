function h = output_fcn(in1,in2)
%OUTPUT_FCN
%    H = OUTPUT_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Jun-2022 10:47:18

psi = in1(9,:);
x = in1(1,:);
y = in1(2,:);
z = in1(3,:);
h = [x;y;z;psi];

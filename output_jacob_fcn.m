function [C,D] = output_jacob_fcn(in1,in2)
%OUTPUT_JACOB_FCN
%    [C,D] = OUTPUT_JACOB_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Jun-2022 08:12:05

C = reshape([1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[4,12]);
if nargout > 1
    D = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[4,4]);
end

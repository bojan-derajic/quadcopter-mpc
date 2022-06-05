%% Linear MPC Design
model_parameters

nx = 12;
ny = 4;
nu = 4;

MPC_nonlin = nlmpc(nx, ny, nu);

Ts = 0.1;
Np = 20;
Nc = 5;

MPC_nonlin.Ts = Ts;
MPC_nonlin.PredictionHorizon = Np;
MPC_nonlin.ControlHorizon = Nc;

MPC_nonlin.Model.StateFcn = @state_fcn;
MPC_nonlin.Model.OutputFcn = @(x, u) [x(1); x(2); x(3); x(9)];

E = [S^(-1); -S^(-1)];
G = [(Ub*Kv)^(2)*ones(4, 1); zeros(4, 1)];

MPC_nonlin.Optimization.CustomIneqConFcn = @(X, U, e, data) ...
    [reshape(E*U(1:Np, :)', [], 1) - repmat(G, Np, 1)];

%MPC_lin.OV(3).Min = 0;

% MPC_lin.W.MV(1) = 1;
% MPC_lin.W.MV(2) = 100;
% MPC_lin.W.MV(3) = 100;
% MPC_lin.W.MV(4) = 100;
% 
% MPC_lin.W.OV(1) = 1;
% MPC_lin.W.OV(2) = 1;
% MPC_lin.W.OV(3) = 1000;
% MPC_lin.W.OV(4) = 1;

validateFcns(MPC_nonlin, zeros(nx, 1), [m*g; 0; 0; 0]);
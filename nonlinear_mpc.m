%% Nonlinear MPC Design for Quadcopter
create_nlmpc_fcns
model_parameters

nx = 12;
ny = 4;
nu = 4;

MPC_nonlin = nlmpc(nx, ny, nu);

Ts = 0.1;
Np = 25;
Nc = 2;

MPC_nonlin.Ts = Ts;
MPC_nonlin.PredictionHorizon = Np;
MPC_nonlin.ControlHorizon = Nc;

MPC_nonlin.Model.StateFcn = @state_fcn;
MPC_nonlin.Jacobian.StateFcn = @state_jacob_fcn;
MPC_nonlin.Model.OutputFcn = @output_fcn;
MPC_nonlin.Jacobian.OutputFcn = @output_jacob_fcn;

E = [S^(-1); -S^(-1)];
G = [(Ub*Kv)^(2)*ones(4, 1); zeros(4, 1)];

MPC_nonlin.Optimization.CustomIneqConFcn = @(X, U, e, data) ...
    [reshape(E*U(1:Np, :)', [], 1) - repmat(G, Np, 1)];

MPC_nonlin.OutputVariables(3).Min = 0;

MPC_nonlin.W.ManipulatedVariables(1) = 0.01;
MPC_nonlin.W.ManipulatedVariables(2) = 0.1;
MPC_nonlin.W.ManipulatedVariables(3) = 0.1;
MPC_nonlin.W.ManipulatedVariables(4) = 1;

MPC_nonlin.W.ManipulatedVariablesRate(1) = 0;
MPC_nonlin.W.ManipulatedVariablesRate(2) = 0;
MPC_nonlin.W.ManipulatedVariablesRate(3) = 0;
MPC_nonlin.W.ManipulatedVariablesRate(4) = 0;

MPC_nonlin.W.OutputVariables(1) = 1;
MPC_nonlin.W.OutputVariables(2) = 1;
MPC_nonlin.W.OutputVariables(3) = 1.5;
MPC_nonlin.W.OutputVariables(4) = 1;

validateFcns(MPC_nonlin, zeros(nx, 1), [m*g; 0; 0; 0]);
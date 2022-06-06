%% Linear MPC Design
model_parameters

% Nominal values of states and inputs at operating point
x0 = [0 0 0 0 0 0 0 0 0 0 0 0]';
u0 = [m*g 0 0 0]';

% Continious state-space model definition of the linearized model
A = [0  0  0  1  0  0  0  0  0  0  0  0
     0  0  0  0  1  0  0  0  0  0  0  0
     0  0  0  0  0  1  0  0  0  0  0  0
     0  0  0  0  0  0  0  g  0  0  0  0
     0  0  0  0  0  0 -g  0  0  0  0  0
     0  0  0  0  0  0  0  0  0  0  0  0
     0  0  0  0  0  0  0  0  0  1  0  0
     0  0  0  0  0  0  0  0  0  0  1  0
     0  0  0  0  0  0  0  0  0  0  0  1
     0  0  0  0  0  0  0  0  0  0  0  0
     0  0  0  0  0  0  0  0  0  0  0  0
     0  0  0  0  0  0  0  0  0  0  0  0];
 
B = [   0     0     0     0
        0     0     0     0
        0     0     0     0
        0     0     0     0
        0     0     0     0
       1/m    0     0     0
        0     0     0     0
        0     0     0     0
        0     0     0     0
        0   1/Jxx   0     0
        0     0   1/Jyy   0
        0     0     0   1/Jzz];

C = [1 0 0 0 0 0 0 0 0 0 0 0
     0 1 0 0 0 0 0 0 0 0 0 0
     0 0 1 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 1 0 0 0];
 
D = zeros(4);
     
quadcopter_lin_c = ss(A, B, C, D);

quadcopter_lin_c.InputName = {'Ft'; 'Mx'; 'My'; 'Mz'};
quadcopter_lin_c.InputUnit = {'N'; 'Nm'; 'Nm'; 'Nm'};

quadcopter_lin_c.OutputName = {'x'; 'y'; 'z'; 'yaw'};
quadcopter_lin_c.OutputUnit = {'m'; 'm'; 'm'; 'rad'};

n = size(A, 1);
k = size(B, 2);
p = size(C, 1);

% Diskretization
Ts = 0.1;
quadcopter_lin = c2d(quadcopter_lin_c, Ts);

% Controllabiliti and Observability Check
Qc_rank = rank(ctrb(quadcopter_lin));
Qo_rank = rank(obsv(quadcopter_lin));

if Qc_rank == n
    fprintf('Model is controllable: rank(Qc) = %d\n', Qc_rank);
else
    fprintf('Model is not controllable: rank(Qc) = %d\n', Qc_rank);
end

if Qo_rank == n
    fprintf('Model is observable: rank(Qo) = %d\n', Qc_rank);
else
    fprintf('Model is not observable: rank(Qo) = %d\n', Qc_rank);
end

% MPC Design
mpcverbosity('off');
MPC_lin = mpc(quadcopter_lin, Ts);

% MPC Controller Parameters
MPC_lin.Model.Nominal.X = x0;
MPC_lin.Model.Nominal.U = u0;

MPC_lin.PredictionHorizon = 60;
MPC_lin.ControlHorizon = 5;

E = [S^(-1); -S^(-1)];
F = [];
G = [(Ub*Kv)^(2)*ones(4, 1); zeros(4, 1)];
V = zeros(8, 1);

setconstraint(MPC_lin, E, F, G, V);

MPC_lin.OV(3).Min = 0;

MPC_lin.W.MV(1) = 1;
MPC_lin.W.MV(2) = 100;
MPC_lin.W.MV(3) = 100;
MPC_lin.W.MV(4) = 100;

MPC_lin.W.OV(1) = 1;
MPC_lin.W.OV(2) = 1;
MPC_lin.W.OV(3) = 1000;
MPC_lin.W.OV(4) = 1;
 
get(MPC_lin);

% Linear MPC test
T = 56;
r = [1, -2, 4, -0.2];
sim(MPC_lin, T, r)
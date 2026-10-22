% Motor parameters (CTMS DC motor speed example)
J = 0.01;     % rotor inertia [kg·m²]
b = 0.1;      % viscous friction [N·m·s]
K = 0.01;     % motor constant (Kt = Ke)
R = 1;        % armature resistance [Ω]
L = 0.5;      % armature inductance [H]

% Test conditions
speed_ref   = 1;     % speed reference [rad/s]
t_load      = 3;     % time of load torque step [s]
T_load_step = 0.01;  % load torque step size [N·m]
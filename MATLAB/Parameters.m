%% Parameter loading file
% loads all parameters

%% known parameters
params.kappa = 1.35; 
params.R = 287;
params.V_c = 2.48e-4;
params.V_d = 2.48e-3;
params.lambda = 1;    % assumed to be constant (according to exercise manual)
params.sigma_0 = 14.7; % Stoichiometric air/fuel ratio gasoline
params.H_l = 42.5e6;
params.k_zeta = 2.3345e-4;
params.eta_gen = 0.7;

%% identified parameters
% Throttle Area
params.alpha0 = 1.4177e-6;
params.alpha1 = 5.8877e-6;
% Intake Manifold
params.Vm = 3.966146e-3;
% Engine Air Mass FLow
params.gamma_0 = 0.6279;
params.gamma_1 = 9.8537e-4;
% % Engine Torque Generation
% params.eta_0 = ;
% params.eta_1 = ;
% params.beta_0 = ;
% % Engine Inertia
% params.theta_e = ;


%% initial guess parameters
% Throttle Area
params.alpha0_guess = 3e-6;
params.alpha1_guess = 6e-6;
% Intake Manifold
params.Vm_guess = 7e-3;
% Engine Air Mass FLow
params.gamma_0_guess = 0.6;
params.gamma_1_guess = 0.002;
% Engine Torque Generation
params.eta_0_guess = 0.3;
params.eta_1_guess = -3e-4;
params.beta_0_guess = 7;
% Engine Inertia
params.theta_e_guess = 0.2;


%% initial conditions of integrators (obtained from steady state simulation)

% intake manifold
params.p_m0 = 3.7e4;

% engine inertia
params.omega_e0 = 75;

% subsystem xy
% ...

%% sim options

% sim options
simopt = simset('Solver','ode1','FixedStep',1e-3,'SrcWorkspace','current');

    
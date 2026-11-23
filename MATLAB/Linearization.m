function [nom, lin] = Linearization(params, options)
% Description: Normalization is done in the simulink model and here the Linearization is done 

%% Get nominal values for normalization

% Run steady-state simulation with reasonable steady state input values
u_alpha = 2.5;      % Throttle command [%]              Steady state ~ 1.5 - 3 %
du_ign_nom = -20;   % Ignotion retardation [deg CA]     Steady state ~ -40 - -10 °CA
tEnd = 60;          % Steady state simulation duration
out = sim('NonlinearModel', tEnd, options.sim_options);

% Extract simulation results
t = out.omega_e.Time;
omega_e = out.omega_e.Data; %                           Steady state ~ 70 - 200 rad/s
p_m = out.p_m.Data;         %                           Steady state ~ 0.2 - 0.4 bar

% Store preliminary nominal values 
nom.u_alpha = u_alpha;      % defined as scalar for input
nom.du_ign = du_ign_nom;    % defined as scalar for input
nom.omega_e = omega_e(end); % only take last steady state value (scalar)
nom.p_m = p_m(end);         % only take last steady state value (scalar)

% Nominal delay times used in pade approximation
nom.tau_IPS = 2*pi / nom.omega_e;
nom.tau_ign = 2*pi / (5*nom.omega_e);

% Nominal inputs of the Padé delay elements used for normalization
nom.q_IPS = out.mdot_beta.Data(end) / nom.omega_e;
nom.q_ign = nom.du_ign;

%% 4th order Pade approximation

% IPS delay approximation
[numIPS, denIPS] = pade(nom.tau_IPS, 4);
sysIPS = ss(tf(numIPS, denIPS));
[Aips, Bips, Cips, Dips] = ssdata(sysIPS);

% Ignition delay approximation
[numIGN, denIGN] = pade(nom.tau_ign, 4);
sysIGN = ss(tf(numIGN, denIGN));
[Aign, Bign, Cign, Dign] = ssdata(sysIGN);

% Initial conditions of normalized Padé elements
x0ips = -(Aips \ Bips);
x0ign = -(Aign \ Bign);

% Store Padé variables
nom.Aips = Aips;
nom.Bips = Bips;
nom.Cips = Cips;
nom.Dips = Dips;
nom.x0ips = x0ips;

nom.Aign = Aign;
nom.Bign = Bign;
nom.Cign = Cign;
nom.Dign = Dign;
nom.x0ign = x0ign;

u_norm = [1; 1];
x_norm = [1; 1; nom.x0ips; nom.x0ign];

%% Identify Linearized Model Matrices ABCD

% This gives nom over to the Main Workspace, so Simulink can see it
assignin('base', 'nom', nom); 

% get matrices
[lin.As, lin.Bs, lin.Cs, lin.Ds] = linmod('NonlinearModelNorm', x_norm, u_norm);

end


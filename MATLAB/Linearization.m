function nom = Linearization(params, options)

%% 2.4.2 - Normalization and Linearization

% Starting guesses
u_alpha = 2.5;      % Throttle command [%]              They recommend 1.5 - 3 %
du_ign_nom = -20;   % Ignotion retardation [deg CA]     They recommend -40 - -10 °CA

tEnd = 60; % Simulation duration

%% Run steady-state simulation

out = sim('NonlinearModel', tEnd, options.sim_options);

% Extract simulation results
t = out.omega_e.Time;
omega_e = out.omega_e.Data; %                           They recommend 70 - 200 rad/s

p_m = out.p_m.Data; %                                   They recommend 0.2 - 0.4 bar

% Store preliminary nominal values
nom.u_alpha = u_alpha;
nom.du_ign = du_ign_nom;
nom.omega_e = omega_e(end);
nom.p_m = p_m(end);

% Nominal delay times
nom.tau_IPS = 2*pi / nom.omega_e;

nom.tau_ign = 2*pi / (5*nom.omega_e);

% Nominal inputs of the Padé delay elements
nom.q_IPS = out.mdot_beta.Data(end) / nom.omega_e;

%% 4th order Pade approximation

% IPS delay
[numIPS, denIPS] = pade(nom.tau_IPS, 4);

sysIPS = ss(tf(numIPS, denIPS));

[Aips, Bips, Cips, Dips] = ssdata(sysIPS);

% Ignition delay
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

end
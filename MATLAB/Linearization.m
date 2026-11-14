function nom = Linearization(params, options)

%% 2.4.2 - Normalization and Linearization

% Starting guesses
u_alpha = 2.5;      % Throttle command [%] 
du_ign_nom = -20;   % Ignotion retardation [deg CA] 

tEnd = 60; % Simulation duration

%% Run steady-state simulation

out = sim('NonlinearModel', tEnd, options.sim_options);

% Extract simulation results
t = out.omega_e.Time;
omega_e = out.omega_e.Data;

p_m = out.p_m.Data;

end
function PlotModelValidation(Model, FileName, projectFolder, params, options, lin)

% Takes a File from the Data folder and plots the input Model vs the data.

% Distinguish between linear and nonlinear inputs
if nargin >= 6
    As = lin.As;
    Bs = lin.Bs;
    Cs = lin.Cs;
    Ds = lin.Ds;
    nom = lin.nom;
end

% Load independent validation dataset
val = load(fullfile(projectFolder, 'Data', FileName));

% Rename it, as Simulink expects "data.meas..."
data = val;

% Initialize model states from val measurements (For the start)
params.p_m0 = data.meas.p_m.signals.values(1);
params.omega_e0 = data.meas.omega_e.signals.values(1);

% Run the complete model with id parameters
out = sim(Model, ...
    data.meas.u_alpha.time(end), options.sim_options);

% Compare measured and simulated engine speed

% Measured engine speed (From val data)
t_meas = data.meas.omega_e.time;
omega_meas = data.meas.omega_e.signals.values;

% Simulated engine speed
t_sim = out.omega_e.Time;
omega_sim = out.omega_e.Data;

% Plot 
rpmFactor = 60/(2*pi);

figure;

plot(t_meas, omega_meas*rpmFactor, 'b');
hold on;
plot(t_sim, omega_sim*rpmFactor, 'r');
hold off;

xlabel('Time [s]');
ylabel('Engine speed [rpm]');
title("Validation of """ + string(Model) + ...
    """ on """ + string(FileName) + """", ...
    'Interpreter', 'none');
legend('Measured', 'Simulated');
grid on;

end
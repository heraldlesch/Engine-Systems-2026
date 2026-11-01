clear; clc; close all;

%% Setup Paths (Give Main.m access to the Data and Simulink folders)

% Folder containing Main.m
matlabFolder = fileparts(mfilename('fullpath'));

% Project root folder
projectFolder = fileparts(matlabFolder);

% Add MATLAB scripts and Simulink models to search path
addpath(matlabFolder);
addpath(fullfile(projectFolder, 'Simulink'));

%% Load Data

runIdentification = false;

% Load Parameters
Parameters
fprintf('\n Parameters loaded \n')

% Simulation Options
options.sim_options = simopt;
options.enablePlot = false;
options.fig_num = 1;

if runIdentification

    % Load Measurement Data
    quasi = load(fullfile(projectFolder, 'Data', 'quasistatic_0001.mat'));
    id = load(fullfile(projectFolder, 'Data', 'dynamic_0002.mat'));

    % Parameter identification
    params = linparid(params, quasi.meas); % 2.3.3 & 2.3.4
    params = nonlinparid_Intake_Manifold(params, id, options); % 2.3.5
    params = nonlinparid_Engine_Torque_and_Inertia(params, id, options); % 2.3.7

    % Save Identified parameters (overwrites old file)
    save(fullfile(matlabFolder, 'identified_params.mat'), 'params');

    fprintf('\nAll parameters identified and saved.\n');


else

    % Load previously identified parameters
    load(fullfile(matlabFolder, 'identified_params.mat'), 'params');

    fprintf('\nPreviously identified parameters loaded.\n');

end

% Display parameters
disp(params);

%% Validation of the identified parameters - 2.3.8

% Load independent validation dataset
val = load(fullfile(projectFolder, 'Data', 'dynamic_0003.mat'));

% Rename it, as Simulink expects "data.meas..."
data = val;

% Initialize model states from val measurements (For the start)
params.p_m0 = data.meas.p_m.signals.values(1);
params.omega_e0 = data.meas.omega_e.signals.values(1);

% Run the complete model with id parameters
out = sim('ParId_Engine_Torque_and_Inertia', ...
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
title('Model Validation - 2.3.8');
legend('Measured', 'Simulated');
grid on;
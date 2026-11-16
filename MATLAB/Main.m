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

PlotModelValidation('ParId_Engine_Torque_and_Inertia', ...
    'dynamic_0003.mat', ...
    projectFolder, params, options);


%% Linearization - 2.4.2 - 5.

% Reload identified parameters, so previous validation aren't carried over
saved = load(fullfile(matlabFolder, 'identified_params.mat'), 'params');

nom = Linearization(saved.params, options);

% Nominal inputs for the normalized model
u_alpha_norm = 1;
u_ign_norm = 1;

u_norm = [1; 
          1];

x_norm = [1;
          1; 
          nom.x0ips; 
          nom.x0ign];

[As, Bs, Cs, Ds] = linmod('NonlinearModelNorm', x_norm, u_norm);


%% Validation of the linearized model 

% Give the lin struct over to the function,
% so it can call LinearizedModel correctly
lin.As = As;
lin.Bs = Bs;
lin.Cs = Cs;
lin.Ds = Ds;
lin.nom = nom;

PlotModelValidation('LinearizedModel', ...
    'dynamic_0003.mat', ...
    projectFolder, params, options, lin)

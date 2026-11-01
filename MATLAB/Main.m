clear; clc; close all;

%% Setup Paths

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
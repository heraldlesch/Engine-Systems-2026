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

% Load Parameters
Parameters
fprintf('\n Parameters loaded \n')

% Load Measurement Data

% Quasistatic data for linear identification
quasi = load(fullfile(projectFolder, 'Data', ...
    'quasistatic_0001.mat'));

% Dynamic data for nonlinear identification
id = load(fullfile(projectFolder, 'Data', ...
    'dynamic_0002.mat'));

%% Simulation Options

options.sim_options = simopt;
options.enablePlot = false;
options.fig_num = 1;

%% Parameter identification

% Linear Parameter Identification - 2.3.3 & 2.3.4
params = linparid(params, quasi.meas);

% Nonlinear parameter identification - 2.3.5
params = nonlinparid_Intake_Manifold(...
    params, id, options);

% Nonlinear parameter identification - 2.3.7
params = nonlinparid_Engine_Torque_and_Inertia(...
    params, id, options);

% Save Identified parameters
save(fullfile(matlabFolder, 'identified_params.mat'), 'params');

fprint('\n All parameters identified \n');

disp(params);
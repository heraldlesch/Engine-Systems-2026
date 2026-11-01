clear; clc; close all;

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

% Simulation Options

options.sim_options = simopt;
options.enablePlot = false;
options.fig_num = 1;

%% Parameter identification

% Linear Parameter Identification - Manifold

params = linparid(params, quasi.meas);

fprint('\nManifold volume identified\n');

% Nonlinear parameter identification - Engine

params = nonlinparid_Engine_Torque;
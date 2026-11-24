clear; clc; close all;

%% USER INPUT SECTION

runIdentification = false; % false -> loads already identified parameters ¦ 
                           % true -> identifies params again and overwrites 
                           % old param matfile (takes a while ~15min(?))

%% Setup Paths 

% Folder containing this script (Main.m)
matlabFolder = fileparts(mfilename('fullpath'));

% Project root folder
projectFolder = fileparts(matlabFolder);

% Add remaining folders in project folder to path
addpath(matlabFolder);
addpath(fullfile(projectFolder, 'Simulink'));
addpath(fullfile(projectFolder, 'Plots'));
addpath(fullfile(projectFolder, 'Data'));


%% Load Data & Parameters (or identify parameters again if specfied in input)
fprintf('\n============== LOAD DATA ==============\n')
% Load Parameters
Parameters
fprintf('\nParameters loaded\n')

% Simulation Options
options.sim_options = simopt;
options.enablePlot = false;
options.fig_num = 1;

if runIdentification

    % Load Measurement Data
    quasi = load('quasistatic_0001.mat');
    id = load('dynamic_0002.mat');

    % Parameter identification
    params = linparid(params, quasi.meas); 
    params = nonlinparid_Intake_Manifold(params, id, options); 
    params = nonlinparid_Engine_Torque_and_Inertia(params, id, options); 

    % Save Identified parameters (overwrites old file)
    save(fullfile(matlabFolder, 'identified_params.mat'), 'params');

    fprintf('\nAll parameters identified and saved:\n');

else

    % Load previously identified parameters
    load('identified_params.mat');
    fprintf('\nPreviously identified parameters loaded:\n');

end

% Display parameters
disp(params);

%% Validation of the identified parameters
fprintf('\n============== Validating parameters... ==============\n')
% with one of these -> 'dynamic_0003.mat' 'studentId_0004.mat' 'studentId_0005.mat' 

PlotModelValidation('ParId_Engine_Torque_and_Inertia', 'dynamic_0003.mat', ...
    projectFolder, params, options);


%% Normalization & Linearization 

[nom, lin] = Linearization(params, options);
lin.nom = nom;

%% Validation of the linearized model 

% Compare linearized sim output with measurments
PlotModelValidation('LinearizedModel', 'dynamic_0003.mat', ...
    projectFolder, params, options, lin)

% Check linearized system behavior
LinearizedModelCheck

%% Controller Synthesis

% Tuning
tuning.KI = 0.5;
tuning.r1 = 6;
tuning.r2 = 10;
tuning.q = 0.1; % q -> 0: fast observer, trust measurement; q -> inf: slow observer, trust model 
tuning.Ts = 0.001; % sampling rate - must be 0.001

% Build controller
ctrl = ControllerSynthesis(lin, tuning);

% Extract controller matrices for Simulink
Ad = ctrl.Ad;
Bd = ctrl.Bd;
Cd = ctrl.Cd;
Dd = ctrl.Dd;

% Normalization matrices
ISCS_Tu = diag([nom.u_alpha, nom.du_ign]);
ISCS_Ty = nom.omega_e;


%% Test controller on nonlinear engine

fprintf('\n========== NONLINEAR CONTROLLER TEST ==========\n');

% Longer simulation because the current controller has slow poles
tEnd = 60;

% Run nonlinear closed-loop Simulink model
out = sim('ControllerTestModel', tEnd, options.sim_options);

% Extract logged engine speed and reference
t = out.omega_e.Time;
omega = out.omega_e.Data;

t_ref = out.omega_ref.Time;
omega_ref = out.omega_ref.Data;

% Plot closed-loop response
figure;

plot(t, omega, 'LineWidth', 1.5);
hold on;

plot(t_ref, omega_ref, '--', 'LineWidth', 1.5);

yline(50, ':r', 'Stall threshold');

xlabel('Time [s]');
ylabel('Engine speed [rad/s]');
title('Nonlinear Engine - Closed-Loop Speed Tracking');
legend('Actual speed', 'Reference speed', ...
    'Minimum allowed speed', 'Location', 'best');

grid on;


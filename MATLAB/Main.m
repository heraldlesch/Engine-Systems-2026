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
ControllerSynthesis

%% Test the controller with a trial sim
% tEnd = 10; % Simulation duration
% out = sim('ControllerTestModel', tEnd, options.sim_options);
% 
% % Extract simulation results
% t_test = out.omega_e.Time;
% omega_e_test = out.omega_e.Data; 
% t_test_in = out.omega_ref.Time;
% omega_ref_test = out.omega_ref.Data;
% 
% figure;
% plot(t_test, omega_e_test, t_test_in, omega_ref_test); 
% % ylim([0.05e4,3.6e4])
% % xlim([0,60])
% xlabel('Time [s]');
% ylabel('Engine speed [rad]');
% legend('Output','Reference')


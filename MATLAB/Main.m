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
params = saved.params;

% Linearize the Model
[nom, lin] = Linearization(params, options);

lin.nom = nom;

%% Validation of the linearized model 

PlotModelValidation('LinearizedModel', ...
    'dynamic_0003.mat', ...
    projectFolder, params, options, lin)
% Integral gain
KI = 0.5;  % Initialwert, beispielhaft

% Extension matrices
Ae = 0;
Be = [1 0];
Ce = [KI; 0];
De = eye(2);

% Extended plant
A = [lin.As, lin.Bs*Ce;
    zeros(1,size(lin.As,2)), Ae];

B = [lin.Bs*De;
    Be];

C = [lin.Cs, lin.Ds*Ce];
D = lin.Ds*De;

r1 = 6;
r2 = 10;

Q = C' * C;
R = diag([r1 r2]);

K = lqr(A,B,Q,R);

% Observer tuning parameter
q = 0.1;

% Observer gain
L = lqr(A', C', B*B', q)';

% Closed-loop poles
poles_controller = eig(A - B*K);

% Observer poles
poles_observer = eig(A - L*C);

disp('Closed-loop poles:')
disp(poles_controller)

disp('Observer poles:')
disp(poles_observer)

% Compare dominant decay rates
ratio = abs(max(real(poles_observer))) / ...
    abs(max(real(poles_controller)));

disp('Observer/controller speed ratio:')
disp(ratio)

if all(real(poles_controller) < 0)
    disp('Controller is stable')
else
    disp('Controller is unstable')
end

if all(real(poles_observer) < 0)
    disp('Observer is stable')
else
    disp('Observer is unstable')
end

% LQGI Controller Matrix Synthesis

% Direct feed-trough matrix of the controller
Dc_pre = zeros(2,1);

% Controller-Matrices
Ac = [Ae,       -Be*K;
    -L,         A-B*K-L*C];

Bc = [Be*Dc_pre;
    -L];

Cc = [Ce, -De*K];

Dc = De*Dc_pre;

%% Build continuous controller
Controller_c = ss(Ac, Bc, Cc, Dc);

%% Check stability of the controller-system
p_controller = eig(Ac);

if all(real(p_controller) < 0)
    disp('Controller is internally stable.');
else
    disp('Caution: Controller has unstable or limited stable Poles.');
end


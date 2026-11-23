clear; clc; close all;


%% Setup Paths (Give Main.m access to the Data and Simulink folders)

% Folder containing Main.m
matlabFolder = fileparts(mfilename('fullpath'));

% Project root folder
projectFolder = fileparts(matlabFolder);

% Add remaining folders in project folder to search path
addpath(matlabFolder);
addpath(fullfile(projectFolder, 'Simulink'));
addpath(fullfile(projectFolder, 'Plots'));
addpath(fullfile(projectFolder, 'Data'));


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
    quasi = load('quasistatic_0001.mat');
    id = load('dynamic_0002.mat');

    % Parameter identification
    params = linparid(params, quasi.meas); % 2.3.3 & 2.3.4
    params = nonlinparid_Intake_Manifold(params, id, options); % 2.3.5
    params = nonlinparid_Engine_Torque_and_Inertia(params, id, options); % 2.3.7

    % Save Identified parameters (overwrites old file)
    save(fullfile(matlabFolder, 'identified_params.mat'), 'params');

    fprintf('\nAll parameters identified and saved.\n');

else

    % Load previously identified parameters
    load('identified_params.mat');

    fprintf('\nPreviously identified parameters loaded.\n');

end

% Display parameters
disp(params);

%% Validation of the identified parameters - 2.3.8
% with one of these -> 'dynamic_0003.mat' 'studentId_0004.mat' 'studentId_0005.mat' 

PlotModelValidation('ParId_Engine_Torque_and_Inertia', 'dynamic_0003.mat', ...
    projectFolder, params, options);


%% Linearization - 2.4.2 - 5.

[nom, lin] = Linearization(params, options);
lin.nom = nom;

%% Validation of the linearized model 

PlotModelValidation('LinearizedModel', 'dynamic_0003.mat', ...
    projectFolder, params, options, lin)

%% TASK 6: Check System Matrices

% Create continuous-time state-space model
Plant_s = ss(lin.As, lin.Bs, lin.Cs, lin.Ds);

% Determine dimensions
[n_states, n_inputs] = size(lin.Bs);
[n_outputs, n_states_C] = size(lin.Cs);

fprintf('\n===== SYSTEM DIMENSIONS =====\n');

fprintf('Number of states:  %d\n', n_states);
fprintf('Number of inputs:  %d\n', n_inputs);
fprintf('Number of outputs: %d\n', n_outputs);

% Check consistency of dimensions
assert(size(lin.As,1) == size(lin.As,2), ...
    'As must be square.');

assert(size(lin.As,1) == size(lin.Bs,1), ...
    'As and Bs have inconsistent dimensions.');

assert(size(lin.As,2) == size(lin.Cs,2), ...
    'As and Cs have inconsistent dimensions.');

assert(size(lin.Ds,1) == n_outputs && size(lin.Ds,2) == n_inputs, ...
    'Ds has inconsistent dimensions.');

% Check requirements
fprintf('\n===== REQUIREMENT CHECK =====\n');

if n_states == 10
    disp('PASS: System has 10 states.');
else
    disp('FAIL: System does not have 10 states.');
end

if n_inputs == 2
    disp('PASS: System has 2 inputs.');
else
    disp('FAIL: System does not have 2 inputs.');
end

if n_outputs == 1
    disp('PASS: System has 1 output.');
else
    disp('FAIL: System does not have 1 output.');
end

% Check direct feedthrough
if norm(lin.Ds,'fro') < 1e-12
    disp('PASS: No direct feedthrough (Ds = 0).');
else
    disp('FAIL: Direct feedthrough is present.');
end

%% TASK 7: Analyze Step Response

% Time vector for simulation
t = 0:0.001:10;

% Plot step responses of both inputs
figure;
step(Plant_s, t);

grid on;
title('Step Response of Linearized Engine Model');
xlabel('Time [s]');
ylabel('Engine Speed Response');

legend('Throttle Input', 'Ignition Input');

% Calculate step response data
[y_step, t_step] = step(Plant_s, t);

% Plot each input separately
figure;

subplot(2,1,1);
plot(t_step, y_step(:,1,1), 'LineWidth', 1.5);
grid on;
title('Step Response - Throttle Input');
xlabel('Time [s]');
ylabel('Speed Deviation');

subplot(2,1,2);
plot(t_step, y_step(:,1,2), 'LineWidth', 1.5);
grid on;
title('Step Response - Ignition Input');
xlabel('Time [s]');
ylabel('Speed Deviation');

% Check stability
fprintf('\n===== SYSTEM STABILITY =====\n');

if isstable(Plant_s)
    disp('Linearized system is stable.');
else
    disp('Linearized system is unstable or marginally stable.');
end

% Display poles
disp('Poles of linearized system:');
disp(pole(Plant_s));

%% TASK 8: Calculate Condition Numbers

fprintf('\n===== CONDITION NUMBERS =====\n');

cond_As = cond(lin.As);
cond_Bs = cond(lin.Bs);
cond_Cs = cond(lin.Cs);
cond_Ds = cond(lin.Ds);

fprintf('Condition number As: %.4e\n', cond_As);
fprintf('Condition number Bs: %.4e\n', cond_Bs);
fprintf('Condition number Cs: %.4e\n', cond_Cs);
fprintf('Condition number Ds: %.4e\n', cond_Ds);

% Display condition numbers in a table
Matrix = {'As'; 'Bs'; 'Cs'; 'Ds'};
ConditionNumber = [cond_As; cond_Bs; cond_Cs; cond_Ds];

ConditionTable = table(Matrix, ConditionNumber);

disp(ConditionTable);

%% Controller Synthesis
%% 1) linearized system is extended by an integrator acting on u_alpha

% Integral gain
KI = 0.5;  

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

%% 2) state feedback gain K is designed

r1 = 6;  % penalization of u_alpha
r2 = 10; % penalization of delta_u_zeta
R = diag([r1 r2]);

Q = C' * C;

K = lqr(A,B,Q,R);

%% 3) State observer is built and checked for quality

% Observer tuning parameter q:
% q -> 0:   fast observer, trust measurement
% q -> inf: slow observer, trust model
q = 0.1;

% get observer gain
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

%% 4) LQGI Controller Matrix Synthesis

% Direct feed-trough matrix of the controller
Dc_pre = zeros(2,1);

% Zero matrix for Ac
Zero_block = zeros(size(A,1), size(Ae,1));

% Controller-Matrices
Ac = [Ae,       -Be*K;
    Zero_block,         A-B*K-L*C];

Bc = [Be*Dc_pre;
    -L];

Cc = [Ce, -De*K];

Dc = De*Dc_pre;

%% 5) Check stability of controller and discretize continuous controller

% continuous controller
Controller_c = ss(Ac, Bc, Cc, Dc);

% 5.1) Check stability of the controller-system
p_controller = eig(Ac);
if all(real(p_controller) <= 0)
    disp('Controller is internally stable.');
else
    disp('Caution: Controller has unstable or limited stable Poles.');
end

% Assemble Controller-Plant-system 
Plant = ss(lin.As, lin.Bs, lin.Cs, lin.Ds);
Lopen = Plant * Controller_c;

w = logspace(-3, 3, 10000);
Ljw = squeeze(freqresp(Lopen, w));
mu = min(abs(1 + Ljw));
fprintf('Minimum return difference mu = %.4f\n', mu);

[Gm, Pm, Wcg, Wcp] = margin(Lopen);

fprintf('Gain crossover frequency = %.4f rad/s\n', Wcp);

% 5.2) Discretize the Controller
Ts = 0.001; % sampling rate - must be 0.001
Controller_d = c2d(Controller_c, Ts, 'tustin');
[Ad, Bd, Cd, Dd] = ssdata(Controller_d);
ISCS_Tu = [nom.u_alpha, 0;
    0, nom.du_ign]
ISCS_Ty = nom.omega_e
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


clear; close all; clc;

%% LOAD DATA
% Load parameters and sim options 
Parameters
options.sim_options = simopt; % simopt from Parameters.m
    
% Load measurement data for parameter identification & validation
id  = load('dynamic_0002.mat');  % id         - as recommended in exercise 6 slides (new structure: e.g. id.meas.u_alpha.signals.values)
val = load('dynamic_0003.mat');  % validation - as recommended in exercise 6 slides (new structure: e.g. val.meas.u_alpha.signals.values)

% Find the number of open figures, so a new figure can be created and used
% over and over again. This command is not necessarily needed but it is
% nice to have.
fh=findall(0,'type','figure');
options.fig_num = length(fh)+1;
options.enablePlot = 1;

fprintf('\n Data Loaded. \n')

%% Param ID - Engine torque and inertia - 2.3.7
% Define starting values of the optimization variables.
par0 = [params.eta_0_guess, params.eta_1_guess, params.beta_0_guess, params.theta_e_guess]; 

% use id data
data = id;

% Run fminsearch using the error function and find optimal parameter values      
% Define function with only one input (parameters to be optimized)
errorfnc_fminsearch = @(par0) Modelerror_Engine_Torque_and_Inertia(par0, data, options, params);
% Call fminsearch    
opt_options = optimset('Algorithm','sqp','display','iter','Maxit',30);

fprintf('\n Fminsearch started. \n')

optpar = fminsearch(errorfnc_fminsearch, par0, opt_options);

% Extract optimal parameter values                    
params.eta_0   = optpar(1);
params.eta_1   = optpar(2);
params.beta_0  = optpar(3);
params.theta_e = optpar(4);
fprintf('\n identified eta_0 = %d \n', params.eta_0)
fprintf('\n identified eta_1 = %d \n', params.eta_1)
fprintf('\n identified beta_0 = %d \n', params.beta_0)
fprintf('\n identified theta_e = %d \n', params.theta_e)

%% Param Validation - Engine torque and inertia
% use val data
data = val;

out = sim('ParId_Engine_Torque_and_Inertia.slx', data.meas.u_alpha.time(end), options.sim_options);
omega_e_valsim = out.omega_e.Data;  % extract inertia vector
tvalsim = out.omega_e.Time;         % extract time vector
    
% Graphically represent the quality of the optimization routine
figure;

plot(val.meas.omega_e.time, val.meas.omega_e.signals.values, tvalsim, omega_e_valsim);
xlabel('Time [s]');
ylabel('Outputs');
title('Validation');
legend('Measured omega_e','Modelled omega_e')


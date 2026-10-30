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

%% Param ID - Intake Manifold Volume
% Define starting values of the optimization variables.
par0 = [params.Vm_guess]; 

% use id data
data = id;

% Run fminsearch using the error function and find optimal parameter values      
% Define function with only one input (parameters to be optimized)
errorfnc_fminsearch = @(par0) Modelerror_Intake_Manifold(par0, data, options, params);
% Call fminsearch    
opt_options = optimset('Algorithm','sqp','display','iter','Maxit',30);

fprintf('\n Fminsearch started. \n')

optpar = fminsearch(errorfnc_fminsearch, par0, opt_options);

% Extract optimal parameter values                    
params.Vm = optpar(1);
fprintf('\n identified Vm = %d \n', params.Vm)

%% Param Validation - Intake Manifold
% use val data
data = val;

out = sim('ParId_V_m_model.slx', data.meas.u_alpha.time(end), options.sim_options);
pvalsim = out.p_m.Data;   % extract pressure vector
tvalsim = out.p_m.Time;   % extract time vector
    
% Graphically represent the quality of the optimization routine
figure;

plot(val.meas.p_m.time, val.meas.p_m.signals.values, tvalsim, pvalsim);
xlabel('Time [s]');
ylabel('Outputs');
title('Validation');
legend('Measured p','Modelled p')


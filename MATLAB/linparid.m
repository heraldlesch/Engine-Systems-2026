clear; close all; clc;

%% LOAD DATA

% Load Parameters
Parameters
fprintf('\n Parameters loaded \n')

% Load measurement data for parameter identification
load('quasistatic_0001.mat');

% Find the number of open figures, so a new figure can be created and used
% over and over again. This command is not necessarily needed but it is
% nice to have.
fh=findall(0,'type','figure');
options.fig_num = length(fh)+1;
options.enablePlot = 1;


%% Throttle Parameters
%least squares procedure

u_alpha = meas.u_alpha.signals.values;

M_tp = [ones(size(u_alpha)), u_alpha];
y_tp = meas.m_dot_alpha.signals.values .* sqrt(2.*params.R.*meas.T_a.signals.values) ./ meas.p_a.signals.values;

x = lsqr(M_tp,y_tp);

alpha_0 = x(1);
alpha_1 = x(2);

fprintf('\n Throttle Parameters identified and saved to workspace \n')

%% Engine Mass Flow Parameters
%least squares procedure

omega_e = meas.omega_e.signals.values;
theta_m = meas.T_m.signals.values;
m_dot_alpha = meas.m_dot_alpha.signals.values;
p_m = meas.p_m.signals.values;
lambda = meas.lambda.signals.values;

M_emf = [ones(size(omega_e)), omega_e];

C_emf = 4*pi*params.R / params.V_d;
lambda_lp = (params.V_c + params.V_d)/params.V_d - params.V_c/params.V_d * (meas.p_e.signals.values./meas.p_m.signals.values).^(1/params.kappa);

lambda_lomega = C_emf .* (theta_m .* m_dot_alpha) ./ (p_m .* omega_e .* lambda_lp) .* (1 + 1./(lambda .* params.sigma_0));

y_emf = lambda_lomega;

x = lsqr(M_emf,y_emf);
gamma_0 = x(1);
gamma_1 = x(2);


fprintf('\n Engine Mass Flow Parameters identified and saved to workspace \n')
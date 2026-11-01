function params = linparid(params, meas)

%% Throttle Parameters - 2.3.3

%least squares procedure

u_alpha = meas.u_alpha.signals.values;

M_tp = [ones(size(u_alpha)), u_alpha];
y_tp = meas.m_dot_alpha.signals.values .* sqrt(2.*params.R.*meas.T_a.signals.values) ./ meas.p_a.signals.values;

x = lsqr(M_tp,y_tp);

% Assign identified parameters 
params.alpha_0 = x(1);
params.alpha_1 = x(2);

fprintf('\n Throttle Parameters identified and saved to workspace \n')

%% Engine Mass Flow Parameters - 2.3.4

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

% Assign identified parameters 
params.gamma_0 = x(1);
params.gamma_1 = x(2);


fprintf('\n Engine Mass Flow Parameters identified and saved to workspace \n')

end
function params = nonlinparid_Engine_Torque_and_Inertia(params, data, options)

%% Param ID - Engine torque and inertia - 2.3.7

% Define starting values of the optimization variables.
par0 = [params.eta_0_guess, ...
        params.eta_1_guess, ...
        params.beta_0_guess, ...
        params.theta_e_guess]; 

% Run fminsearch using the error function and find optimal parameter values      
% Define function with only one input (parameters to be optimized)
errorfnc_fminsearch = @(par0) Modelerror_Engine_Torque_and_Inertia(...
    par0, data, options, params);

% Call fminsearch    
opt_options = optimset('Algorithm','sqp','display','iter','Maxit',30);

fprintf('\n Fminsearch started. \n')

optpar = fminsearch(errorfnc_fminsearch, par0, opt_options);

% Assign identified parameters                    
params.eta_0   = optpar(1);
params.eta_1   = optpar(2);
params.beta_0  = optpar(3);
params.theta_e = optpar(4);
fprintf('\n identified eta_0 = %d \n', params.eta_0)
fprintf('\n identified eta_1 = %d \n', params.eta_1)
fprintf('\n identified beta_0 = %d \n', params.beta_0)
fprintf('\n identified theta_e = %d \n', params.theta_e)

end


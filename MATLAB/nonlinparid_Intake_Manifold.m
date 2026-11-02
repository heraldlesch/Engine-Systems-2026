function params = nonlinparid_Intake_Manifold(params, data, options)

%% Param ID - Intake Manifold Volume - 2.3.5

% Define starting values of the optimization variables.
par0 = [params.Vm_guess]; 

% Run fminsearch using the error function and find optimal parameter values      
% Define function with only one input (parameters to be optimized)
errorfnc_fminsearch = @(par0) Modelerror_Intake_Manifold(par0, data, options, params);

% Call fminsearch    
opt_options = optimset('Display','off','MaxIter',30);

fprintf('\n Fminsearch started. \n')

optpar = fminsearch(errorfnc_fminsearch, par0, opt_options);

% Assign identified parameters                     
params.Vm = optpar(1);

fprintf('\n identified Vm = %d \n', params.Vm)

end
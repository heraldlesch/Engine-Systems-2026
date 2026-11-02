
clear; close all; clc;

% Set simulation options. The simset options are a command-based way to 
% define the simulation options. An alternative is to set them in the 
% simulink model itself. Using simset and applying the same options to all 
% simulations through the options-input in the "sim"-command is a way to 
% make sure all simulations use the same options.
    options.sim_options = simset('SrcWorkspace','current');


%Intake Manifold Volume
    % Load measurement data for parameter identification and add it to
    % fmin_data sruct
        load('dynamic_0002.mat');
        
    
    % Find the number of open figures, so a new figure can be created and used
    % over and over again. This command is not necessarily needed but it is
    % nice to have.
        fh=findall(0,'type','figure');
        options.fig_num = length(fh)+1;
        options.enablePlot = 1;
    
    % Define starting values of the optimization variables.
    par0 = [7e-3]; % Vm
        
    % Run fminsearch using the error function and find optimal parameter values      
        % Define function with only one input (parameters to be optimized)
            errorfnc_fminsearch = @(par0) error_function_V_m(par0, meas, options);
        % Call fminsearch    
            opt_options = optimset('Algorithm','sqp','display','iter','Maxit',30);
            optpar = fminsearch(errorfnc_fminsearch, par0, opt_options);
            % lb = [0, 0.5];
            % ub = [1, 1];
            % optpar = fmincon(errorfnc_fminsearch, par0, [], [], [], [], lb, ub, [], opt_options);
    
    % Extract optimal parameter values                    
        Vm = optpar(1)
        lambda  = optpar(2)
    
    % Load validation data
        load('dynamic_0003.mat');
    
    % Run a simulation with the identified optimal variable values
        data.mdot_in = ValData.mdot_in;
        data.p = ValData.p;
        simval = sim('Intake_Manifold', ValData.mdot_in.time, options.sim_options);
        tValSim = simval.tout;
        pvalsim = simval.yout{1}.Values.Data;
        
    % Graphically represent the quality of the optimization routine
        figure;
    
        subplot(2,1,1);
        plot(tValSim, ValData.mdot_in.signals.values);
            xlabel('Time [s]');
            ylabel('Inputs');
            legend('mdot_in');
    
        subplot(2,1,2);
        plot(data.p.time, data.p.signals.values, tValSim, pvalsim);
            xlabel('Time [s]');
            ylabel('Outputs');
            legend('Measured p','Modelled p')


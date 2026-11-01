function R2 = Modelerror_Engine_Torque_and_Inertia(par, data, options, params)


% Assign new parameter values (used in Simulink model NonlinearModel.slx)
params.eta_0 = par(1);
params.eta_1 = par(2);
params.beta_0 = par(3);
params.theta_e = par(4);

% Run simulation to determine the quality of the parameter values    
out = sim('ParId_Engine_Torque_and_Inertia.slx', data.meas.u_alpha.time(end), options.sim_options);
omega_e_sim = out.omega_e.Data;   % extract pressure vector
tsim = out.omega_e.Time;                  % extract time vector

% Calculate objective function
R2 = sum((data.meas.omega_e.signals.values - omega_e_sim).^2);

% plot curves
if options.enablePlot
    figure(options.fig_num);
    plot(data.meas.omega_e.time, data.meas.omega_e.signals.values, 'b'); hold on; grid on;
    plot(tsim, omega_e_sim, '-r' ); hold off;
    xlabel('Time [s]');
    ylabel('Model Ooutput [-]');
	legend({'Measurements','Simulation'},'Location','NorthEast')
    title('Identification')
    xlim([data.meas.omega_e.time(1) data.meas.omega_e.time(end)]);
    ylim([min(data.meas.omega_e.signals.values)-1/10*mean(data.meas.omega_e.signals.values)...
        max(data.meas.omega_e.signals.values)+1/10*mean(data.meas.omega_e.signals.values)]);
    drawnow;
end
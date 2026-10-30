function R2 = Modelerror_Intake_Manifold(par, data, options, params)


% Assign new parameter values (used in Simulink model NonlinearModel.slx)
params.Vm = par(1);

% Run simulation to determine the quality of the parameter values    
out = sim('ParId_V_m_model.slx', data.meas.u_alpha.time(end), options.sim_options);
psim = out.p_m.Data;   % extract pressure vector
tsim = out.p_m.Time;                  % extract time vector

% Calculate objective function
R2 = sum((data.meas.p_m.signals.values - psim).^2);

% plot curves
if options.enablePlot
    figure(options.fig_num);
    plot(data.meas.p_m.time, data.meas.p_m.signals.values, 'b'); hold on; grid on;
    plot(tsim, psim, '-r' ); hold off;
    xlabel('Time [s]');
    ylabel('Model Ooutput [-]');
	legend({'Measurements','Simulation'},'Location','NorthEast')
    title('Identification')
    xlim([data.meas.p_m.time(1) data.meas.p_m.time(end)]);
    ylim([min(data.meas.p_m.signals.values)-1/10*mean(data.meas.p_m.signals.values)...
        max(data.meas.p_m.signals.values)+1/10*mean(data.meas.p_m.signals.values)]);
    drawnow;
end
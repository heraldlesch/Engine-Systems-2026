function V = error_function_V_m(par, data, options)





% Assign new parameter values (used in Simulink model SystemModel.slx)
    Vm = par(1);

% Run simulation to determine the quality of the parameter values    
    simout = sim('ParId_V_m_model.slx', data.mdot_in.time, options.sim_options);
    psim = simout.yout{1}.Values.Data;   % extract pressure vector
    tsim = simout.tout;                  % extract time vector

% Calculate objective function
    V = sum((data.p.signals.values-psim).^2);

% plot curves
if options.enablePlot
    figure(options.fig_num);
    plot(data.p.time, data.p.signals.values, 'b'); hold on; grid on;
    plot(tsim, psim, '-r' ); hold off;
    xlabel('Time [s]');
    ylabel('Model Ooutput [-]');
	legend({'Measurements','Simulation'},'Location','NorthEast')
    set(gca,'XLim',[data.p.time(1) data.p.time(end)]);
    set(gca,'YLim',[min(data.p.signals.values)-1/10*mean(data.p.signals.values) max(data.p.signals.values)+1/10*mean(data.p.signals.values)]);
    drawnow;
end
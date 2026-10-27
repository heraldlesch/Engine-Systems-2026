clear; close all; clc;

Parameters
fprintf('\n Parameters loaded \n')


% Load measurement data for parameter identification
load('quasistatic_0001.mat');


%Throttle Parameters

    % Display measurement values
    % figure;
    % 
    % subplot(2,1,1);
    % plot(meas.time, IdData.mdot_in.signals.values);
    % xlabel('Time [s]');
    % legend('mdot_in', 'Location','NorthWest');
    % 
    % subplot(2,1,2);
    % plot(IdData.p.time, IdData.p.signals.values);
    % xlabel('Time [s]');
    % ylabel('pressure [Pa]');
    % legend('p(t)','Location','NorthWest');
    
    % Find the number of open figures, so a new figure can be created and used
    % over and over again. This command is not necessarily needed but it is
    % nice to have.
    fh=findall(0,'type','figure');
    options.fig_num = length(fh)+1;
    options.enablePlot = 1;
    
    
    %least squares procedure
    u = meas.u_alpha.signals.values;

    M = [ones(size(u)) u];
    y = meas.m_dot_alpha.signals.values .* sqrt(2.*params.R.*meas.T_a.signals.values) ./ meas.p_a.signals.values;
    x = lsqr(M,y);
    alpha_0 = x(1);
    alpha_1 = x(2);
    
    

    % Graphically represent the quality of the optimization routine
    % figure;
    % 
    % subplot(2,1,1);
    % plot(tValSim, ValData.mdot_in.signals.values);
    % xlabel('Time [s]');
    % ylabel('Inputs');
    % legend('mdot_in');
    % 
    % subplot(2,1,2);
    % plot(data.p.time, data.p.signals.values, tValSim, pvalsim);
    % xlabel('Time [s]');
    % ylabel('Outputs');
    % legend('Measured p','Modelled p')


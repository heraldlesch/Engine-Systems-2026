clear; clc; close all;

% Load data 
Parameters
fprintf('\n Parameters loaded \n')


% simulate
u_alpha_vec = 0:10:100;
for i = 1:length(u_alpha_vec)
    fprintf('\nstarting iter %d \n', i)

    u_alpha = u_alpha_vec(i);
    out = sim('NonlinearModel.slx');
    
    figure
    plot(out.omega_e)
    xlabel('t')
    ylabel('\omega_e')
    grid on
    title(sprintf('u_alpha = %d', u_alpha))
end

fprintf('\nsims finished \n')


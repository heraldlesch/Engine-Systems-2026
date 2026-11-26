function J = AnalyzeNonlinearController(out, nom, tuning)
% Time-domain plots of speed, throttle, and ignition + competition cost J

t         = out.omega_e.Time;
omega     = out.omega_e.Data;       % [rad/s]
u_alpha   = out.u_alpha_cmd.Data;   % [%]
du_ign    = out.du_ign_cmd.Data;    % [deg CA]

% Align reference speed with t in case omega_ref has its own time vector
t_ref     = out.omega_ref.Time;
omega_ref_raw = out.omega_ref.Data;
if isequal(t, t_ref)
    omega_ref = omega_ref_raw;
else
    omega_ref = interp1(t_ref, omega_ref_raw, t, 'previous', 'extrap');
end

% --- Time-Domain Plot (Speed + Both Actuators) ---
figure('Name', 'Nonlinear Closed-Loop Time-Domain Response');

subplot(3, 1, 1);
plot(t, omega_ref, '--k', 'LineWidth', 1.3, 'DisplayName', '\omega_{e,ref}');
hold on;
plot(t, omega, 'b', 'LineWidth', 1.5, 'DisplayName', '\omega_e');
yline(50, ':r', 'LineWidth', 1.2, 'DisplayName', 'Stall limit (50 rad/s)');
ylabel('\omega_e [rad/s]');
title(sprintf('Closed-Loop Response (K_I=%.2f, r_1=%.2f, r_2=%.2f, q=%.3g)', ...
    tuning.KI, tuning.r1, tuning.r2, tuning.q));
legend('Location', 'best');
grid on;

subplot(3, 1, 2);
plot(out.u_alpha_cmd.Time, out.u_alpha_cmd.Data, 'b', 'LineWidth', 1.5);
hold on;
ylabel('u_\alpha [%]');
grid on;

subplot(3, 1, 3);
plot(out.du_ign_cmd.Time, out.du_ign_cmd.Data, 'b', 'LineWidth', 1.5);
hold on;
ylabel('\Delta u_\zeta [^\circ CA]');
xlabel('Time t [s]');
grid on;

% --- Competition Cost Function J ---
k_zeta = 2.3345e-4; % [(deg CA)^-2]
Delta_u_zeta_nom = nom.du_ign; % [deg CA]

k_max = 1 / (1 - k_zeta * (Delta_u_zeta_nom^2));
speed_err_term = 0.2 * (k_max^2.5) * abs(omega - omega_ref);
du_alpha_dt = gradient(u_alpha, t);
throttle_rate_term = abs(du_alpha_dt);

J = trapz(t, speed_err_term + throttle_rate_term);
fprintf('Competition cost J = %.6f\n', J);
end
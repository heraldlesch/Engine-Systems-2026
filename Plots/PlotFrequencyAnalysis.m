function PlotFrequencyAnalysis(Lopen_c, mu)
% Plot SISO Bode (mag & phase) and Nyquist (with unit circle and -1+0j)

s = allmargin(Lopen_c);
Wcp = s.PMFrequency;
Pm  = s.PhaseMargin;

% 1. Bode plot
figure('Name', 'Open-Loop Bode Analysis');
bode(Lopen_c);
grid on;
if isempty(Wcp)
    maxWcp = NaN;
    title(sprintf('Open-Loop Bode | No 0 dB crossover, \\mu = %.3f (limit \\ge 0.5)', mu));
else
    maxWcp = max(Wcp);
    title(sprintf('Open-Loop Bode | max W_{cp} = %.2f rad/s (limit \\leq 4), \\mu = %.3f (limit \\geq 0.5)', ...
        maxWcp, mu));
end

% 2. Nyquist plot
figure('Name', 'Open-Loop Nyquist Analysis');
nyquist(Lopen_c);
hold on;

theta = linspace(0, 2*pi, 400);
plot(cos(theta), sin(theta), '--k', 'LineWidth', 1.0, 'DisplayName', 'Unit Circle');
plot(-1, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 1.8, 'DisplayName', '(-1, 0j)');

% Zoom in on the region of interest around (-1, 0j) and the unit circle
xlim([-2.5, 2.5]);
ylim([-2.5, 2.5]);

legend('Location', 'best');
grid on;
title('Open-Loop Nyquist Plot');

fprintf('All gain crossover frequencies [rad/s]: ');
disp(Wcp.');
fprintf('Phase margins at gain crossovers [deg]: ');
disp(Pm.');
end
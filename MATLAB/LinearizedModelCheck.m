%% Check System Matrices
fprintf('\n============== CHECK LINEARIZED MODEL ==============\n');

% Create continuous-time state-space model
Plant_s = ss(lin.As, lin.Bs, lin.Cs, lin.Ds);

% Determine dimensions
[n_states, n_inputs] = size(lin.Bs);
[n_outputs, n_states_C] = size(lin.Cs);

fprintf('\n==> CHECK SYSTEM DIMENSIONS:\n');

fprintf('Number of states (need to be 10!):  %d\n', n_states);
fprintf('Number of inputs  (need to be 2!):  %d\n', n_inputs);
fprintf('Number of outputs (need to be 1!): %d\n', n_outputs);

% Check consistency of dimensions
assert(size(lin.As,1) == size(lin.As,2), 'As must be square.');
assert(size(lin.As,1) == size(lin.Bs,1), 'As and Bs have inconsistent dimensions.');
assert(size(lin.As,2) == size(lin.Cs,2), 'As and Cs have inconsistent dimensions.');
assert(size(lin.Ds,1) == n_outputs && size(lin.Ds,2) == n_inputs, 'Ds has inconsistent dimensions.');

% Check direct feedthrough
if norm(lin.Ds,'fro') < 1e-12
    disp('GOOD: No direct feedthrough (Ds = 0).');
else
    disp('FAIL: Direct feedthrough is present.');
end

%% Analyze Step Response
fprintf('\n==> CHECK STEP RESPONSE:\n');

t_s = (0:1e-3:10)';
s   = 0.05 * (t_s >= 1);                       % 5 % step at t = 1 s
names = {'Throttle u_\alpha', 'Ignition \Deltau_\zeta', 'Both inputs'};
sel   = [1 0; 0 1; 1 1];

figure('Name', 'Step response linearized model');
for k = 1:3
    dw = lsim(Plant_s, s*sel(k,:), t_s) * nom.omega_e;   % [rad/s]
    subplot(3,1,k); plot(t_s, dw, 'LineWidth', 1.5); grid on;
    title([names{k} ': +5 % step at t = 1 s']); ylabel('\Delta\omega_e [rad/s]');
end
xlabel('Time [s]');

% Sign check of static gains
G0 = dcgain(Plant_s);
fprintf('Static gains: throttle %+.3f (expect > 0), ignition %+.3f (expect < 0)\n', G0(1), G0(2));

%% Check stability
fprintf('\n==> CHECK SYSTEM STABILITY:\n');

if isstable(Plant_s)
    disp('Linearized system is stable.');
else
    disp('Linearized system is unstable or marginally stable.');
end

% Display poles
disp('Poles of linearized system:');
disp(pole(Plant_s));

%% Calculate Condition Numbers

fprintf('\n==> CHECK CONDITION NUMBERS:\n');

cond_As = cond(lin.As);
cond_Bs = cond(lin.Bs);
cond_Cs = cond(lin.Cs);
cond_Ds = cond(lin.Ds);

fprintf('Condition number As: %.4e\n', cond_As);
fprintf('Condition number Bs: %.4e\n', cond_Bs);
fprintf('Condition number Cs: %.4e\n', cond_Cs);
fprintf('Condition number Ds: %.4e\n', cond_Ds);

% Display condition numbers in a table
Matrix = {'As'; 'Bs'; 'Cs'; 'Ds'};
ConditionNumber = [cond_As; cond_Bs; cond_Cs; cond_Ds];

ConditionTable = table(Matrix, ConditionNumber);

disp(ConditionTable);
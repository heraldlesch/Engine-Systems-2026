function ctrl = ControllerSynthesis(lin, tuning)


%% 1) linearized system is extended by an integrator acting on u_alpha

% Extension matrices
Ae = 0;
Be = [1 0];
Ce = [tuning.KI; 0];
De = eye(2);

% Extended plant
A = [lin.As, lin.Bs*Ce;
    zeros(1,size(lin.As,2)), Ae];
B = [lin.Bs*De;
    Be];
C = [lin.Cs, lin.Ds*Ce];
D = lin.Ds*De;

%% 2) state feedback gain K is designed

R = diag([tuning.r1 tuning.r2]);

Q = C' * C;

K = lqr(A,B,Q,R);

%% 3) State observer is built and checked for quality

% get observer gain
L = lqr(A', C', B*B', tuning.q)';

% Closed-loop poles - To check whether all 11 modes decay
poles_feedback = eig(A - B*K);

% Observer poles - To check whether the estimation error converges to zero
poles_observer = eig(A - L*C);

disp('Ideal state-feedback poles (A-BK):')
disp(poles_feedback)

disp('Observer estimation-error poles (A-LC):')
disp(poles_observer)

% Compare dominant decay rates
ratio = abs(max(real(poles_observer))) / ...
    abs(max(real(poles_feedback)));

disp('Observer/state-feedback decay-rate ratio:')
disp(ratio)

if all(real(poles_feedback) < 0)
    disp('Ideal state-feedback system is asymptotically stable.')
else
    disp('Ideal state-feedback system is NOT asymptotically stable.')
end

if all(real(poles_observer) < 0)
    disp('Observer estimation-error dynamics are asymptotically stable.')
else
    disp('Observer estimation-error dynamics are NOT asymptotically stable.')
end

%% 4) LQGI Controller Matrix Synthesis

% Direct feed-trough matrix of the controller
Dc_pre = zeros(2,1);

% Zero matrix for Ac
Zero_block = zeros(size(A,1), size(Ae,1));

% Controller-Matrices
Ac = [Ae,       -Be*K;
    Zero_block,         A-B*K-L*C];

Bc = [Be*Dc_pre;
    -L];

Cc = [Ce, -De*K];

Dc = De*Dc_pre;

%% 5) Check stability of controller and discretize continuous controller

% Continuous-time controller and plant
Controller_c = ss(Ac, Bc, Cc, Dc);
Plant = ss(lin.As, lin.Bs, lin.Cs, lin.Ds);

%% 5.1) Open-loop frequency-domain analysis

% Series connection: L(s) = P(s)*C(s)
Lopen_c = Plant * Controller_c;

% Minimum return difference
w = logspace(-3, 3, 10000);
Ljw = squeeze(freqresp(Lopen_c, w));

mu = min(abs(1 + Ljw));

fprintf('\n============== OPEN-LOOP ANALYSIS ==============\n');
fprintf('Minimum return difference mu = %.4f\n', mu);

% Gain/phase margins and crossover frequency
[Gm, Pm, Wcg, Wcp] = margin(Lopen_c);

fprintf('Gain crossover frequency = %.4f rad/s\n', Wcp);
fprintf('Phase margin = %.4f deg\n', Pm);


%% 5.2) Closed-loop stability

% Combined plant-controller matrix
Tclosed_c = feedback(Lopen_c, 1);

% Obtain the poles
poles_closed_c = pole(Tclosed_c);

% Check asymptotic stability
tol = 1e-8;
stable_closed_c = all(real(poles_closed_c) < -tol);

fprintf('\n============== CLOSED-LOOP STABILITY ==============\n')

if stable_closed_c
    disp('Continuous closed loop is stable.');
else
    warning('Continuous closed loop is not asymptotically stable.');
end

fprintf('Largest pole real part: %.4f\n', ...
    max(real(poles_closed_c)));

%% 5.3) Discretization

% Discretize the controller
Controller_d = c2d(Controller_c, tuning.Ts, 'tustin');
[Ad, Bd, Cd, Dd] = ssdata(Controller_d);

% ... and the plant
Plant_d = c2d(Plant, tuning.Ts, 'zoh');
[Ap, Bp, Cp, Dp] = ssdata(Plant_d);

% 5.4) Discrete closed-loop stability

% Discrete series connection: L(s) = P(s)*C(s)
Lopen_d = Plant_d * Controller_d;

% Close negative feedback loop
Tclosed_d = feedback(Lopen_d, 1);

% Calculate discrete closed-loop poles
poles_closed_d = pole(Tclosed_d);

% Discrete stability: all poles inside unit circle
rho = max(abs(poles_closed_d));

stable_closed_d = rho < 1;

fprintf('\n============= DISCRETE CLOSED-LOOP STABILITY =============\n');

if stable_closed_d
    disp('Discrete closed loop is asymptotically stable.');
else
    warning('Discrete closed loop is NOT asymptotically stable.');
end

fprintf('Maximum pole magnitude = %.8f\n', rho);

%% Return controller results

% Extended plant
ctrl.A = A;
ctrl.B = B;
ctrl.C = C;
ctrl.D = D;

% Feedback and observer gains
ctrl.K = K;
ctrl.L = L;

% Continuous controller
ctrl.Ac = Ac;
ctrl.Bc = Bc;
ctrl.Cc = Cc;
ctrl.Dc = Dc;

% Discrete controller
ctrl.Ad = Ad;
ctrl.Bd = Bd;
ctrl.Cd = Cd;
ctrl.Dd = Dd;

% Complete state-space objects
ctrl.Controller_c = Controller_c;
ctrl.Controller_d = Controller_d;
ctrl.Plant = Plant;


end
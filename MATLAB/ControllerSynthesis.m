%% 1) linearized system is extended by an integrator acting on u_alpha

% Integral gain
KI = 0.5;   % tunable

% Extension matrices
Ae = 0;
Be = [1 0];
Ce = [KI; 0];
De = eye(2);

% Extended plant
A = [lin.As, lin.Bs*Ce;
    zeros(1,size(lin.As,2)), Ae];
B = [lin.Bs*De;
    Be];
C = [lin.Cs, lin.Ds*Ce];
D = lin.Ds*De;

%% 2) state feedback gain K is designed

r1 = 6;  % penalization of u_alpha  % tunable
r2 = 10; % penalization of delta_u_zeta % tunable
R = diag([r1 r2]);

Q = C' * C;

K = lqr(A,B,Q,R);

%% 3) State observer is built and checked for quality

% Observer tuning parameter q:
% q -> 0:   fast observer, trust measurement
% q -> inf: slow observer, trust model
q = 0.1; % tunable

% get observer gain
L = lqr(A', C', B*B', q)';


% Closed-loop poles
poles_controller = eig(A - B*K);

% Observer poles
poles_observer = eig(A - L*C);

disp('Closed-loop poles:')
disp(poles_controller)

disp('Observer poles:')
disp(poles_observer)

% Compare dominant decay rates
ratio = abs(max(real(poles_observer))) / ...
    abs(max(real(poles_controller)));

disp('Observer/controller speed ratio:')
disp(ratio)

if all(real(poles_controller) < 0)
    disp('Controller is stable')
else
    disp('Controller is unstable')
end

if all(real(poles_observer) < 0)
    disp('Observer is stable')
else
    disp('Observer is unstable')
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

% continuous controller
Controller_c = ss(Ac, Bc, Cc, Dc);

% 5.1) Check stability of the controller-system
p_controller = eig(Ac);
if all(real(p_controller) <= 0)
    disp('Controller is internally stable.');
else
    disp('Caution: Controller has unstable or limited stable Poles.');
end

% Assemble Controller-Plant-system 
Plant = ss(lin.As, lin.Bs, lin.Cs, lin.Ds);
Lopen = Plant * Controller_c;

w = logspace(-3, 3, 10000);
Ljw = squeeze(freqresp(Lopen, w));
mu = min(abs(1 + Ljw));
fprintf('Minimum return difference mu = %.4f\n', mu);

[Gm, Pm, Wcg, Wcp] = margin(Lopen);

fprintf('Gain crossover frequency = %.4f rad/s\n', Wcp);

% 5.2) Discretize the Controller
Ts = 0.001; % sampling rate - must be 0.001
Controller_d = c2d(Controller_c, Ts, 'tustin');
[Ad, Bd, Cd, Dd] = ssdata(Controller_d);
% System parameters
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

sys = ss(A, B, C, D);

% Define the weight matrices for the cost function
Q = [10 0; 0 1];  % Penalize deviations in position heavily
R = 1;            % Penalize control effort

% Compute the optimal feedback gain K using the LQR function
[K, S, e] = lqr(A, B, Q, R);

% Display the results
disp('Optimal feedback gain K:');
disp(K);

% Closed-loop system simulation
t = 0:0.01:10;        % Time vector
x0 = [-5; 0];          % Initial condition

% Closed-loop system: Acl = (A - B*K)
Acl = A - B*K;
sys_cl = ss(Acl, B, C, D);

% Simulate the system response
[y, t, x] = initial(sys_cl, x0, t);

% Plot the results
figure;
plot(t, x);
title('Closed-Loop System Response');
xlabel('Time (s)');
ylabel('State variables');

% Use cell array to define legend labels explicitly
legend({'Position', 'Velocity'});

grid on;

%% Exploring system response.

[y_orig, t_orig] = step(sys, t);

[y_ctrl, t_ctrl] = step(sys_cl, t);

% Time vector for simulation
t = 0:0.01:10;        % Time vector

% Plot both step responses
figure;
plot(t_orig, y_orig, 'r--', 'LineWidth', 2);  % Original system response (red dashed line)
hold on;
plot(t_ctrl, y_ctrl, 'b-', 'LineWidth', 2);   % Controlled system response (blue solid line)
title('Step Response: Original vs Controlled System');
xlabel('Time (s)');
ylabel('Position');
legend({'Original System', 'Controlled System'});
grid on;
hold off;
% controlled system has a huge steady state error.


%% LQR on system augmented with integral action

A_aug = [A, zeros(2,1); -C, 0];  % Augmented A matrix with integral state
B_aug = [B; 0];                  % Augmented B matrix

% Define the weight matrices for the cost function
Q = [10 0 0; 0 1 0; 0 0 100];  % Penalize the augmented state (integrator)
R = 1;                         % Penalize control effort

% Compute the LQR gain for the augmented system
K_aug = lqr(A_aug, B_aug, Q, R);

% Separate the gain into state feedback and integral action components
Kx = K_aug(1:2);  % State feedback part
Ki = K_aug(3);    % Integral gain

% Closed-loop system simulation with integral action
Acl_aug = [A - B*Kx, -B*Ki; -C, 0];  % Closed-loop A matrix with integral action
Bcl_aug = [0; 0; 1];                 % Input matrix for the reference
Ccl_aug = [C, 0];                    % Output matrix

sys_controlled_aug = ss(Acl_aug, Bcl_aug, Ccl_aug, D);

% Time vector for simulation
t = 0:0.01:10;        % Time vector

% Step response for the controlled system with integral action
[y_ctrl_aug, t_ctrl_aug] = step(sys_controlled_aug, t);

% Step response for the original (open-loop) system
sys = ss(A, B, C, D);  % Open-loop system
[y_orig, t_orig] = step(sys, t);

% Plot both step responses
figure;
plot(t_orig, y_orig, 'r--', 'LineWidth', 2);  % Original system response (red dashed line)
hold on;
plot(t_ctrl_aug, y_ctrl_aug, 'b-', 'LineWidth', 2);  % Controlled system response with integrator (blue solid line)
title('Step Response: Original vs Controlled System with Integrator');
xlabel('Time (s)');
ylabel('Position');
legend({'Original System', 'Controlled System with Integrator'});
grid on;
hold off;

%% Modelling the Pololu 3Pi+ Robot with LQR.

num = 0.0006262;
den = [1 0.04342 0.000648];
sys_tf = tf(num, den);

[Ar, Br, Cr, Dr] = tf2ss(num, den);

sys_r = ss(Ar, Br, Cr, Dr);

% Define the weight matrices for the cost function
Q = [10 0; 0 1];  % Penalize deviations in position heavily
R = 1;            % Penalize control effort

% Compute the optimal feedback gain K using the LQR function
[K, S, e] = lqr(Ar, Br, Q, R);

% Display the results
disp('Optimal feedback gain K:');
disp(K);

% Closed-loop system simulation
t = 0:0.01:10;        % Time vector

% Closed-loop system: Acl = (A - B*K)
Arcl = Ar - Br*K;
sys_rcl = ss(Arcl, Br, Cr, Dr);

% Time vector for simulation
t = 0:0.01:10;        % Time vector

step(sys_rcl)

%% Modelling the Pololu 3Pi+ Robot with LQR and Increased Integral Action.

% Transfer function parameters
num = 0.0006262;
den = [1 0.04342 0.000648];
sys_tf = tf(num, den);

% Convert transfer function to state-space representation
[Ar, Br, Cr, Dr] = tf2ss(num, den);
sys_r = ss(Ar, Br, Cr, Dr);

% Check stability of the open-loop system
disp('Poles of the original system:');
disp(eig(Ar));

% Define the weight matrices for the cost function
% Increase integral action while keeping state feedback strong
Q = [1000 0 0; 0 100 0; 0 0 100];  % Higher weight on integrator to speed up correction
R = 1;  % Allow sufficient control effort

% Augment the system to include integral action
n = size(Ar,1);  % Number of states
p = size(Cr,1);  % Number of outputs

% Augment state-space model: Ar_aug = [Ar, zeros(n,p); -Cr, 0], Br_aug = [Br; 0]
Ar_aug = [Ar, zeros(n, p); -Cr, 0];  % Augmented A matrix with integral state
Br_aug = [Br; 0];  % Augmented B matrix
Cr_aug = [Cr, 0];  % Augmented C matrix (same output)

% Compute the LQR gain for the augmented system
K_aug = lqr(Ar_aug, Br_aug, Q, R);

% Separate the gain into state feedback (for Ar) and integral action (for integrator)
Kx = K_aug(1:n);  % State feedback part
Ki = K_aug(end)*500;  % Integral gain (increased)

% Display the feedback gains
disp('State feedback gain Kx:');
disp(Kx);
disp('Integral gain Ki (increased):');
disp(Ki);

% Closed-loop system with increased integral action
Arcl_aug = [Ar - Br*Kx, -Br*Ki; -Cr, 0];  % Closed-loop augmented A matrix
Bcl_aug = [0; 0; 1];  % Input matrix for step reference (integral action)
Ccl_aug = [Cr, 0];  % Output matrix

% Create the closed-loop system with integral action
sys_rcl_aug = ss(Arcl_aug, Bcl_aug, Ccl_aug, Dr);

% Check the poles of the closed-loop system for stability
disp('Poles of the closed-loop system with increased integral action:');
disp(eig(Arcl_aug));

% Simulate the step response of the closed-loop system with increased integral gain
t = 0:0.01:20;  % Time vector for simulation
figure;
step(sys_rcl_aug, t);  % Step response
title('Step Response: Closed-Loop System with Increased Integral Action');
grid on;







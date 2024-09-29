% System parameters
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

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
sys_cl = ss(Acl, [], C, D);

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

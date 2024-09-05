%% Defining the system
num = 1;
den = [1 10 20];
sys = tf(num, den);
step(sys)

% kp = 1;
% ki = 0.01;
% kd = 0.01;
kp = 158562163.8205;
ki = 249995612.9873;
kd = 18363560.8069;

c = pid(kp, ki, kd);

T = feedback(c*sys, 1); % Negative feedback
step(T)


%% Implementing PSO to tune PID Controller
% PSO Parameters
num_particles = 30;  % Number of particles in the swarm
max_iterations = 100;  % Maximum number of iterations
w = 0.7;  % Inertia weight
c1 = 1.5;  % Cognitive acceleration coefficient
c2 = 1.5;  % Social acceleration coefficient

% Random initialization of particles (PID parameters: [Kp, Ki, Kd])
Kp_min = 0; Kp_max = 10;  % Bounds for Kp
Ki_min = 0; Ki_max = 10;  % Bounds for Ki
Kd_min = 0; Kd_max = 10;  % Bounds for Kd

particles = [Kp_min + rand(num_particles, 1) * (Kp_max - Kp_min), ...
             Ki_min + rand(num_particles, 1) * (Ki_max - Ki_min), ...
             Kd_min + rand(num_particles, 1) * (Kd_max - Kd_min)];

% Initialize velocity of particles
velocity = zeros(num_particles, 3);  % [Kp_velocity, Ki_velocity, Kd_velocity]

% Initialize personal and global best positions
personal_best = particles;
personal_best_cost = inf(num_particles, 1);
global_best = particles(1, :);
global_best_cost = inf;

% Objective function: Integral of Absolute Error (IAE)
objective_function = @(Kp, Ki, Kd) pid_cost_function(Kp, Ki, Kd, sys);

% PSO Loop
for iter = 1:max_iterations
    for i = 1:num_particles
        % Evaluate the cost for each particle (PID gains)
        Kp = particles(i, 1); Ki = particles(i, 2); Kd = particles(i, 3);
        cost = objective_function(Kp, Ki, Kd);
        
        % Update personal best if current cost is lower
        if cost < personal_best_cost(i)
            personal_best(i, :) = [Kp, Ki, Kd];
            personal_best_cost(i) = cost;
        end
        
        % Update global best if current cost is lower
        if cost < global_best_cost
            global_best = [Kp, Ki, Kd];
            global_best_cost = cost;
        end
    end
    
    % Update velocities and positions of particles
    for i = 1:num_particles
        r1 = rand(); r2 = rand();
        velocity(i, :) = w * velocity(i, :) + c1 * r1 * (personal_best(i, :) - particles(i, :)) ...
                                            + c2 * r2 * (global_best - particles(i, :));
        particles(i, :) = particles(i, :) + velocity(i, :);  % Update particle position (PID gains)
    end
    
    % Display iteration and global best cost
    fprintf('Iteration %d: Best Cost = %.4f\n', iter, global_best_cost);
end

% Final tuned PID gains
Kp_opt = global_best(1);
Ki_opt = global_best(2);
Kd_opt = global_best(3);
fprintf('Optimal PID Gains: kp = %.4f, ki = %.4f, kd = %.4f\n', Kp_opt, Ki_opt, Kd_opt);

% Objective function definition (example: IAE cost function)
function cost = pid_cost_function(Kp, Ki, Kd, sys)
    % Define the PID controller transfer function
    C = pid(Kp, Ki, Kd);
    
    % Closed-loop system with the PID controller
    T = feedback(C * sys, 1);
    
    % Step response
    [y, t] = step(T);
    
    % Calculate the error (difference from desired output)
    e = 1 - y;  % Assuming unit step input
    
    % Integral of Absolute Error (IAE) as cost function
    cost = trapz(t, abs(e));  % Numerical integration of the absolute error
end














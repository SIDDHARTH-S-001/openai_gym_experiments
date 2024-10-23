% Define the 2nd order system as a transfer function
num = [1]; % Numerator of transfer function
den = [1, 2*0.7*1, 1^2]; % Denominator (example with zeta=0.7, wn=1)
G = tf(num, den); % Continuous-time transfer function

% PSO Parameters
num_particles = 20;     % Number of particles in the swarm
num_iterations = 500;    % Number of iterations for optimization
c1 = 1.5;               % Cognitive parameter (self-learning)
c2 = 1.5;               % Social parameter (swarm-learning)
w = 0.9;                % Inertia weight
dim = 3;                % Three dimensions: Kp, Ki, Kd

% Define the bounds for Kp, Ki, and Kd
Kp_bounds = [0, 100];
Ki_bounds = [0, 50];
Kd_bounds = [0, 20];

% Initialize particles' positions and velocities
positions = rand(num_particles, dim) .* [Kp_bounds(2), Ki_bounds(2), Kd_bounds(2)];
velocities = zeros(num_particles, dim);
personal_best_positions = positions;
global_best_position = zeros(1, dim);

% Initialize objective values
personal_best_scores = inf(num_particles, 1);
global_best_score = inf;

% PSO optimization loop
for iter = 1:num_iterations
    for i = 1:num_particles
        Kp = positions(i, 1);
        Ki = positions(i, 2);
        Kd = positions(i, 3);

        % Create the PID controller
        C = pid(Kp, Ki, Kd);
        closed_loop_system = feedback(C * G, 1);
        
        % Calculate system response metrics
        [y, t] = step(closed_loop_system);
        setpoint = 1;
        error = setpoint - y;
        
        % Calculate max overshoot, settling time, and total error
        info = stepinfo(closed_loop_system);
        Overshoot = info.Overshoot / 100; % Convert to fractional overshoot
        SettlingTime = info.SettlingTime;  % Settling time in seconds
        
        % Objective function: equally weigh error, overshoot, and settling time
        objective_value = mean(abs(error)) + Overshoot + SettlingTime;
        
        % Update personal best
        if objective_value < personal_best_scores(i)
            personal_best_scores(i) = objective_value;
            personal_best_positions(i, :) = positions(i, :);
        end

        % Update global best
        if objective_value < global_best_score
            global_best_score = objective_value;
            global_best_position = positions(i, :);
        end
    end
    
    % Update particles' velocities and positions
    for i = 1:num_particles
        r1 = rand;
        r2 = rand;
        velocities(i, :) = w * velocities(i, :) + c1 * r1 * (personal_best_positions(i, :) - positions(i, :)) + c2 * r2 * (global_best_position - positions(i, :));
        positions(i, :) = positions(i, :) + velocities(i, :);

        % Ensure positions remain within bounds
        positions(i, 1) = max(min(positions(i, 1), Kp_bounds(2)), Kp_bounds(1));
        positions(i, 2) = max(min(positions(i, 2), Ki_bounds(2)), Ki_bounds(1));
        positions(i, 3) = max(min(positions(i, 3), Kd_bounds(2)), Kd_bounds(1));
    end
    
    % Display progress
    disp(['Iteration ' num2str(iter) ' - Best Objective Value: ' num2str(global_best_score)]);
end

% Display the best found PID gains
Kp_optimal = global_best_position(1);
Ki_optimal = global_best_position(2);
Kd_optimal = global_best_position(3);

disp(['Optimal Kp: ' num2str(Kp_optimal)]);
disp(['Optimal Ki: ' num2str(Ki_optimal)]);
disp(['Optimal Kd: ' num2str(Kd_optimal)]);

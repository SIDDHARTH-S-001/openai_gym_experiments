% Define symbolic variables
syms l1 l2 theta1 theta2 x y

% Define the forward kinematics equations
eq1 = x == l1 * cos(theta1) + l2 * cos(theta1 + theta2);
eq2 = y == l1 * sin(theta1) + l2 * sin(theta1 + theta2);

% Solve the equations for theta1 and theta2
solutions = solve([eq1, eq2], [theta1, theta2]);

% Display the solutions
disp('Solutions for theta1:');
disp(solutions.theta1);
disp('Solutions for theta2:');
disp(solutions.theta2);

%% Newton Raphson Technique (basic example)

% Define the function f(x) and its derivative f'(x)
f = @(x) x^2 - 2;
df = @(x) 2*x;

% Initial guess
x0 = 1.5;

% Tolerance for stopping criterion
tol = 1e-6;

% Maximum number of iterations
max_iter = 100;

% Newton-Raphson iteration
x = x0;
for i = 1:max_iter
    x_new = x - f(x)/df(x);
    
    % Check for convergence
    if abs(x_new - x) < tol
        break;
    end
    
    % Update the current value
    x = x_new;
end

% Display the result
fprintf('The root is approximately: %.6f\n', x);
fprintf('Number of iterations: %d\n', i);

%% Bisection method

% Define the function
f = @(x) x^2 - 2;

% Initial interval [a, b]
a = 0;
b = 2;

% Tolerance for stopping criterion
tol = 1e-6;

% Maximum number of iterations
max_iter = 100;

% Bisection Method iteration
for i = 1:max_iter
    % Calculate the midpoint
    c = (a + b) / 2;
    
    % Evaluate the function at the midpoint
    fc = f(c);
    
    % Check for convergence
    if abs(fc) < tol
        break;
    end
    
    % Determine the new interval
    if f(a) * fc < 0
        b = c;
    else
        a = c;
    end
end

% Display the result
fprintf('The root is approximately: %.6f\n', c);
fprintf('Number of iterations: %d\n', i);

%% IK of 2R robot using newton raphson 

% Define symbolic variables
syms theta1 theta2 l1 l2 x y real

% Forward kinematics equations
x_eq = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
y_eq = l1 * sin(theta1) + l2 * sin(theta1 + theta2);

% Desired end-effector position
x_d = 1.0; % Example value
y_d = 1.0; % Example value
l1_val = 1.0; % Length of the first link
l2_val = 1.0; % Length of the second link

% Initial guess for joint angles
theta1_0 = 0.5;
theta2_0 = 0.5;

% Tolerance for stopping criterion
tol = 1e-6;

% Maximum number of iterations
max_iter = 100;

% Newton-Raphson iteration
theta = [theta1_0; theta2_0];
for i = 1:max_iter
    % Evaluate the current end-effector position
    x_val = double(subs(x_eq, [theta1, theta2, l1, l2], [theta(1), theta(2), l1_val, l2_val]));
    y_val = double(subs(y_eq, [theta1, theta2, l1, l2], [theta(1), theta(2), l1_val, l2_val]));

    % Compute the error vector
    F = [x_val - x_d; y_val - y_d];
    
    % Compute the Jacobian matrix
    J = jacobian([x_eq, y_eq], [theta1, theta2]);
    J_val = double(subs(J, [theta1, theta2, l1, l2], [theta(1), theta(2), l1_val, l2_val]));
    
    % Update the joint angles
    delta_theta = J_val \ -F;
    theta = theta + delta_theta;

    % Check for convergence
    if norm(delta_theta) < tol
        break;
    end
end

% Display the result
theta1_sol = theta(1);
theta2_sol = theta(2);
fprintf('The solutions are: theta1 = %.6f, theta2 = %.6f\n', theta1_sol, theta2_sol);
fprintf('Number of iterations: %d\n', i);

%% 

% Define symbolic variables
syms theta1 theta2 l1 l2 x y real

% Forward kinematics equations
x_eq = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
y_eq = l1 * sin(theta1) + l2 * sin(theta1 + theta2);

% Define the equations as a system
eq1 = x_eq == x;
eq2 = y_eq == y;

% Solve for theta1 and theta2 symbolically
solutions = solve([eq1, eq2], [theta1, theta2]);

% Display the solutions
disp('Solutions for theta1:');
disp(solutions.theta1);
disp('Solutions for theta2:');
disp(solutions.theta2);


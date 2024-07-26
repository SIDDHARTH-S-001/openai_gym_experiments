%% Velocity Kinematics 3R robot

% Define symbolic variables
syms theta1 theta2 theta3 l1 l2 l3 real

% DH parameters
% Example DH table [a, alpha, d, theta]
% Modify the following DH table as needed for your specific robot
DH_table = [ 0     0     0    theta1;
             l1    0     0    theta2;
             l2    0     0    theta3;
             l3    0     0    0];

% Number of rows in the DH table
num_rows = size(DH_table, 1);

% Initialize a cell array to store the transformation matrices
% Its an array of cells and cells can contain data of any type
T_matrices = cell(num_rows, 1);

% Function to compute the transformation matrix from DH parameters
T_DH = @(a, alpha, d, theta) [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                               0,           sin(alpha),             cos(alpha),            d;
                               0,           0,                      0,                     1 ];

% Loop through each row of the DH table and compute the transformation matrix
for i = 1:num_rows
    a_i = DH_table(i, 1);
    alpha_i = DH_table(i, 2);
    d_i = DH_table(i, 3);
    theta_i = DH_table(i, 4);
    
    % Compute the transformation matrix for the current row
    T_matrices{i} = T_DH(a_i, alpha_i, d_i, theta_i);
    
    % Display the transformation matrix
    % fprintf('Transformation Matrix for row %d:\n', i);
    % disp(T_matrices{i});
end

T = T_matrices{1}*T_matrices{2}*T_matrices{3}*T_matrices{4};

P = T(1:3, 4); % Position of End-Effector

J = jacobian(P, [theta1, theta2, theta3]); % Jacobian Matrix from the base frame

% The Jacobian matrix relates joint velocities to end-effector velocities. Its frame of reference determines how these velocities are expressed. 
% There are primarily two types of Jacobians:
% a) Geometric Jacobian (J_G): Expresses end-effector velocity in the base frame.
% b) Analytical Jacobian (J_A): Expresses end-effector velocity in the end-effector frame.

% To change the Jacobian's frame of reference, you multiply it by a rotation matrix:
% J_new = R * J_old, Where R is the rotation matrix from the old frame to the new frame.

% Changing the Jacobian's frame of reference affects:
% a) Interpretation of results: Velocities will be expressed relative to different coordinate systems.
% b) Singularity analysis: Singularities remain the same, but their interpretation changes.
% c) Force/torque calculations: The dual of velocity kinematics, important for control and dynamics.
% d) Computational efficiency: Some frames might lead to simpler expressions.
% Base frame (world coordinates): Useful for global path planning and obstacle avoidance.
% End-effector frame: Beneficial for tasks defined relative to the end-effector, like tool use.






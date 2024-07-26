%% Velocity Kinematics 3R robot

% Define symbolic variables
syms theta1 theta2 theta3 theta1_dot theta2_dot theta3_dot l1 l2 l3 real

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

J = jacobian(P, [theta1, theta2, theta3]); % Jacobian Matrix 

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

J_base = J; % Jacobian Matrix expressed in base frame

R_base_to_ee = T(1:3, 1:3);

J_ee = R_base_to_ee * J_base; % Jacobian Matrix expressed in end-effector frame

theta_dot = [theta1_dot, theta2_dot, theta3_dot]';

v_base = J_base * theta_dot; % end-effector's velocity as seen from the base (or world) frame.
v_ee = J_ee * theta_dot; % end-effector's velocity as seen from the end-effector's own perspective.

% In a serial manipulator, angular velocity propagates from the base to the end-effector, with each joint contributing to the overall angular velocity of the subsequent links. 
% This progression follows a recursive pattern. ω_i = ω_(i-1) + θ̇_i * z_i

z1 = [0 0 1];
z2 = [0 0 1];
z3 = [0 0 1];

w0 = [0 0 0]; % Angular Velocity of base link
w1 = w0 + theta_dot(1)*z1; % Angular Velocity of link 1
w2 = w1 + theta_dot(2)*z2; % Angular Velocity of link 2
w3 = w2 + theta_dot(3)*z3; % Angular Velocity of link 3

% The angular velocity propagation directly affects the rotational part of the Jacobian matrix.









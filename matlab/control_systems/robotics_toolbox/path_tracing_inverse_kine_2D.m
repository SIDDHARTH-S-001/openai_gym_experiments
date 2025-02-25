% This script is to calculate inverse kinematics for a simple 2-D manipulator using the inverseKinematics class.
% Manipulator is a simple 2 DOF planar robot with both revolute joints
% A circular trajectory is created in a 2-D plane and given as points to the inverse kinematics solver.
% The solver calculates the required joint positions to achieve this trajectory. 

% Construct the robot
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

% Specify arm lengths
L1 = 0.3;
L2 = 0.3;

% Add link1 with joint1
body = rigidBody('link1'); % create link as a body
joint = rigidBodyJoint('joint1', 'revolute'); % create joint for the link (here it's revolute)
setFixedTransform(joint,trvec2tform([0 0 0])); % set the transformation for the link (this function sets Transformation b/w child and parent through the joint)
joint.JointAxis = [0 0 1]; % assign the axis of rotation for the joint 
body.Joint = joint; % assign the joint to the body (link1)
addBody(robot, body, 'base'); % add the body (link1) to the rigidBodyTree

% Add link2 with joint2
body = rigidBody('link2');
joint = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint, trvec2tform([L1, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link1') % required params are -> rigidBodyTree, body, parent

% Add end-effector with fixed joint
body = rigidBody('tool');
joint = rigidBodyJoint('fix1', 'fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

showdetails(robot)

% Define the Trajectory (Circle to be traced over in 10 secs)
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

% Inverse Kinematics Solution
% pre-allocate config solutions as a matrix qs
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

% Create Kinematic solver
ik = inverseKinematics('RigidBodyTree', robot);
% since only the xy Cartesian points are the only important factors of the end-effector pose for this workflow, 
% specify a non-zero weight for the fourth and fifth elements of the weight vector. All other elements are set to zero.
weights = [0, 0, 0, 1, 1, 0];
end_effector = 'tool';

% Loop through the trajectory of points to trace the circle.
qInit = q0; % use home config as initial guess
for i = 1:count
    % solve for config satisfying end-effector position
    point = points(i, :);
    % Call the ik object for each point to generate the joint configuration that achieves the end-effector position. 
    qsol = ik(end_effector, trvec2tform(point), weights, qInit);
    % Store the configuration
    qs(i, :) = qsol;
    % Start from prior solution
    qInit = qsol;
end

% Animate the solution
figure
show(robot, qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

% Set up a rateControl object to display the robot trajectory at a fixed rate of 15 frames per second. 
framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end



















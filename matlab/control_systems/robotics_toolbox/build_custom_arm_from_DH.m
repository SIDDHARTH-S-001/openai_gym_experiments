% This script builds a custom robotic arm from its corresponding DH Table
% order of parameters (column) of DH table -> a, alpha, d, theta
% naturally theta is ignored for initial configuration
dhParams = [0    0 0 0;
            0.46 0 0 0;
            0.38 0 0 0];

% create a rigid body tree
robot = rigidBodyTree;

% Create a cell array for the rigid body object, and another for the joint objects
bodies = cell(3,1);
joints = cell(3,1);
for i = 1:3
    % Create a rigidBody object with a unique name.
    bodies{i} = rigidBody(['body' num2str(i)]);
    % Create and name a revolute rigidBodyJoint object.
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    % Use setFixedTransform to specify the body-to-body transformation of the joint using DH parameters. 
    setFixedTransform(joints{i},dhParams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    % Attach each body to the tree
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

showdetails(robot)

figure(Name="3R Robot Interactive GUI")
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);
% This script builds a robotic arm from its corresponding DH Table
% Puma560 robot is taken as example

% DH Table for the Puma560 Robot
dhparams = [0   	pi/2	0   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0];

% Create a RigidBodyTree
robot = rigidBodyTree;

% Create a cell array for the rigid body object, and another for the joint objects. 
bodies = cell(6,1);
joints = cell(6,1);
% Iterate through the DH parameters performing this process.
for i = 1:6
    % Create a rigidBody object with a unique name.
    bodies{i} = rigidBody(['body' num2str(i)]);
    % Create and name a revolute rigidBodyJoint object.
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    % Use setFixedTransform to specify the body-to-body transformation of the joint using DH parameters. 
    % The function ignores the final element of the DH parameters (theta) because the angle of the body is dependent on the joint position.
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    % Use addBody to attach the body to the rigid body tree.
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

showdetails(robot)

% figure(Name="PUMA Robot Model")
% show(robot);

% Visualize the robot model to confirm its dimensions by using the interactiveRigidBodyTree object.
figure(Name="Interactive GUI")
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);
% The GUI uses inverse kinematics to solve for the joint positions that achieve the best possible match to the specified end-effector position
% Right-click a specific body frame to set it as the target marker body, or to change the control method for setting specific joint positions.
































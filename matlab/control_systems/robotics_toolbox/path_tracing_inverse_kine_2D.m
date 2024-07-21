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




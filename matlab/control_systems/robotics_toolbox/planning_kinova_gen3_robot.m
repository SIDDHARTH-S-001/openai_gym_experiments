%% This script is for planning, task execution and trajectory using Kinova Gen3 robotic arm

% Load the rigid body tree of the Kinova Gen3 Robot
robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);

% set configuration
currentRobotJConfig = homeConfiguration(robot);

% get the no of joints and the eef frame
numJoints = numel(currentRobotJConfig);
endEffector = "EndEffector_Link";

% set trajectory time and approx tool speed
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

% set initial and final pose of eef
jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);
taskFinal = trvec2tform([0.4,0,0.6])*axang2tform([0 1 0 pi]);

% Compute task-space trajectory waypoints via interpolation.
% 1) compute tool travelling distance
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

% 2) define traj times based on travelling distance and tool speed
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

% 3) Interpolate between taskInit and taskFinal to compute intermediate task-space waypoints.
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 

% Create a task space motion model for PD control on the joints.
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','EndEffector_Link');

% Set the proportional and derivative gains on orientation to zero, so that controlled behavior just follows the reference positions
tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;

% Define the initial states (joint positions and velocities)
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

% Use ode15s to simulate the robot motion
[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);

% Generate Joint space trajectory
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

% Calculate the initial and desired joint configurations using inverse kinematics.
initialGuess = jointInit;
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);

% By default, the IK solution respects joint limits. However, for continuous joints (revolute joints with infinite range), 
% the resultant values may be unnecessarily large and can be wrapped to [-pi, pi] to ensure that the final trajectory covers a minimal distance
wrappedJointFinal = wrapToPi(jointFinal);

% Interpolate between them using a cubic polynomial function to generate an array of evenly-spaced joint configurations.
ctrlpoints = [jointInit',wrappedJointFinal']; % notice !!! the transpose of these matrices are used
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);

% Control Joint space trajectory
jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot,'MotionType','PDControl');

% Set initial states
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

% Use ode15s to simulate the robot motion. 
[tJoint,stateJoint] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; qd0]);

% Show robot's initial config
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

% Viz the task space trajectory
for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
    drawnow;
end

% Viz the joint space trajectory
% Return to initial configuration
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tJoint,stateJoint(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    drawnow;
end

% Add a legend and title
legend([taskSpaceMarker jointSpaceMarker], {'Defined in Task-Space', 'Defined in Joint-Space'});
title('Manipulator Trajectories')


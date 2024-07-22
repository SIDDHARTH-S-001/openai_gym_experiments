%% Create a custom arm out of DH table
% order of parameters (column) of DH table -> a, alpha, d, theta
% naturally theta is ignored for initial configuration
l1 = 0.46;
l2 = 0.38;
dhParams = [0  0 0 0;
            l1 0 0 0;
            l2 0 0 0];

% create a rigid body tree
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

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

% figure(Name="3R Robot Interactive GUI")
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);

% Add a text object for displaying coordinates and joint angles
coordText = text(0, 0, 0, '', 'FontSize', 12, 'BackgroundColor', 'w', 'Margin', 2);

% Create and start a timer to periodically update the display
updateTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                    'TimerFcn', @(~,~) updateDisplay(robot, gui, coordText));
start(updateTimer);

%% Kinematics

t = (0:0.2:10)'; % Time
count = length(t);
center = [0.5 0.1 0];
radius = 0.2;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

% Pre-allocate configuration solutions as a matrix qs.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

% Because the xy Cartesian points are the only important factors of the end-effector pose for this workflow, 
% specify a non-zero weight for the fourth and fifth elements of the weight vector. 
% All other elements are set to zero.
ik = inverseKinematics('RigidBodyTree', robot);
% Weight for pose tolerances, specified as a six-element vector. 
% The first three elements correspond to the weights on the error in orientation for the desired pose. 
% The last three elements correspond to the weights on the error in xyz position for the desired pose.
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'body3';

% Loop through the trajectory of points to trace the circle
qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end



%% Function definition
function updateDisplay(robot, gui, coordText)
    % Get the current configuration from the GUI
    config = gui.Configuration;
    
    % Calculate forward kinematics to get the end-effector position
    eeTform = getTransform(robot, config, 'body3');
    eePos = tform2trvec(eeTform);

    % Update the text display with the coordinates and joint angles
    angles = rad2deg(config);
    textString = sprintf('Position: [%.2f, %.2f, %.2f]\nJoint Angles: [%.2f, %.2f, %.2f]', eePos, angles);
    coordText.Position = eePos + [0 0 0.1]; % Slightly offset the text position
    coordText.String = textString;
end












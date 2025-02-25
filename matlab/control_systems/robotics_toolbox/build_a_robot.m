body1 = rigidBody('body1');
% Use the trvec2tform function to convert from a translation vector to a homogenous transformation
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = pi/4;
tform = trvec2tform([0.25, 0.25, 0]);
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

% Create a rigid body tree and attach the 1st body to it
robot = rigidBodyTree;
addBody(robot,body1,'base')

% Create 2nd body
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = pi/6; % User defined
tform2 = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

% Creating bodies 3 and 4, also attaching them to body 2
body3 = rigidBody('body3');
body4 = rigidBody('body4');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt4 = rigidBodyJoint('jnt4','revolute'); 
tform3 = trvec2tform([0.6, -0.1, 0])*eul2tform([-pi/2, 0, 0]); % User defined
tform4 = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(jnt3,tform3);
setFixedTransform(jnt4,tform4);
jnt3.HomePosition = pi/4; % User defined
body3.Joint = jnt3;
body4.Joint = jnt4;
addBody(robot,body3,'body2'); % Add body3 to body2
addBody(robot,body4,'body2'); % Add body4 to body2

% Define end-effector as a rigid body with a fixed joint
bodyEndEffector = rigidBody('endeffector');
tform5 = trvec2tform([0.5, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform5);
addBody(robot,bodyEndEffector,'body4');

config = randomConfiguration(robot) % sets the robot (rigid body tree) to a random configuration
tform = getTransform(robot,config,'endeffector','base') % gets transformation b/w two body frames
show(robot)



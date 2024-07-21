%% Mathematical Model of DC motor
% syms K theta_dot V Ia J L R b s
% Input -> V
% Output -> theta_dot (angular velocity of shaft)
% Magnetic field is assumed to be constant, so torque will be proportional to armature current Ia.
% T = Kt * Ia;
% back emf (e) proportional to angular velocity of shaft (theta_dot) 
% e = Kv * theta_dot;
% In SI Units, Kt = Kv, so its taken as K uniformly.

% Transfer function model (theta_dot / V) in laplace domain
% TF = K /[(J*s + b)*(L*s + R) + (K*K)];

%% Example

J = 0.01; % moment of inertia of combination (shaft + damping + load)
b = 0.1;  % viscous damping
K = 0.01; % back emf constant
R = 1;    % armature resistance
L = 0.5;  % armature inductance
s = tf('s'); % creating s as a TF variable
TF = K/((J*s+b)*(L*s+R)+K^2); % open loop TF

% State Space representation
A = [-b/J   K/J
    -K/L   -R/L];
B = [0
    1/L];
C = [1   0];
D = 0;
motor_ss = ss(A,B,C,D);

% conversion from TF to SS can also be done as below
SS = ss(TF);

%% Analysis
% For a 1-rad/sec step reference, the design criteria are the following.
% Settling time less than 2 seconds
% Overshoot less than 5%
% Steady-state error less than 1%

linearSystemAnalyzer('step', TF, 0:0.1:10) % specify that the step response plot should include data points for times from 0 to 5 seconds in steps of 0.1 seconds

% system has one pole at s = -10 and another at s= -2
% Since the one pole is 5 times more negative than the other, the slower of the two poles will dominate the dynamics. 
% That is, the pole at s = -2 primarily determines the speed of response of the system and the system behaves similarly to a first-order system
% by slower pole, we mean the pole that decays slower (less -ve real part). This is called a dominant pole.

%% Now modelling the system as an approx 1st order system, 
rP_motor = 0.1 / (0.5*s + 1);

% plot both, the original system and the approximated 1st order system, 
linearSystemAnalyzer('step', TF, rP_motor, 0:0.1:10)

% observations:
% rise time = 1.1s for 1st order approx, 1.14 for original system
% settling time = 1.96 for 1st order approx, 2.07 for original system
% A first-order approximation of our motor system is relatively accurate. 
% The primary difference can be seen at t = 0 where a second order system will have a derivative of zero, but our first-order model will not.
% With a first-order system, the settling time is equal to 4*Tau, where Tau is the time constant which in this case is approx 0.5

% use lsim command or Linear simulation plot in linearSystemAnalyzer to study system response to various inputs.
lsim(SS, rP_motor)

%% PID Control
% For a 1-rad/sec step reference, the design criteria are the following. 
% Settling time less than 2 seconds
% Overshoot less than 5%
% Steady-state error less than 1%

Kp = 100;
C = pid(Kp);
sys_cl = feedback(C*TF, 1); % feedback(sys1,sys2) returns a model object sys for the negative feedback interconnection of model objects sys1,sys2.


% analyzing step response of closed loop system
t = 0:0.1:5;
step(sys_cl, t)
grid
title('step response with proportional control')

% observations
% rise time = 0.13s
% settling time = 0.57s
% max peak overshoot = 21.9% (Too High)
% steady state value = 0.909 (approx 0.1 Ess -> Too High)

% Note:
% Increasing Kp reducing Ess but increases overshoot
controlSystemDesigner(TF) % create new step response plot and select PID tuning from the tuning methods menu for compensator tuning

%% PID Control Continuation
% Adding an integral term will eliminate the steady-state error to a step reference and a derivative term will often reduce the overshoot. 

Kp = 100;
Ki = 200;
Kd = 10;
C = pid(Kp, Ki, Kd);
sys_cl = feedback(C*TF, 1);
t = 0:0.1:200;
step(sys_cl, t);
title('PID Control with Small Ki and Small Kd')

% observations: (Kp:75, Ki:1 and Kd:1)
% rise time = 0.172s
% settling time = 150s (way too high !!!)
% steady state value = 1 (Ess = 0)
% peak amplitude = 0.998 (approx 0% overshoot)

% observations: (Kp:100, Ki:100 and Kd:1)
% rise time = 0.129s
% settling time = 0.586s 
% steady state value = 1 (Ess = 0)
% peak overshoot = 19.4% (Too high)

% observations: (Kp:100, Ki:100 and Kd:10) -> Best set of parameters for the design requirement
% rise time = 0.144s
% settling time = 0.269s 
% steady state value = 1 (Ess = 0)
% peak overshoot = 1.03% (fits requirement)

% Inference
% Increasing Kp reduces steady state error
% Increasing Ki improves response and reduces settling time
% Increasing Kd reduces overshoot

%% Root Locus
% Requirements
% For a 1-rad/sec step reference, the design criteria are the following.
% 
% Settling time less than 2 seconds
% Overshoot less than 5%
% Steady-state error less than 1%

controlSystemDesigner('rlocus', TF)

% desired regions can be added to the root locus plot by right-clicking on the plot and choosing Design Requirements > New
% after setting the req for settling time and peak overshoot, The resulting desired region for the closed-loop poles is shown by the unshaded region of the above figure. 
% More specifically, the two rays centered at the origin represent the overshoot requirement; the smaller the angle these rays make with the negative real-axis, the less overshoot is allowed. The vertical line at s = -2 represents the settling time requirement, 
% where the farther to left the closed-loop poles are located the smaller the settling time is.

% since the closed-loop system has two poles with no zeros, placing the closed-loop poles in the shown region will guarantee satisfaction of our transient response requirements.

% observations for this case, poles at -6 +-2i
% gain = 10 (approx)
% overshoot = 0%
% settling time = 0.83s
% steady state value = 0.5
% although it satisfies requirement, the steady state value is not 1 yet

% Note: Increasing system gain by moving poles vertically upward will lead to large overshoot, 
% so a way around is to use a lag compensator










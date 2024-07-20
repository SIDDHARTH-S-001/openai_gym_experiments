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
TF = K/((J*s+b)*(L*s+R)+K^2) 

% State Space representation
A = [-b/J   K/J
    -K/L   -R/L];
B = [0
    1/L];
C = [1   0];
D = 0;
motor_ss = ss(A,B,C,D)

% conversion from TF to SS can also be done as below
SS = ss(TF)

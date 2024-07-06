%% Resources
% https://www.youtube.com/playlist?list=PLUMWjy5jgHK3-ca6GP6PL0AgcNGHqn33f (Brian Douglas Playlist on Root Locus)
% https://www.tutorialspoint.com/control_systems/control_systems_construction_root_locus.htm
%% Basic Root Locus Plot
num = [2 5 1];
den = [1 2 3];
sys = tf(num, den)

rlocus(sys) % This function plots the root locus of the system
[r, k] = rlocus(sys) % This function extract the closed-loop poles and associated feedback gain values 

%% Closed-Loop Pole Locations for a Set of Feedback Gain Values
num1 = [0.5 0 -1];
den1 = [4 0 3  0 2];
sys1 = tf(num1, den1);

% considering a range of values for feedback gain K
k = (1:0.5:5); % a values between 1 and 5 with increments of 0.5
r = rlocus(sys1, k);
size(r) % there are 4 closed loop poles and 9 specific gain values (column)
% even trajectory of r-locus for specific gain values can be visualized
rlocus(sys1, k)
%% SISO Tool based visualization
num2 = [1 2];
den2 = [1 4 5 6 3];
sys2 = tf(num2, den2);





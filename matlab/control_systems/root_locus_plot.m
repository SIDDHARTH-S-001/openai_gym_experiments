%% Basic Root Locus Plot
num = [2 5 1];
den = [1 2 3];
sys = tf(num, den)

rlocus(sys) % This function plots the root locus of the system
[r, k] = rlocus(sys) % This function extract the closed-loop poles and associated feedback gain values 

%% Closed-Loop Pole Locations for a Set of Feedback Gain Values


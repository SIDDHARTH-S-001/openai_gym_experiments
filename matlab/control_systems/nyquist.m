%% This script consists of all crazy experiments with nyquist plot and the nyquist stability criterion

% sys = tf([1], [1 2])
z = [-1, 1]; % list of zeros of the system
p = [0, 1, -2]; % list of poles of the system
k = 1; % gain
sys = zpk(z, p, k)

% rlocus(sys)
% bode(sys)
% nyquist(sys)

subplot(3, 1, 1)
rlocus(sys)
subplot(3, 1, 2)
bode(sys)
subplot(3, 1, 3)
nyquist(sys)
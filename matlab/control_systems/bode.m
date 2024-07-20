z = [0]; % list of zeros of the system
p = [0, 1, 2]; % list of poles of the system
k = 1; % gain
sys = zpk(z, p, k)

bode(sys)
% The plot displays the magnitude (in dB) and phase (in degrees) of the system response as a function of frequency. 
% Bode automatically determines frequencies to plot based on system dynamics.


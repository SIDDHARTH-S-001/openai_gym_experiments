%% This script is to explore transient response analysis - Ogata Chapter 5
% Q1)
A = [-1 -1;
     6.5 0];

B = [1 1;
     1 0];

C = [1 0;
     0 1];

D = [0 0;
     0 0];

step(A, B, C, D)

%% Q2
% In this program we plot step-response curves of a system
% having two inputs (u1 and u2) and two outputs (y1 and y2)
% We shall first plot step-response curves when the input is u1
% Then we shall plot step-response curves when the input is u2 
% Enter matrices A, B, C, and D
A = [-1 -1;6.5 0];
B = [1 1;1 0];
C = [1 0;0 1];
D = [0 0;0 0];
% ***** To plot step-response curves when the input is u1, enter
% the command 'step(A,B,C,D,1)' *****
step(A,B,C,D,1)
grid
title ('Step-Response Plots: Input = u1 (u2 = 0)')
text(3.4, -0.06,'Y1')
text(3.4, 1.4,'Y2')
hold on
% ***** Next, we shall plot step-response curves when the input
% is u2. Enter the command 'step(A,B,C,D,2)' *****
step(A,B,C,D,2)
grid
title ('Step-Response Plots: Input = u2 (u1 = 0)')
text(3,0.14,'Y1')
text(2.8,1.1,'Y2')

%% Standard form of 2nd order system
% Q3
wn = 5; % natural frequency
zeta = .4; % damping ratio
[num, den] = ord2(wn, zeta); % generates a standard continuous 2nd order LTI model
num = 5^2 * num;
printsys(num, den, 's');

t = 0:0.01:3;

% Obtaining step response
% step(num, den);
[y,x,t] = step(num, den, t);
plot(t, y);
grid
title (' Unit-Step Response of G(s) = 25/(s^2+4s+25)')
xlabel('t - sec')
ylabel('Output')

%% Q4 - 3D plot of unit step response
t = linspace(0, 10, 51);
zeta = [0 0.2 0.4 0.6 0.8 1];

y = zeros(length(t), length(zeta)); % Preallocate y with correct dimensions

for n = 1:length(zeta)
    num = [1];
    den = [1 2*zeta(n) 1];
    y(:, n) = step(num, den, t); % Collect step response
end

length(t)
length(y)

% Use plot function for 2D plot
figure;
plot(t, y);
grid on;
title('2D plot of u(t) response');
xlabel('t (sec)')
ylabel('Response')
text(4.1,1.86,'\zeta = 0')
text(3.5,1.5,'0.2')
text(3.5,1.24,'0.4')
text(3.5,1.08,'0.6')
text(3.5,0.95,'0.8')
text(3.5,0.86,'1.0')

% Use mesh function for 3D plot
figure;
mesh(t, zeta, y'); 
title('3D plot of u(t) response');
xlabel('t - sec')
ylabel('\zeta')
zlabel('response')

%% Q5 - Getting system response parameters
num = 25;
den = [1 6 25];
t = 0:0.005:5

[y,x,t] = step(num, den, t);
r = 1; 

while y(r) < 1.0001
    r = r+1;
end

rise_time = (r-1)*0.005

[ymax, tp] = max(y);
peak_time = (tp-1)*0.005

max_overshoot = ymax - 1

s = 1001;
while y(s) > 0.98 && y(s) < 1.02
    s = s-1;
end

settling_time = (s-1)*0.005





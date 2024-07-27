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

%% Q6 - Unit Impulse Response
num = 1;
den = [1 0.2 1];
impulse(num, den);
grid
title('Unit Impulse response')

% when initial conditions are zero, unit impulse response of G(s) is same
% as unit step response of sG(s), this can be done by increasing order of
% numerator to 1 by multiplying by s and running unit step response.

num1 = [0 1 0];
den1 = [1 0.2 1];
step(num1, den1);
grid
title('Unit step response of sG(s)')

%% Q7 - Obtain ramp response
% Matlab doesnt have unit ramp response, so divide G(s) by s and use step reponse.

num = [2 1];
den = [1 1 1];

% dividing system TF by s, now system is G(s)/s
num1 = [2 1];
den1 = [1 1 1 0];
t = 0:0.1:10;

% step(num1, den1); % step response of G(s)/s is same as ramp response of G(s)
c = step(num1, den1, t);
% In plotting the ramp-response curve, add the reference
% input to the plot. The reference input is t. Add to the
% argument of the plot command with the following: t,t,'-'. Thus
% the plot command becomes as follows: plot(t,c,'o',t,t,'-')

plot(t,c,'o',t,t,'-')

grid
title('Unit ramp response of G(s)')
xlabel('t Sec')
ylabel('Input and Output')

%% Q8 - Ramp response of a state space model
% The unit-ramp response is obtained by adding a new state variable x3. 
% The dimension of the state equation is enlarged by one.

A = [ 0  1;
     -1 -1];
B = [0;
     1];
C = [1 0];
D = 0;

% Enter matrices AA, BB, CC, and DD of the new, enlarged state equation and output equation
AA = [A zeros(2,1);C 0];
BB = [B;0];
CC = [0 0 1];
DD = 0;

step(AA, BB, CC, DD);
% [z, x, t] = step(AA, BB, CC, DD);
x3 = [0 0 1]*x'; plot(t,x3,'o',t,t,'-')

grid
title('Unit-Ramp Response')
xlabel('t Sec')
ylabel('Input and Output')

%% Q9 - Response to an arbitrary input
% function lsim can be used to get response of any arbitrary input.

num = [2 1];
den = [1 1 1];
t = 0:0.1:10;
r = t; % defining input as a function of time (here its same as ramp function)
y = lsim(num,den,r,t);
plot(t,r,'-',t,y,'o')
grid
title('Unit-Ramp Response Obtained by Use of Command "lsim"')
xlabel('t Sec')
ylabel('Unit-Ramp Input and System Output')
text(6.3,4.6,'Unit-Ramp Input')
text(4.75,9.0,'Output')










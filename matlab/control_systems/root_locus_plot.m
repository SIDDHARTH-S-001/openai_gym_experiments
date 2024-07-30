%% Resources
% https://www.youtube.com/playlist?list=PLUMWjy5jgHK3-ca6GP6PL0AgcNGHqn33f (Brian Douglas Playlist on Root Locus)
% https://www.tutorialspoint.com/control_systems/control_systems_construction_root_locus.htm

% This script contains root locus and compensators
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
%% Worked out example (manual and then software generated plot)
p = [0, -1, -5];
gain = 1;
sys3 = zpk([], p, gain);

% open loop poles at 0, -1 and -5 and doesnt have any open loop zeros,
% hence P > Z. and since P = 3, no of root locus branches = 3
% centroid of asymptotes is at -2
% angle of asymptotes are 60, 180 and 300 degrees, because of which 2 root locus branches will meet jw axis
% By using the Routh array method and special case(ii), the root locus branches intersects the imaginary axis at -j√5  and +j√5
% Break away point on s = -0.473

rlocus(sys3)

%% Q1 - Ogata Example 6-3
a = [1 1 0];
b = [1 4 16];
c = conv(a, b);
r = roots(b);

num = [1 3];
den = [1 5 20 16 0];
sys = tf(num, den);
rlocus(sys)
% region of root locus needs to be -> -6<=x<=+6 and -6<=y<=+6
v = [-6 6 -6 6];
axis(v);
axis('square');
% With this command, the region of the plot is as specified and a line with slope 1 is at a true 45°,
% not skewed by the irregular shape of the screen

%% Q2 - Ogata Example 6-4
a = [0 1 0];
b = [0 1 0.5];
c = [1 0.6 10];
d = conv(a, b);
e = conv(d, c);
num = 1;
den = [1 1.1 10.3 5 0];
sys = tf(num, den);
rlocus(sys)

%% Plotting Q2 with user defined gain
num = 1;
den = [1 1.1 10.3 5 0];
K1 = 0:0.2:20;
K2 = 20:0.1:30;
K3 = 30:5:1000;
K = [K1 K2 K3];
r = rlocus(num,den,K);
plot(r, 'o')
v = [-4 4 -4 4]; axis(v)
grid
title('Root-Locus Plot of G(s) = K/[s(s + 0.5)(s^2 + 0.6s + 10)]')
xlabel('Real Axis')
ylabel('Imag Axis')

%% Q3 - Ogata Example 6-5
A = [0    1  0;
     0    0  1;
    -160 -56 -14];

B = [0 ; 1 ; -14];
C = [1 0 0];
D = 0;
sys = ss(A, B, C, D);
[num, den] = ss2tf(A, B, C, D);
r = roots(den); % 3 roots to den => 3 open loop poles => 3 root locus branches 
rlocus(sys)

%% Plotting Polar Grids in the Root-Locus Diagram (sgrid - command)
sgrid
v = [-3 3 -3 3];
axis(v);
axis('square')
title('Constant \zeta Lines and Constant \omega_n Circles')
xlabel('Real Axis')
ylabel('Imag Axis')

%% Example
num = [1];
den = [1 4 5 0];
K = 0:0.01:1000;
r = rlocus(num,den,K);
plot(r,'-'); v = [-3 1 -2 2]; axis(v); axis('square')
sgrid([0.5,0.707], [0.5,1,2])
grid
title('Root-Locus Plot with \zeta = 0.5 and 0.707 Lines and \omega_n = 0.5,1, and 2 Circles')
xlabel('Real Axis'); ylabel('Imag Axis')
gtext('\omega_n = 2')
gtext('\omega_n = 1')
gtext('\omega_n = 0.5')
% Place 'x' mark at each of 3 open-loop poles.
gtext('x')
gtext('x')
gtext('x')

%% Q3 - Ogata matlab program 6-7 
num = [1 2 4];
den = conv(conv([1 4 0], [1 6]), [1 1.4 1]);
rlocus(num, den)
% this system is stable only for limited ranges of the value of K—that is, 0<K<12 and 73<K<154. 
% The system becomes unstable for 12<K<73 and 154<K. Such a system is Conditionally stable.
% Addition of proper compensation system will eliminate conditional stability.

%% Non-minimum phase system - Ogata Matlab program 6-8
% If all the poles and zeros of a system lie in the lefthalf s plane, then the system is called minimum phase. 
% If a system has at least one pole or zero in the right-half s plane, then the system is called nonminimum phase
num = [-0.5 1];
den = [1 1 0];
k1 = 0:0.01:30;
k2 = 30:1:100;
k3 = 100:5:500;
K = [k1 k2 k3];
rlocus(num,den,K)
v = [-2 6 -4 4]; axis(v); axis('square')
grid
title('Root-Locus Plot of G(s) = K(1 - 0.5s)/[s(s + 1)]')
% Place 'x' mark at each of 2 open-loop poles.
% Place 'o' mark at open-loop zero.
gtext('x')
gtext('x')
gtext('o')

%% Q4 - Example 6-6 (Lead Compensator)
num = 10;
denc = [1 1 10]; % denominator of closed loop TF -> wn = 10^(0.5) = 3.16 rads/s
% and zeta = 0.1581 (obtained from standard form of 2nd order system).
% since zeta is small, system will have large overshoot
deno = [1 1 0]; % denominator of open loop TF -> poles at origin and -1
sys_open = tf(num, deno);
r = roots(denc)
rlocus(num, deno) % remember root locus is drawn for open loop system

% Goal is design a lead compensator to ensure zeta = 0.5 and wn = 3 rads/s
zeta = 0.5; % desired damping ratio
wn = 3; % desired undamped natural frequency
% now the new system becomes
denn = [1 2*zeta*wn wn*wn];
rd = roots(denn) % poles of the desired TF -> dominant poles at -1.5 +- 2.59811

% Designing steps: (Follow Ogata book - method 1 and 2)
% Find the sum of the angles at the desired location of one of the dominant closed-loop poles with the 
% open-loop poles and zeros of the original system, and determine the necessary angle to be added so that
% the total sum of the angles is equal to odd multiple of 180 degrees. The lead compensator must contribute this
% angle. (If the angle to be compensated is quite large, then two or more lead networks may be needed rather than
% a single one.)
% In this case the deficit angle is 40.894 degrees.

% Following Method 1: Zero at -1.9432 and pole at -4.6458 (lead compensator)
% alpha = 1.9432/4.6458 = 0.418, Kc = 1.2287 (obtained from magnitude condition)
Kc = 1.2287;
num_comp = [1 1.9432];
den_comp = [1 4.6458];
sys_comp = tf(Kc*num_comp, den_comp);
sys_compensated = sys_comp*sys_open;
rlocus(sys_compensated)
Kv = 5.139; % static velocity constant (higher values is desired as it produces less Ess - steady state error)

% Following Method 2: Placing zero at -1 so that it cancels out open loop pole at -1 
% in that case, pole of compensator must be located at -3.
Kc = 0.9;
num_comp = [1 1];
den_comp = [1 3];
sys_comp = tf(Kc*num_comp, den_comp);
sys_compensated = sys_comp*sys_open;
rlocus(sys_compensated)
Kc = 3; % lesser compared to Method 1.

%% Comparison of step response of compensated and uncompensated system (Ogata Matlab Program 6-9)
% Compensator designed with method 1:
num1 = [12.287 23.876];
den1 = [1 5.646 16.933 23.876];
% Compensator designed with method 2:
num2 = 9;
den2 = [1 3 9];
% Uncompensated system
num = 10;
den = [1 1 10];
t = 0:0.05:5;
c1 = step(num1,den1,t);
c2 = step(num2,den2,t);
c = step(num,den,t);
plot(t,c1,'-',t,c2,'.',t,c,'x')
grid
title('Unit-Step Responses of Compensated Systems and Uncompensated System')
xlabel('t Sec')
ylabel('Outputs c1, c2, and c')
text(1.51,1.48,'Compensated System (Method 1)')
text(0.9,0.48,'Compensated System (Method 2)')
text(2.51,0.67,'Uncompensated System')

%% Comparison of ramp response of compensated and uncompensated system (Ogata Matlab Program 6-9)
num1 = [12.287 23.876];
den1 = [1 5.646 16.933 23.876 0];
num2 = 9;
den2 = [1 3 9 0];
t = 0:0.05:5;
c1 = step(num1,den1,t);
c2 = step(num2,den2,t);
plot(t,c1,'-',t,c2,'.',t,t,'-')
grid
title('Unit-Ramp Responses of Compensated Systems')
xlabel('t Sec')
ylabel('Unit-Ramp Input and Outputs c1 and c2')
text(2.55,3.8,'Input')
text(0.55,2.8,'Compensated System (Method 1)')
text(2.35,1.75,'Compensated System (Method 2)')

%% Lag compensator - Ogata Example 6-7
% Steps to be followed:
% 1) Draw root locus of uncompensated system and locate closed loop poles from transient response.
% 2) Assume the standard form of lag compensator, given by equation 6-19 Ogata book.
% 3) Evaluate the particular static error constant provided in the problem.
% 4) Determine the required increase in static error constant to meet requirement.
% 5) Determine pole and zero of lag compensator to increase static error constant, without changing the original root locus a lot.
% 6) Draw new root locus and compare with original.
% 7) Adjust gain Kc of the compensator from the magnitude condition so that the dominant closed-loop poles lie at the desired location. Kc will be approximately 1.

num = 1.06;
deno = conv([1 0],conv([1 1], [1 2]));
sys_open = tf(num, deno);
denc = [1 3 2 1.06];
sys_closed = tf(num, denc);
% for the system above, 
% zeta = 0.491 (damping ratio of closed loop poles)
% wn = 0.673 rads/s and Kv = 0.53 s^-1
% Desired Kv = 5 s^-1 (about 10 times) without much change in original root locus
% We choose beta = 10, place zero of lag compensator at -0.05 and pole at -0.005.
% Kv_ = Kc_*beta*Kv, where Kv is static velocity constant of original system, Kc_ is the compensator gain (approx 1) 
% Kc_ of the compensator is obtained from magnitude condiition so that the dominant closed loop poles lie at the desired location.
K = 1.0235; % open loop gain 
Kc_ = K / 1.06;
num_comp = [1 0.05];
den_comp = [1 0.005];
sys_comp = tf(K*num_comp, den_comp);
sys_compensated = sys_comp*sys_open;
rlocus(sys_open);
hold on
rlocus(sys_compensated)

%%
% ***** Unit-ramp responses of compensated system and
% uncompensated system *****
% ***** Unit-ramp response will be obtained as the unit-step
% response of C(s)/[sR(s)] *****
% ***** Enter the numerators and denominators of C1(s)/[sR(s)]
% and C2(s)/[sR(s)], where C1(s) and C2(s) are Laplace
% transforms of the outputs of the compensated and un-
% compensated systems, respectively. *****
numc = [1.0235 0.0512];
denc = [1 3.005 2.015 1.0335 0.0512 0];
num = [1.06];
den = [1 3 2 1.06 0];
% ***** Specify the time range (such as t= 0:0.1:50) and enter
% step command and plot command. *****
t = 0:0.1:50;
c1 = step(numc,denc,t);
c2 = step(num,den,t);
plot(t,c1,'-',t,c2,'.',t,t,'--')
grid
text(2.2,27,'Compensated system');
text(26,21.3,'Uncompensated system');
title('Unit-Ramp Responses of Compensated and Uncompensated Systems')
xlabel('t Sec');
ylabel('Outputs c1 and c2')

%%
% ***** Unit-step responses of compensated system and
% uncompensated system *****
% ***** Enter the numerators and denominators of the
% compensated and uncompensated systems *****
numc = [1.0235 0.0512];
denc = [1 3.005 2.015 1.0335 0.0512];
num = [1.06];
den = [1 3 2 1.06];
% ***** Specify the time range (such as t = 0:0.1:40) and enter
% step command and plot command. *****
t = 0:0.1:40;
c1 = step(numc,denc,t);
c2 = step(num,den,t);
plot(t,c1,'-',t,c2,'.')
grid
text(13,1.12,'Compensated system')
text(13.6,0.88,'Uncompensated system')
title('Unit-Step Responses of Compensated and Uncompensated Systems')
xlabel('t Sec')
ylabel('Outputs c1 and c2')

% Refer to handwritten notes for brian douglas method of finding pole and
% zero location of lag compensator.



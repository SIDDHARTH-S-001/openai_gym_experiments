%% This script involes exercises from BS Greval Higher Engineering Mathematics Book - Linear Algebra
%% Example 2.29 - solution of linear equations
% The techniques below work when the linear equations don't form a singular matrix and the number of known variabes equal to no of equations
% Determinant-cramers rule 
A = [3 1 2;
     2 -3 -1;
     1 2 1];
D = [3 -3 4]';
x = det([D A(:,2) A(:,3)])/det(A);
y = det([A(:,1) D A(:,3)])/det(A);
z = det([A(:,1) A(:,2) D])/det(A);

% Matrix inversion method
X = inv(A)*D;

%% Exaple 2.46 
A = [1  1  3;
     1  3 -3;
    -2 -4 -4];

ch = poly(A); % characterestic eqn of A.
% Caley-Hamilton Theorem, every matrix satisfies its characterestic equation.
A_eqn = (A^3) - 20*A + 8;

% Steps to find inverse.
% A^3 - 20A + 8 = 0 
% A^3 - 20A = -8I
% A_in*A^3 - 20A_inv*A = -8A_inv
% A^2 - 20I = -8A_inv

I = eye(3);
A_inv = (5/2)*I - (1/8)*A^2


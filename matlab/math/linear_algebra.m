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
%% Verification Matrix
syms a b c d e f

mat1 = [0 -1 0 a;
        1 0 0 b;
        0 0 1 c;
        0 0 0 1
    ];

mat2 = [0 -1 0 d;
        1  0 0 e;
        0 0 1 f;
        0 0 0 1
    ];




%% Navin Matrix

syms theta1 l1 l2 theta2

M1 = [cos(theta1)    -sin(theta1) 0  l1*cos(theta1);
      sin(theta1)     cos(theta1) 0  l1*sin(theta1);
      0               0           1  0             ;
      0               0           0  1  ];

M2 = [cos(theta2)    -sin(theta2) 0  l2*cos(theta2);
      sin(theta2)     cos(theta2) 0  l2*sin(theta2);
      0               0           1  0             ;
      0               0           0  1  ];

Tn = M1 * M2

%% Daksiin matrix

syms theta1 l1 l2 theta2

M1 = [cos(theta1)    -sin(theta1) 0  0;
      sin(theta1)     cos(theta1) 0  0;
      0               0           1  0;
      0               0           0  1  ];

% M2 = [cos(theta2)    -sin(theta2) 0  l1*cos(theta2);
%       sin(theta2)     cos(theta2) 0  l1*sin(theta2);
%       0               0           1  0             ;
%       0               0           0  1  ];

M2 = [cos(theta2+theta1)    -sin(theta2+theta1) 0  l1*cos(theta2+theta1);
      sin(theta2+theta1)     cos(theta2+theta1) 0  l1*sin(theta2+theta1);
      0               0           1  0             ;
      0               0           0  1  ];

M3 = [1 0 0 l2;
      0 1 0 0 ;
      0 0 1 0 ;
      0 0 0 1;
    ];

Ti = M1*M2;

Td = M1*M2*M3;

%% Confirmation

syms theta1 theta2 l1 l2

M1 = [cos(theta1)    -sin(theta1) 0  0;
      sin(theta1)     cos(theta1) 0  0;
      0               0           1  0;
      0               0           0  1  ];

M2 = [cos(theta2)    -sin(theta2) 0  0;
      sin(theta2)     cos(theta2) 0  0;
      0               0           1  0;
      0               0           0  1  ];

M3 = [1 0 0 l1;
      0 1 0 0 ;
      0 0 1 0 ;
      0 0 0 1];

M4 = [1 0 0 l2;
      0 1 0 0 ;
      0 0 1 0 ;
      0 0 0 1];

Tc = M1*M2*M3*M4

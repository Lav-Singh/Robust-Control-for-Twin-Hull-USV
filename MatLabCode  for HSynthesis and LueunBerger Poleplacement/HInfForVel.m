% System Parameters (from URDF and dynamic equations)
mass = 180;       % Mass in kg
inertia = 446;    % Inertia around z-axis in kg.m^2
drag_x = 51.3;    % Linear drag in x (surge)
drag_xx = 72.4;   % Quadratic drag in x (surge)
drag_y = 40;      % Linear drag in y (sway)
drag_r = 400;     % Linear drag in yaw (r)
y_L = 1.03;       % Distance of left thruster from center in y
y_R = -1.03;      % Distance of right thruster from center in y

% Define the operating point (linearization around zero velocities)
u_op = 0;  % Surge velocity at operating point
v_op = 0;  % Sway velocity at operating point
r_op = 0;  % Yaw rate at operating point

% Evaluate Mass matrix M at operating point
M = [mass - drag_x, 0, 0;
     0, mass - drag_y, mass * y_L;
     0, mass * y_L, inertia - drag_r];

% Evaluate Coriolis matrix C at operating point (v = 0, u = 0, r = 0)
C = [0, -mass * r_op, mass * y_L * v_op + drag_r * v_op;
     mass * r_op, 0, drag_x * u_op;
     -mass * y_L * v_op - drag_r * v_op, -drag_x * u_op, 0];

% Evaluate Damping matrix D at operating point
D = [-drag_x - drag_xx * abs(u_op), 0, 0;
     0, -drag_y, 0;
     0, 0, -drag_r];

% State-Space Representation A and B Matrices
% A matrix is the dynamics: M * x_dot = -C(nu)*x - D(nu)*x
A = -inv(M) * (C + D);

% Modify the B matrix to reflect control via v (linear velocity) and omega (angular velocity)
B = inv(M) * [1, 0; 0, 0; 0, 1];

% C and D Matrices for full-state feedback
C = eye(3);  % Full-state feedback (outputs are the states)
D = zeros(3, 2);  % No direct feedthrough from inputs to outputs

% Check if the matrices are fully numeric
disp('A Matrix:');
disp(A);
disp('B Matrix:');
disp(B);

% Create the state-space system (ensure that the matrices are numeric)
sys = ss(double(A), double(B), C, D);

% Weighting Matrices for H-Infinity Synthesis
W1 = diag([2, 0, 1]);  % Higher weight on yaw for better performance
W2 = diag([10, 30]);       % Penalty on control effort
W3 = [];                      % No direct weighting for output

% Augmented system for H-infinity synthesis
P = augw(sys, W1, W2, W3);    % W2 is the weighting for the input, W1 for the output

% Number of measured outputs (C matrix rows)
nmeas = size(C, 1);

% Number of control inputs (B matrix columns)
ncon = size(B, 2);

% Perform H-infinity synthesis
[K, CL, gamma, info] = hinfsyn(P, nmeas, ncon);

% Display results
disp('H-infinity Gain Matrix:');
disp(K);

% Extract the state-space matrices from the controller K
[A_k, B_k, C_k, D_k] = ssdata(K);
disp('Matrix A_k:');
disp(A_k);
disp('Matrix B_k:');
disp(C_k);
disp('Matrix C_k:');
disp(C_k);
disp('Matrix D_k:');
disp(D_k);

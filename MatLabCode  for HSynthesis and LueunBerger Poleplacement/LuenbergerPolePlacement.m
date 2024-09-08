% Use the A and B matrices from the H-infinity synthesis
A = [0.3986, 0, 0;
     0, -0.0659, 2.6549;
     0, 0.2655, -2.0048];

B = [0.0078, 0;
     0, 0.0066;
     0, -0.0050];

C = eye(3);  % Identity matrix since we are measuring states directly

% Define the system's poles
sys_poles = eig(A);   % Calculate the poles of the system with H-infinity synthesis
disp('System poles:');
disp(sys_poles);

% Choose observer poles faster than the system poles
% Ideally, the observer poles should be about 2-10x faster than the slowest system poles
factor = 5;

% Compute desired observer poles by shifting the real parts of the system poles
desired_observer_poles = real(sys_poles) * factor;

% Adjust for stability (ensure observer poles are negative)
desired_observer_poles = min(real(sys_poles)) - abs(desired_observer_poles);

% Calculate the Luenberger observer gain using the place function
% Use A' and C' for observer design, and then transpose the result back
L = place(A', C', desired_observer_poles)';  % Transpose A and C for observer design
disp('Calculated Luenberger observer gain matrix L:');
disp(L);

% Verify the observer poles by checking the eigenvalues of (A - L*C)
observer_poles = eig(A - L * C);
disp('Observer poles (eigenvalues of (A - L*C)):');
disp(observer_poles);

% Plot the system poles and observer poles for comparison
figure;
hold on;
scatter(real(sys_poles), imag(sys_poles), 'bo', 'filled', 'DisplayName', 'System Poles');
scatter(real(observer_poles), imag(observer_poles), 'rx', 'filled', 'DisplayName', 'Observer Poles');
legend();
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('System Poles vs. Observer Poles');
grid on;


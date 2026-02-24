% --- OPTICAL SENSOR TO 4x4 TRANSFORMATION MATRIX ---
clear; clc;

%% 1. INPUT: Raw Sensor Data
% Simulated brightness values from your 6 LEDs (e.g., in volts or ADC counts)
% Let's assume a baseline of 0 means "centered/neutral" for simplicity.
V_raw = [0.15; -0.02; 0.05; 0.10; -0.12; 0.08]; 

%% 2. THE CALIBRATION MATRIX (C)
% This 6x6 matrix is the "secret sauce" of your specific hardware.
% You cannot guess this; it must be found experimentally during calibration.
% For this script, we generate a random dummy matrix to represent it.
C_matrix = rand(6, 6) * 0.01; % Dummy scaling factors

%% 3. EXTRACT THE 6DoF POSE VECTOR (P)
% This is where P = C * V happens.
% P will contain [x, y, z, roll(alpha), pitch(beta), yaw(gamma)]
P = C_matrix * V_raw;

% Unpack the vector for readability
x = P(1); y = P(2); z = P(3);
alpha = P(4); % Roll  (Rotation around X)
beta  = P(5); % Pitch (Rotation around Y)
gamma = P(6); % Yaw   (Rotation around Z)

fprintf('Decoded Pose:\n X: %.4fm, Y: %.4fm, Z: %.4fm\n Roll: %.4frad, Pitch: %.4frad, Yaw: %.4frad\n\n', ...
        x, y, z, alpha, beta, gamma);

%% 4. BUILD THE 4x4 HOMOGENEOUS TRANSFORMATION MATRIX (T)
% First, calculate the 3x3 Rotation Matrix (R) using standard Euler angles
% R = Rz(gamma) * Ry(beta) * Rx(alpha)

% Rotation around X (Roll)
Rx = [1, 0, 0;
      0, cos(alpha), -sin(alpha);
      0, sin(alpha), cos(alpha)];

% Rotation around Y (Pitch)
Ry = [cos(beta), 0, sin(beta);
      0, 1, 0;
      -sin(beta), 0, cos(beta)];

% Rotation around Z (Yaw)
Rz = [cos(gamma), -sin(gamma), 0;
      sin(gamma), cos(gamma), 0;
      0, 0, 1];

% Combined Rotation Matrix
R = Rz * Ry * Rx; 

% Combine Rotation and Translation into the final 4x4 T Matrix
T_sensor = eye(4);          % Start with a 4x4 identity matrix
T_sensor(1:3, 1:3) = R;     % Insert the 3x3 rotation block
T_sensor(1:3, 4) = [x; y; z]; % Insert the 3x1 translation column

disp('Final 4x4 Transformation Matrix (T_sensor):');
disp(T_sensor);
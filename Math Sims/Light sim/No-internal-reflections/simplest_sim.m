% -------------------------------------------------------------------------
% Simplified 6DoF Optical Sensor Model (No Internal Reflections)
% -------------------------------------------------------------------------
clear; clc; close all;

%% 1. PARAMETERS TO VARY
% --- Geometrical Parameters (mm) ---
R_led = 1.0;        % Radius where the 3 LEDs are placed from center
R_slit = 3.0;       % Radius where the 2 slits are placed (Drive this to test 7mm vs 10mm PCB)
Z_gap_nom = 2.5;    % Nominal vertical gap between bottom and top plate

% --- Optical Parameters ---
I0 = 1000;          % Peak axial luminous intensity of the LED (arbitrary units)
half_angle = 30;    % LED half-intensity viewing angle in degrees (determines beam width)
m = -log(2) / log(cosd(half_angle)); % Lambertian mode order

% --- 6DoF TOP PLATE MOVEMENT (Test your poses here!) ---
tx = 0.5;   % X Translation (mm)
ty = -0.2;  % Y Translation (mm)
tz = 0.2;   % Z Translation relative to nominal gap (mm)
rx = 5;     % Roll around X-axis (degrees)
ry = -3;    % Pitch around Y-axis (degrees)
rz = 15;    % Yaw around Z-axis (degrees)

%% 2. DEFINE BASE GEOMETRY (Static Bottom Plate)
% 3 LEDs placed 120 degrees apart on the bottom plate facing UP (+Z)
theta_leds = [0, 120, 240] * pi/180;
led_pos = [R_led*cos(theta_leds); R_led*sin(theta_leds); zeros(1,3)];
led_normals = repmat([0; 0; 1], 1, 3); % All pointing straight up

% 2 Slits (e.g., one 'vertical', one 'horizontal' axis) on the Top Plate facing DOWN (-Z)
theta_slits = [0, 90] * pi/180; 
slit_pos_nominal = [R_slit*cos(theta_slits); R_slit*sin(theta_slits); Z_gap_nom*ones(1,2)];
slit_normals_nominal = repmat([0; 0; -1], 1, 2); % Facing straight down

%% 3. APPLY 6DoF TRANSFORMATION TO TOP PLATE
% Create Rotation Matrices
Rx = [1 0 0; 0 cosd(rx) -sind(rx); 0 sind(rx) cosd(rx)];
Ry = [cosd(ry) 0 sind(ry); 0 1 0; -sind(ry) 0 cosd(ry)];
Rz = [cosd(rz) -sind(rz) 0; sind(rz) cosd(rz) 0; 0 0 1];
R = Rz * Ry * Rx; % Combined rotation matrix

% Translation Vector
T = [tx; ty; tz];

% Move the slits
slit_pos_moved = zeros(3,2);
slit_normals_moved = zeros(3,2);
for i = 1:2
    % Rotate and Translate positions
    % Note: rotation is applied relative to the center [0;0;Z_gap_nom]
    centered_pos = slit_pos_nominal(:,i) - [0; 0; Z_gap_nom];
    slit_pos_moved(:,i) = (R * centered_pos) + [0; 0; Z_gap_nom] + T;
    
    % Rotate normals
    slit_normals_moved(:,i) = R * slit_normals_nominal(:,i);
end

%% 4. CALCULATE OPTICAL IRRADIANCE AT SLITS
% Initialize brightness readings for the 2 slits
slit_brightness = [0, 0]; 

for s = 1:2 % Loop through each slit
    for L = 1:3 % Loop through each LED contributing light to that slit
        
        % Vector from LED to Slit
        v = slit_pos_moved(:,s) - led_pos(:,L);
        r = norm(v); % Distance
        v_dir = v / r; % Normalized direction vector
        
        % Emission Angle (theta): Angle between LED normal and light ray
        cos_theta = dot(led_normals(:,L), v_dir);
        
        % Incidence Angle (phi): Angle between Slit normal and incoming light ray
        cos_phi = dot(slit_normals_moved(:,s), -v_dir);
        
        % Only calculate if light is actually hitting the front of the slit
        % and emitting from the front of the LED
        if cos_theta > 0 && cos_phi > 0
            % The core radiometric equation
            irradiance = (I0 * (cos_theta^m) / (r^2)) * cos_phi;
            slit_brightness(s) = slit_brightness(s) + irradiance;
        end
    end
end

%% 5. OUTPUT & VISUALIZATION
fprintf('--- Simulation Results ---\n');
fprintf('Slit 1 (0 deg) Brightness:  %.2f\n', slit_brightness(1));
fprintf('Slit 2 (90 deg) Brightness: %.2f\n', slit_brightness(2));

% Plotting the Setup
figure('Name', '6DoF Sensor Geometry', 'Color', 'w'); hold on; grid on; axis equal;
view(3); xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

% Plot LEDs
scatter3(led_pos(1,:), led_pos(2,:), led_pos(3,:), 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
for i=1:3, quiver3(led_pos(1,i), led_pos(2,i), led_pos(3,i), led_normals(1,i), led_normals(2,i), led_normals(3,i), 0.5, 'r', 'LineWidth', 2); end

% Plot Slits (Nominal - transparent)
scatter3(slit_pos_nominal(1,:), slit_pos_nominal(2,:), slit_pos_nominal(3,:), 80, 'k');

% Plot Slits (Moved)
scatter3(slit_pos_moved(1,:), slit_pos_moved(2,:), slit_pos_moved(3,:), 100, 'b', 'filled', 'MarkerEdgeColor', 'k');
for i=1:2, quiver3(slit_pos_moved(1,i), slit_pos_moved(2,i), slit_pos_moved(3,i), slit_normals_moved(1,i), slit_normals_moved(2,i), slit_normals_moved(3,i), 0.5, 'b', 'LineWidth', 2); end

% Draw light rays from LEDs to Slits
for s = 1:2
    for L = 1:3
        plot3([led_pos(1,L), slit_pos_moved(1,s)], [led_pos(2,L), slit_pos_moved(2,s)], [led_pos(3,L), slit_pos_moved(3,s)], 'y--', 'LineWidth', 0.5);
    end
end
title('Sensor Geometry: Red = LEDs, Blue = Moved Slits');
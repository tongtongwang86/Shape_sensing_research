% --- SUPER BASIC SOFT ROBOT PCC SIMULATOR ---
clear; clc; close all;

%% 1. DEFINE THE PHYSICAL ROBOT
L = 0.05;       % Length of each soft segment in meters (5 cm)
r_tube = 0.01;  % Radius of the soft robot tube (1 cm)
num_segments = 2; 

%% 2. SIMULATE MOVEMENT & GENERATE SENSOR DATA
% Let's pretend the robot is bending. We define the bend angle (theta) 
% and the bending plane angle (phi) for each segment.
% Segment 1: Bends 45 degrees, pointing along the X-axis
theta_sim = [deg2rad(0), deg2rad(90)]; 
phi_sim   = [deg2rad(0),  deg2rad(90)]; 

% Create an array to hold our "Sensor Data" (Homogeneous Transformation Matrices)
% T_sensors(:,:,1) is the base (fixed). 
% T_sensors(:,:,2) is Sensor 1. 
% T_sensors(:,:,3) is Sensor 2.
T_sensors = zeros(4, 4, num_segments + 1);
T_sensors(:,:,1) = eye(4); % Base is at [0,0,0] with no rotation

% Generate the 6DoF Sensor Data for the bent state
T_current = eye(4);
for i = 1:num_segments
    theta = theta_sim(i);
    phi = phi_sim(i);
    
    % Forward Kinematics to find the pose of the sensor at the end of the segment
    if theta == 0
        % Straight line translation
        T_seg = [eye(3), [0; 0; L]; 0 0 0 1];
    else
        kappa = theta / L;
        % Position of the sensor
        p_x = (1/kappa) * (1 - cos(theta)) * cos(phi);
        p_y = (1/kappa) * (1 - cos(theta)) * sin(phi);
        p_z = (1/kappa) * sin(theta);
        
        % Rotation of the sensor
        c_phi = cos(phi); s_phi = sin(phi); c_th = cos(theta); s_th = sin(theta);
        R_seg = [c_phi^2*(c_th-1)+1,  s_phi*c_phi*(c_th-1),   c_phi*s_th;
                 s_phi*c_phi*(c_th-1),s_phi^2*(c_th-1)+1,     s_phi*s_th;
                 -c_phi*s_th,         -s_phi*s_th,            c_th];
             
        T_seg = [R_seg, [p_x; p_y; p_z]; 0 0 0 1];
    end
    
    % Combine with previous segments to get global pose
    T_current = T_current * T_seg;
    T_sensors(:,:,i+1) = T_current;
end

disp('--- SENSOR DATA GENERATED ---');
disp('Pose of Top Sensor (Segment 2):');
disp(T_sensors(:,:,3));

%% 3. RECONSTRUCT SHAPE FROM SENSOR DATA (PCC)
% Now, we pretend we ONLY have T_sensors. We will draw the tube.

figure('Color','w','Name','Soft Robot Shape Sensing');
hold on; grid on; axis equal; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
xlim([-0.05 0.1]); ylim([-0.05 0.1]); zlim([0 0.12]);

points_per_segment = 20;

% Loop through each segment to draw it
for i = 1:num_segments
    % Get the start and end poses for this specific segment
    T_start = T_sensors(:,:,i);
    
    % In a real scenario, you extract theta and phi from the relative 
    % transformation between T_start and T_end. Since we simulated it via PCC, 
    % we already know the underlying theta and phi (theta_sim, phi_sim).
    theta = theta_sim(i); 
    phi = phi_sim(i);
    
    % Create points along the arc (arc length s goes from 0 to L)
    s_vals = linspace(0, L, points_per_segment);
    
    for j = 1:length(s_vals)
        s = s_vals(j);
        
        % 1. Calculate local position on the arc
        if theta == 0
            p_local = [0; 0; s];
            R_local = eye(3);
        else
            kappa = theta / L;
            p_local = [(1/kappa)*(1-cos(kappa*s))*cos(phi);
                       (1/kappa)*(1-cos(kappa*s))*sin(phi);
                       (1/kappa)*sin(kappa*s)];
                   
            % Local rotation at point s
            c_p = cos(phi); s_p = sin(phi); c_ks = cos(kappa*s); s_ks = sin(kappa*s);
            R_local = [c_p^2*(c_ks-1)+1,  s_p*c_p*(c_ks-1),   c_p*s_ks;
                       s_p*c_p*(c_ks-1),  s_p^2*(c_ks-1)+1,   s_p*s_ks;
                       -c_p*s_ks,         -s_p*s_ks,          c_ks];
        end
        
        % 2. Transform local point to global space using the base of this segment (T_start)
        p_global = T_start(1:3,1:3) * p_local + T_start(1:3,4);
        R_global = T_start(1:3,1:3) * R_local;
        
        % 3. Draw the circular cross-section (the "tube") at this point
        theta_circle = linspace(0, 2*pi, 15);
        circle_x = r_tube * cos(theta_circle);
        circle_y = r_tube * sin(theta_circle);
        circle_z = zeros(1, length(theta_circle));
        
        % Transform the circle points to the current arc position and orientation
        circle_points = R_global * [circle_x; circle_y; circle_z] + p_global;
        
        % Plot the cross-section (forming a wireframe tube)
        plot3(circle_points(1,:), circle_points(2,:), circle_points(3,:), 'Color', [0.5 0.5 0.5 0.5]);
        
        % Plot the backbone line
        scatter3(p_global(1), p_global(2), p_global(3), 10, 'r', 'filled');
    end
    
    % Plot the Sensor Node (Rigid PCB Plate)
    sensor_pos = T_sensors(1:3, 4, i+1);
    scatter3(sensor_pos(1), sensor_pos(2), sensor_pos(3), 100, 'b', 'filled', 'MarkerEdgeColor', 'k');
end

title('Soft Robot: 6DoF Sensor Data to PCC Tube');
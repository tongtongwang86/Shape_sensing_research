function spacemouse_sim()
    % 1. Define Geometry and Transformation Matrix (M)
    % Rows: X, Y, Z, Rx, Ry, Rz | Cols: h1, v1, h2, v2, h3, v3
    M = [ 1.0,  0.0, -0.5,  0.0, -0.5,  0.0;  % X
          0.0,  0.0,  0.86, 0.0, -0.86, 0.0;  % Y
          0.0,  1.0,  0.0,  1.0,  0.0,  1.0;  % Z
          0.0,  1.0,  0.0, -0.5,  0.0, -0.5;  % Rx
          0.0,  0.0,  0.0,  0.86, 0.0, -0.86; % Ry
          1.0,  0.0,  1.0,  0.0,  1.0,  0.0]; % Rz

    % 2. SET DESIRED STRAIN (Change these to see the tube move!)
    % Translation (units) and Rotation (degrees)
    target_X = 0;   target_Y = 0;   target_Z = 0;
    target_Rx = 0; target_Ry = 0; target_Rz = 0;

    % 3. GENERATE SYNTHETIC SENSOR DATA (Inverse Matrix)
    movement_vec = [target_X; target_Y; target_Z; target_Rx; target_Ry; target_Rz];
    sensor_data = M \ movement_vec; % Solving M * sensors = movement

    % 4. RECONSTRUCT MOTION FROM SENSORS (Forward Matrix)
    % In a real device, this is the only part the software sees
    rec = M * sensor_data;
    
    % 5. VISUALIZATION
    figure('Color', 'w', 'Name', '6DoF Strain Visualization');
    [x, y, z] = cylinder([1 1], 20); % Create basic tube
    z = z * 2; % Make it taller
    
    % Apply Rotations (using reconstructed values)
    % Convert deg to rad for rotation matrices
    rx = deg2rad(rec(4)); ry = deg2rad(rec(5)); rz = deg2rad(rec(6));
    
    % Standard 3D Rotation Matrices
    RotX = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
    RotY = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
    RotZ = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
    TotalRot = RotZ * RotY * RotX;

    % Transform every point in the cylinder
    orig_pts = [x(:), y(:), z(:)]';
    transformed_pts = TotalRot * orig_pts;
    
    % Apply Translation
    tx = transformed_pts(1,:) + rec(1);
    ty = transformed_pts(2,:) + rec(2);
    tz = transformed_pts(3,:) + rec(3);
    
    % Reshape back for plotting
    x_final = reshape(tx, size(x));
    y_final = reshape(ty, size(y));
    z_final = reshape(tz, size(z));

    % Draw the "Rest" position (ghosted)
    surf(x, y, z, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
    hold on;
    
    % Draw the "Strained" position
    s = surf(x_final, y_final, z_final, 'FaceColor', '#0072BD', 'EdgeAlpha', 0.3);
    
    % Formatting
    axis equal; grid on; view(3);
    xlabel('X-Axis'); ylabel('Y-Axis'); zlabel('Z-Axis');
    title('6DoF Reconstructed Strain (Tube Model)');
    xlim([-10 10]); ylim([-10 10]); zlim([0 15]);
    
    fprintf('--- Reconstructed 6DoF Values ---\n');
    fprintf('X: %.2f, Y: %.2f, Z: %.2f\n', rec(1), rec(2), rec(3));
    fprintf('Rx: %.2f°, Ry: %.2f°, Rz: %.2f°\n', rec(4), rec(5), rec(6));
end

spacemouse_sim()
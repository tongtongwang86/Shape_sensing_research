function sensor_scaling_analysis()
    % Parameters
    diameters = linspace(1, 20, 100); % Board diameter from 1mm to 20mm
    sensor_noise_std = 0.01; % 1% noise floor (standard for 10-bit ADC/LED jitter)
    num_trials = 1000;
    
    error_xyz = zeros(size(diameters));
    error_rot = zeros(size(diameters));

    for i = 1:length(diameters)
        L = diameters(i) / 2; % Leverage arm (radius)
        
        % Build Geometry-Dependent Matrix M
        % As L shrinks, the coefficients for rotations must grow to compensate
        M = [ 1.0,  0.0, -0.5,  0.0, -0.5,  0.0;     % X
              0.0,  0.0,  0.86, 0.0, -0.86, 0.0;    % Y
              0.0,  1.0,  0.0,  1.0,  0.0,  1.0;    % Z
              0.0,  1/L,  0.0, -0.5/L, 0.0, -0.5/L; % Rx (Leverage L matters here!)
              0.0,  0.0,  0.0,  0.86/L, 0.0, -0.86/L;% Ry
              1/L,  0.0,  1/L,  0.0,  1/L,  0.0];   % Rz
        
        % Simulation: Apply random noise to sensors and reconstruct 6DoF
        noise = sensor_noise_std * randn(6, num_trials);
        reconstructed = M * noise; 
        
        % Calculate Root Mean Square Error
        error_xyz(i) = rms(vecnorm(reconstructed(1:3, :))); % Translation Error
        error_rot(i) = rms(vecnorm(reconstructed(4:6, :))); % Rotation Error
    end

    % Plotting
    figure('Color', 'w');
    yyaxis left
    plot(diameters, error_xyz, 'LineWidth', 2); ylabel('Translation Error (mm)');
    hold on;
    yyaxis right
    plot(diameters, error_rot, 'LineWidth', 2); ylabel('Rotation Error (Degrees)');
    
    xlabel('Sensor Board Diameter (mm)');
    title('The "Miniaturization Tax": Noise Amplification vs. Size');
    grid on;
    
    % Highlight your 3mm target
    xline(3, '--r', '3mm Target Area');
end

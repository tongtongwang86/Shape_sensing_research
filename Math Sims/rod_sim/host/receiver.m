%% receiver.m - Load strain and shape data from simulation logs
%
% Usage:
%   [strain, shape] = receiver();
%   [strain, shape] = receiver('path/to/data/logs');
%
% Returns:
%   strain - struct with fields:
%       .timestamps  - Nx1 vector of timestamps (ms)
%       .epsilon     - Nx3x6 array (samples x sensors x strain_dims)
%
%   shape - struct with fields:
%       .timestamps  - Mx1 vector of timestamps (ms)
%       .points      - MxPx3 array (samples x points x xyz)

function [strain, shape] = receiver(data_dir)
    if nargin < 1
        data_dir = '../data/logs';
    end
    
    strain_file = fullfile(data_dir, 'strain_data.csv');
    shape_file = fullfile(data_dir, 'shape_data.csv');
    
    %% Load Strain Data
    fprintf('Loading strain data from %s...\n', strain_file);
    
    if ~exist(strain_file, 'file')
        error('Strain data file not found: %s\nRun the firmware simulation first.', strain_file);
    end
    
    strain_table = readtable(strain_file);
    n_samples = height(strain_table);
    
    strain.timestamps = strain_table.timestamp_ms;
    strain.epsilon = zeros(n_samples, 3, 6);  % samples x sensors x strain_dims
    
    % Parse strain columns: eps1_s1, eps2_s1, ..., eps6_s3
    for s = 1:3
        for i = 1:6
            col_name = sprintf('eps%d_s%d', i, s);
            strain.epsilon(:, s, i) = strain_table.(col_name);
        end
    end
    
    fprintf('  Loaded %d strain samples\n', n_samples);
    
    %% Load Shape Data
    fprintf('Loading shape data from %s...\n', shape_file);
    
    if ~exist(shape_file, 'file')
        error('Shape data file not found: %s\nRun the firmware simulation first.', shape_file);
    end
    
    shape_table = readtable(shape_file);
    
    % Get unique timestamps and number of points
    unique_times = unique(shape_table.timestamp_ms);
    n_frames = length(unique_times);
    n_points = sum(shape_table.timestamp_ms == unique_times(1));
    
    shape.timestamps = unique_times;
    shape.points = zeros(n_frames, n_points, 3);  % frames x points x xyz
    
    % Parse shape data
    for f = 1:n_frames
        t = unique_times(f);
        mask = shape_table.timestamp_ms == t;
        frame_data = shape_table(mask, :);
        
        % Sort by point index to ensure correct ordering
        [~, sort_idx] = sort(frame_data.point_idx);
        frame_data = frame_data(sort_idx, :);
        
        shape.points(f, :, 1) = frame_data.x;
        shape.points(f, :, 2) = frame_data.y;
        shape.points(f, :, 3) = frame_data.z;
    end
    
    fprintf('  Loaded %d shape frames (%d points each)\n', n_frames, n_points);
    
    %% Print summary
    fprintf('\nData Summary:\n');
    fprintf('  Time range: %.2f to %.2f seconds\n', ...
            strain.timestamps(1)/1000, strain.timestamps(end)/1000);
    fprintf('  Sample rate: %.1f Hz\n', ...
            1000 / mean(diff(strain.timestamps)));
    
    % Show sample strain values
    fprintf('\nSample strain at t=0:\n');
    for s = 1:3
        fprintf('  Sensor %d: twist=%.3f, bend_y=%.3f, bend_z=%.3f rad/m\n', ...
                s, strain.epsilon(1,s,1), strain.epsilon(1,s,2), strain.epsilon(1,s,3));
    end
    
    % Show tip position range
    tip_pos = squeeze(shape.points(:, end, :));
    fprintf('\nTip position range:\n');
    fprintf('  X: [%.4f, %.4f] m\n', min(tip_pos(:,1)), max(tip_pos(:,1)));
    fprintf('  Y: [%.4f, %.4f] m\n', min(tip_pos(:,2)), max(tip_pos(:,2)));
    fprintf('  Z: [%.4f, %.4f] m\n', min(tip_pos(:,3)), max(tip_pos(:,3)));
end

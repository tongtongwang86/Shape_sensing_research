%% render.m - 3D Shape Visualization for Soft Robot Proprioception
%
% Reads shape data from simulation and renders 3D reconstruction
% Based on the Flex-2P6D sensor and GVS model
%
% Usage:
%   1. Compile and run the C simulation:
%      >> cd ../firmware_sim
%      >> !gcc -o sim_sensor main.c -lm && ./sim_sensor
%
%   2. Run this script:
%      >> render
%
% The visualization shows:
%   - 3D tube representing the soft robot shape
%   - Color coding for curvature along the length
%   - Time animation of the deformation

clear; clc; close all;

%% Parameters
ROBOT_RADIUS = 0.016;     % 16mm radius (32mm diameter)
NUM_TUBE_FACES = 16;      % Faces for tube rendering
FRAME_SKIP = 2;           % Skip frames for faster playback

%% Load Data using receiver
[strain, shape] = receiver('../data/logs');

num_frames = length(shape.timestamps);
num_points = size(shape.points, 2);

fprintf('\nStarting visualization with %d frames...\n', num_frames);

%% Setup Figure
fig = figure('Name', 'Soft Robot Shape Reconstruction', ...
             'Position', [100, 100, 1000, 800], ...
             'Color', 'w');

% Create subplots
ax_main = subplot(1, 1, 1);
hold on; grid on; box on;
axis equal;
view(30, 20);

% Axis labels
xlabel('y (m)', 'FontSize', 12);
ylabel('z (m)', 'FontSize', 12);
zlabel('x (m)', 'FontSize', 12);
title('Soft Robot 3D Shape Reconstruction', 'FontSize', 14);

% Set axis limits based on robot length
L = 0.165;  % Robot length
axis_range = [-0.08, 0.08, -0.08, 0.08, -0.25, 0.02];
axis(axis_range);

% Colorbar for strain visualization
colormap(jet);
cb = colorbar;
cb.Label.String = 'Curvature (rad/m)';
cb.Label.FontSize = 11;

% Create colormap for elongation/compression
% Red = elongation, Blue = compression, Green = neutral
strain_cmap = [
    0.0, 0.0, 0.8;   % Blue (compression)
    0.0, 0.5, 0.8;
    0.0, 0.8, 0.6;
    0.2, 0.9, 0.2;   % Green (neutral)
    0.6, 0.9, 0.0;
    0.9, 0.7, 0.0;
    0.9, 0.3, 0.0;   % Red (elongation)
    0.9, 0.0, 0.0
];
colormap(strain_cmap);

%% Animation Loop
fprintf('Starting animation...\n');

% Initialize graphics handles
tube_surf = [];
centerline_plot = [];
tip_marker = [];
frame_arrows = [];

% Time text
time_text = text(0.02, 0.02, 0, '', 'Units', 'normalized', ...
                 'FontSize', 12, 'FontWeight', 'bold');

for frame_idx = 1:FRAME_SKIP:num_frames
    t_ms = shape.timestamps(frame_idx);
    
    % Extract points for this frame
    x = squeeze(shape.points(frame_idx, :, 1))';
    y = squeeze(shape.points(frame_idx, :, 2))';
    z = squeeze(shape.points(frame_idx, :, 3))';
    
    % Calculate curvature for coloring (approximate from position)
    curvature = zeros(size(x));
    for i = 2:length(x)-1
        % Second derivative approximation
        dx = (x(i+1) - 2*x(i) + x(i-1));
        dy = (y(i+1) - 2*y(i) + y(i-1));
        dz = (z(i+1) - 2*z(i) + z(i-1));
        curvature(i) = sqrt(dx^2 + dy^2 + dz^2) * 100;  % Scale for visibility
    end
    curvature(1) = curvature(2);
    curvature(end) = curvature(end-1);
    
    % Normalize curvature for colormap
    curv_min = 0;
    curv_max = 15;  % rad/m range
    curv_norm = (curvature - curv_min) / (curv_max - curv_min);
    curv_norm = max(0, min(1, curv_norm));
    
    % Generate tube mesh around centerline
    [tube_X, tube_Y, tube_Z, tube_C] = generate_tube(x, y, z, ROBOT_RADIUS, NUM_TUBE_FACES, curv_norm);
    
    % Clear previous graphics
    if ~isempty(tube_surf)
        delete(tube_surf);
        delete(centerline_plot);
        delete(tip_marker);
        delete(frame_arrows);
    end
    
    % Draw tube surface
    tube_surf = surf(ax_main, tube_X, tube_Y, tube_Z, tube_C, ...
                     'EdgeColor', 'none', ...
                     'FaceAlpha', 0.9, ...
                     'FaceLighting', 'gouraud');
    
    % Draw centerline
    centerline_plot = plot3(ax_main, x, y, z, 'k-', 'LineWidth', 1);
    
    % Draw tip marker (coordinate frame)
    tip_pos = [x(end), y(end), z(end)];
    tip_marker = plot3(ax_main, tip_pos(1), tip_pos(2), tip_pos(3), ...
                       'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Draw coordinate frame at tip
    frame_arrows = draw_frame(ax_main, x, y, z, 0.02);
    
    % Update time display
    time_text.String = sprintf('t = %.2f s', t_ms / 1000);
    
    % Add lighting
    if frame_idx == 1
        light('Position', [1, 1, 1], 'Style', 'infinite');
        light('Position', [-1, -1, 0.5], 'Style', 'infinite');
        material dull;
    end
    
    % Update colorbar limits
    caxis([curv_min, curv_max]);
    
    drawnow;
    
    % Small pause for animation
    pause(0.02);
end

fprintf('Animation complete.\n');

%% Helper Functions

function [X, Y, Z, C] = generate_tube(cx, cy, cz, radius, n_faces, colors)
    % Generate tube mesh around a centerline
    % cx, cy, cz: centerline coordinates
    % radius: tube radius
    % n_faces: number of faces around circumference
    % colors: color values for each point along centerline
    
    n_pts = length(cx);
    theta = linspace(0, 2*pi, n_faces+1);
    
    X = zeros(n_pts, n_faces+1);
    Y = zeros(n_pts, n_faces+1);
    Z = zeros(n_pts, n_faces+1);
    C = zeros(n_pts, n_faces+1);
    
    for i = 1:n_pts
        % Calculate local coordinate frame
        if i == 1
            tangent = [cx(2)-cx(1), cy(2)-cy(1), cz(2)-cz(1)];
        elseif i == n_pts
            tangent = [cx(i)-cx(i-1), cy(i)-cy(i-1), cz(i)-cz(i-1)];
        else
            tangent = [cx(i+1)-cx(i-1), cy(i+1)-cy(i-1), cz(i+1)-cz(i-1)];
        end
        tangent = tangent / (norm(tangent) + 1e-10);
        
        % Find perpendicular vectors
        if abs(tangent(3)) < 0.9
            up = [0, 0, 1];
        else
            up = [1, 0, 0];
        end
        normal = cross(tangent, up);
        normal = normal / (norm(normal) + 1e-10);
        binormal = cross(tangent, normal);
        
        % Generate circle points
        for j = 1:n_faces+1
            offset = radius * (cos(theta(j)) * normal + sin(theta(j)) * binormal);
            X(i, j) = cx(i) + offset(1);
            Y(i, j) = cy(i) + offset(2);
            Z(i, j) = cz(i) + offset(3);
            C(i, j) = colors(i);
        end
    end
end

function h = draw_frame(ax, cx, cy, cz, scale)
    % Draw coordinate frame at tip of robot
    % Returns handle array for deletion
    n = length(cx);
    tip = [cx(n), cy(n), cz(n)];
    
    % Calculate tip orientation from last few points
    if n > 3
        tangent = [cx(n)-cx(n-3), cy(n)-cy(n-3), cz(n)-cz(n-3)];
    else
        tangent = [0, 0, -1];
    end
    tangent = tangent / (norm(tangent) + 1e-10);
    
    % Local frame
    if abs(tangent(3)) < 0.9
        up = [0, 0, 1];
    else
        up = [1, 0, 0];
    end
    x_axis = tangent;
    y_axis = cross([0,0,1], x_axis);
    y_axis = y_axis / (norm(y_axis) + 1e-10);
    z_axis = cross(x_axis, y_axis);
    
    % Draw axes
    h(1) = quiver3(ax, tip(1), tip(2), tip(3), x_axis(1)*scale, x_axis(2)*scale, x_axis(3)*scale, ...
            'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    h(2) = quiver3(ax, tip(1), tip(2), tip(3), y_axis(1)*scale, y_axis(2)*scale, y_axis(3)*scale, ...
            'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    h(3) = quiver3(ax, tip(1), tip(2), tip(3), z_axis(1)*scale, z_axis(2)*scale, z_axis(3)*scale, ...
            'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end

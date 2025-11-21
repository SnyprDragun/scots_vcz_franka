%% visualize_scots_controller.m
%
%  Loads and visualizes a 3D controller synthesized by SCOTS.
%  This script is designed for the 'xyz_roll_pitch_yaw.csv' file generated
%  by the C++ code, which (with N_LINKS=3) produces a 3D (x,y,z) controller.
%
%  This script performs two main tasks:
%  1. Static Plot: Displays the entire winning domain and the
%     associated control vector field using quiver3.
%  2. Dynamic Plot: Simulates a single trajectory from a starting point
%     in the winning domain until it reaches the target set.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;
close all;
%% 1. Configuration and Data Loading
% --- Configuration ---
csv_filename = 'fr3_hw_ex_2_t2.csv';
% IMPORTANT: Must match the 'tau' in your latest C++ code.
tau = 0.3; 
% --- Target Set Definition (from C++ code) ---
% The C++ code defines the target for cell *centers* x in [3.6, 3.9].
target1_min_pos = [0.5, -0.05, 0.35];
target1_size = [0.1, 0.1, 0.05];
target2_min_pos = [0.35, 0.15, 0.35];
target2_size = [0.1, 0.1, 0.05];
target3_min_pos = [0.35, -0.25, 0.35];
target3_size = [0.1, 0.1, 0.05];
% --- Obstacle Definition (Requested obstacle for visualization) ---
obstacle1_min_pos = [-0.3, -0.3, 0.2];
obstacle1_size = [0.6, 0.6, 0.5];
obstacle2_min_pos = [0.6, -0.7, 0.2];
obstacle2_size = [0.1, 1.4, 0.5];
obstacle3_min_pos = [-0.7, -0.7, 0.2];
obstacle3_size = [0.1, 1.4, 0.5];
obstacle4_min_pos = [-0.6, 0.6, 0.2];
obstacle4_size = [1.2, 0.1, 0.5];
obstacle5_min_pos = [-0.6, -0.7, 0.2];
obstacle5_size = [1.2, 0.1, 0.5];

% obstacle6_min_pos = [0.5, -0.05, 0.35];
% obstacle6_size = [0.1, 0.1, 0.05];
% obstacle7_min_pos = [0.35, 0.15, 0.35];
% obstacle7_size = [0.1, 0.1, 0.05];
% obstacle8_min_pos = [0.35, -0.25, 0.35];
% obstacle8_size = [0.1, 0.1, 0.05];

obstacle9_min_pos = [0.2, 0.05, 0.2];
obstacle9_size = [0.4, 0.1, 0.3];
obstacle10_min_pos = [0.2, -0.15, 0.2];
obstacle10_size = [0.4, 0.1, 0.3];
% obstacle11_min_pos = [-0.05, 0.3, 0.2];
% obstacle11_size = [0.1, 0.3, 0.3];
% --- Load Data ---
fprintf('Loading controller data from %s...\n', csv_filename);
try
    data_table = readtable(csv_filename);
    % Convert table to numeric array
    % It should have 6 columns: x, y, z, vx, vy, vz
    data = data_table{:,:};
    
    % Check if data is 6D (3 state + 3 input)
    if size(data, 2) ~= 6
        error('Expected 6 columns (x,y,z,vx,vy,vz), but found %d. Please check your C++ code and CSV output.', size(data, 2));
    end
    
    states = data(:, 1:3);
    inputs = data(:, 4:6);
catch ME
    fprintf('Error loading or parsing file: %s\n', ME.message);
    fprintf('Please make sure the file ''%s'' is in the same directory and is not empty.\n', csv_filename);
    return;
end
% --- Get Winning Domain (W) and Averaged Control Field (U) ---
% Find unique states (W = Winning Domain)
[W, ~, ic] = unique(states, 'rows');
num_winning_states = size(W, 1);
if num_winning_states == 0
    fprintf('No winning states found in the file. Cannot visualize.\n');
    return;
end
fprintf('Found %d unique winning states.\n', num_winning_states);
% Calculate average control input for each unique state
U = zeros(num_winning_states, 3);
for i = 1:num_winning_states
    % Find all controls for the i-th unique state
    controls_for_this_state = inputs(ic == i, :);
    % Compute the mean
    U(i, :) = mean(controls_for_this_state, 1);
end
fprintf('Controller data processed.\n');
%% 2. Static Visualization: Winning Domain and Control Field
fprintf('Displaying static controller plot...\n');
figure;
hold on;
% Plot the target set (Red)
plotCuboid(target1_min_pos, target1_size, 'r', 0.1, 'Target Set');
plotCuboid(target2_min_pos, target2_size, 'r', 0.1, 'Target Set');
plotCuboid(target3_min_pos, target3_size, 'r', 0.1, 'Target Set');
% % Plot the obstacle (Gray)
plotCuboid(obstacle1_min_pos, obstacle1_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle2_min_pos, obstacle2_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle3_min_pos, obstacle3_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle4_min_pos, obstacle4_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle5_min_pos, obstacle5_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle6_min_pos, obstacle6_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle7_min_pos, obstacle7_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle8_min_pos, obstacle8_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle9_min_pos, obstacle9_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle10_min_pos, obstacle10_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle11_min_pos, obstacle11_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% Plot the winning domain states
scatter3(W(:,1), W(:,2), W(:,3), ...
    15, 'b', 'filled', 'MarkerFaceAlpha', 0.2);
% Plot the control vector field
% We scale the arrows by 0.5 for better visibility
quiver3(W(:,1), W(:,2), W(:,3), ...
    U(:,1), U(:,2), U(:,3), 0.5, 'k'); % 'k' is black
% Setup plot aesthetics
title('SCOTS Controller: Winning Domain (blue), Target (red), and Obstacle (gray)');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
% Custom legend since plotCuboid returns handles
h_scatter = findobj(gca, 'Type', 'scatter');
h_quiver = findobj(gca, 'Type', 'quiver');
h_target = findobj(gca, 'DisplayName', 'Target Set');
h_obstacle = findobj(gca, 'DisplayName', 'Obstacle');
legend([h_target(1), h_obstacle(1), h_scatter(1), h_quiver(1)], ...
       'Target Set', 'Obstacle', 'Winning Domain', 'Control Vectors', 'Location', 'best');
grid on;
axis equal; % Important for correct 3D representation
view(30, 25); % Set a nice 3D viewing angle
xlim([-0.7, 0.7]); ylim([-0.7, 0.7]); zlim([0.0, 0.7]); % Match state space bounds
hold off;
%% 3. Dynamic Simulation: Trajectory
fprintf('Simulating trajectory...\n');
% --- Select Initial State ---
% We must pick a state that is IN the winning domain (W).

%%%%%%%%% TARGET 1 %%%%%%%%%
% target1_center = target1_min_pos + target1_size / 2;
% distances = sum((W - target1_center).^2, 2);
%%%%%%%% TARGET 2 %%%%%%%%%
target2_center = target2_min_pos + target2_size / 2;
distances = sum((W - target2_center).^2, 2);
%%%%%%%%% TARGET 3 %%%%%%%%%
% target3_center = target3_min_pos + target3_size / 2;
% distances = sum((W - target3_center).^2, 2);

[~, idx_start] = max(distances);
% x_current = [0.4, 0.0, 0.5];
% x_current = [0.34,0.2,0.36];
% x_current = [0.5,-0.02,0.36];
x_current = [0.36,-0.2,0.34];
% --- Simulation Setup ---
max_steps = 10; % Increased steps since tau is smaller
trajectory = zeros(max_steps + 1, 3);
trajectory(1, :) = x_current;
x_start = x_current;
fprintf('Starting simulation from [x=%.2f, y=%.2f, z=%.2f]\n', ...
    x_current(1), x_current(2), x_current(3));
% --- Simulation Loop ---
for k = 1:max_steps
    % 1. Check if inside target set
    % if all(x_current >= target1_min_pos) && all(x_current <= (target1_min_pos + target1_size))
    if all(x_current >= target2_min_pos) && all(x_current <= (target2_min_pos + target2_size))
    % if all(x_current >= target3_min_pos) && all(x_current <= (target3_min_pos + target3_size))
        fprintf('Reached target set in %d steps!\n', k-1);
        trajectory = trajectory(1:k, :); % Trim unused part of trajectory
        break;
    end
    
    % 2. Find the control for the current state
    % Find the index 'loc' in W that matches x_current
    [is_member, loc] = ismember(x_current, W, 'rows');
    
    if ~is_member
        fprintf('Error: State [%.2f, %.2f, %.2f] at step %d is not in the winning domain!\n', ...
            x_current(1), x_current(2), x_current(3), k);
        fprintf('This can happen if the winning domain is empty or has "holes". Aborting.\n');
        trajectory = trajectory(1:k, :); % Trim
        break;
    end
    
    % Get the pre-calculated average control for this state
    u_current = U(loc, :);
    
    % 3. Apply dynamics (Single integrator: x_next = x + u * tau)
    x_next_unquantized = x_current + u_current * tau;
    
    % 4. Quantize to the closest state in the grid
    % The controller is defined on the discrete states W. The next
    % state must be snapped to the closest state in W.
    dist_to_w = sum((W - x_next_unquantized).^2, 2);
    [~, idx_next] = min(dist_to_w);
    
    % Update current state for next loop iteration
    x_current = W(idx_next, :);
    trajectory(k+1, :) = x_current;
    
    if k == max_steps
        fprintf('Simulation ended after %d steps without reaching target.\n', max_steps);
    end
end

% ============================================================
% NEW CODE ADDED HERE: EXPORT TRAJECTORY TO CSV
% ============================================================
export_filename = 'simulated_trajectory.csv';
fprintf('Exporting trajectory coordinates to %s...\n', export_filename);

% Convert the numerical trajectory array to a table with headers
T_export = array2table(trajectory, 'VariableNames', {'x', 'y', 'z'});

% Write the table to a CSV file
writetable(T_export, export_filename);
fprintf('Export successful.\n');
% ============================================================

% --- Plot Simulation Results ---
figure;
hold on;
% Plot the target set
plotCuboid(target1_min_pos, target1_size, 'r', 0.1, 'Target Set');
plotCuboid(target2_min_pos, target2_size, 'r', 0.1, 'Target Set');
plotCuboid(target3_min_pos, target3_size, 'r', 0.1, 'Target Set');
% % Plot the obstacle (Gray)
plotCuboid(obstacle1_min_pos, obstacle1_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle2_min_pos, obstacle2_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle3_min_pos, obstacle3_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle4_min_pos, obstacle4_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle5_min_pos, obstacle5_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle6_min_pos, obstacle6_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle7_min_pos, obstacle7_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle8_min_pos, obstacle8_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle9_min_pos, obstacle9_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle10_min_pos, obstacle10_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% plotCuboid(obstacle11_min_pos, obstacle11_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
% Plot the trajectory
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), ...
    'g-o', 'LineWidth', 2, 'MarkerFaceColor', 'g', 'MarkerSize', 4, 'DisplayName', 'Trajectory');
% Plot Start and End points
plot3(x_start(1), x_start(2), x_start(3), ...
    'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'DisplayName', 'Start Point'); % Yellow square
plot3(x_current(1), x_current(2), x_current(3), ...
    'kh', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', 'End Point'); % Magenta hexagram
% Setup plot aesthetics
title('SCOTS Controller: Simulated Trajectory');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
legend('show');
grid on;
axis equal;
view(30, 25);
xlim([-0.7, 0.7]); ylim([-0.7, 0.7]); zlim([0.0, 0.7]);
hold off;
fprintf('Visualization complete.\n');
%% plotCuboid Helper Function
% This function is required to draw 3D boxes for the target and obstacle sets.
function h = plotCuboid(min_pos, size, color, transparency, name)
    % min_pos: [xmin, ymin, zmin]
    % size: [width, height, depth]
    % color: RGB vector or character (e.g., 'r', 'g', [0.5 0.5 0.5])
    % transparency: 0.0 (transparent) to 1.0 (opaque)
    
    % Define the 8 vertices of the cuboid
    x = min_pos(1) + [0 size(1) 0 size(1) 0 size(1) 0 size(1)];
    y = min_pos(2) + [0 0 size(2) size(2) 0 0 size(2) size(2)];
    z = min_pos(3) + [0 0 0 0 size(3) size(3) size(3) size(3)];
    
    % Define the 6 faces (each row is a face, containing 4 vertex indices)
    faces = [1 2 4 3;  % Bottom
             5 6 8 7;  % Top
             1 2 6 5;  % Side 1 (X min)
             3 4 8 7;  % Side 2 (X max)
             1 3 7 5;  % Side 3 (Y min)
             2 4 8 6]; % Side 4 (Y max)
         
    h = patch('Vertices', [x' y' z'], 'Faces', faces, ...
              'FaceColor', color, 'FaceAlpha', transparency, ...
              'EdgeColor', 'k', 'DisplayName', name);
end
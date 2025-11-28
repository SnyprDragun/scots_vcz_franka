clear;
clc;
close all;
csv_filename = 'example_2_T4.csv';
tau = 0.3;

% ============================================================
% TARGETS
% ============================================================
target1_min_pos = [0.5, -0.05, 0.02];
target1_size = [0.1, 0.1, 0.06];
target2_min_pos = [0.35, 0.5, 0.02];
target2_size = [0.1, 0.1, 0.06];
target3_min_pos = [0.35, -0.6, 0.02];
target3_size = [0.1, 0.1, 0.06];
target4_min_pos = [0.3, -0.05, 0.45];
target4_size = [0.05, 0.1, 0.1];

% ============================================================
% OBSTACLES
% ============================================================

% STATE SPACE BOUNDARY
obstacle1_min_pos = [-0.28, -0.28, 0.02];
obstacle1_size = [0.56, 0.56, 0.68];
obstacle2_min_pos = [0.6, -0.7, 0.02];
obstacle2_size = [0.1, 1.4, 0.68];
obstacle3_min_pos = [-0.7, -0.7, 0.02];
obstacle3_size = [0.1, 1.4, 0.68];
obstacle4_min_pos = [-0.6, 0.6, 0.02];
obstacle4_size = [1.2, 0.1, 0.68];
obstacle5_min_pos = [-0.6, -0.7, 0.02];
obstacle5_size = [1.2, 0.1, 0.68];

% INTERNAL BOUNDARY
obstacle6_min_pos = [0.5, -0.6, 0.2];
obstacle6_size = [0.1, 1.2, 0.5];
obstacle7_min_pos = [0.15, -0.6, 0.02];
obstacle7_size = [0.1, 0.32, 0.68];
obstacle8_min_pos = [0.15, 0.28, 0.02];
obstacle8_size = [0.1, 0.32, 0.68];

% OBSTACLES
obstacle9_min_pos = [0.25, 0.15, 0.02];
obstacle9_size = [0.32, 0.2, 0.48];
obstacle10_min_pos = [0.25, -0.35, 0.02];
obstacle10_size = [0.32, 0.2, 0.48];
% ============================================================

fprintf('Loading controller data from %s...\n', csv_filename);
try
    data_table = readtable(csv_filename);
    data = data_table{:,:};
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

[W, ~, ic] = unique(states, 'rows');
num_winning_states = size(W, 1);
if num_winning_states == 0
    fprintf('No winning states found in the file. Cannot visualize.\n');
    return;
end
fprintf('Found %d unique winning states.\n', num_winning_states);
U = zeros(num_winning_states, 3);
for i = 1:num_winning_states
    controls_for_this_state = inputs(ic == i, :);
    U(i, :) = mean(controls_for_this_state, 1);
end
fprintf('Controller data processed.\n');
fprintf('Displaying static controller plot...\n');
figure;
hold on;

plotCuboid(target1_min_pos, target1_size, 'r', 0.1, 'Target Set');
plotCuboid(target2_min_pos, target2_size, 'r', 0.1, 'Target Set');
plotCuboid(target3_min_pos, target3_size, 'r', 0.1, 'Target Set');
plotCuboid(target4_min_pos, target4_size, 'r', 0.1, 'Target Set');

plotCuboid(obstacle1_min_pos, obstacle1_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle2_min_pos, obstacle2_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle3_min_pos, obstacle3_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle4_min_pos, obstacle4_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle5_min_pos, obstacle5_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle6_min_pos, obstacle6_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle7_min_pos, obstacle7_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle8_min_pos, obstacle8_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle9_min_pos, obstacle9_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle10_min_pos, obstacle10_size, [0.5 0.5 0.5], 0.8, 'Obstacle');

scatter3(W(:,1), W(:,2), W(:,3), ...
    15, 'b', 'filled', 'MarkerFaceAlpha', 0.2);
quiver3(W(:,1), W(:,2), W(:,3), ...
    U(:,1), U(:,2), U(:,3), 0.5, 'k');
title('SCOTS Controller: Winning Domain (blue), Target (red), and Obstacle (gray)');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

h_scatter = findobj(gca, 'Type', 'scatter');
h_quiver = findobj(gca, 'Type', 'quiver');
h_target = findobj(gca, 'DisplayName', 'Target Set');
h_obstacle = findobj(gca, 'DisplayName', 'Obstacle');
legend([h_target(1), h_obstacle(1), h_scatter(1), h_quiver(1)], ...
       'Target Set', 'Obstacle', 'Winning Domain', 'Control Vectors', 'Location', 'best');
grid on;
axis equal;
view(30, 25);
xlim([-0.7, 0.7]); ylim([-0.7, 0.7]); zlim([0.0, 0.7]);
hold off;

fprintf('Simulating trajectory...\n');


%======== TARGET 1========%
% target1_center = target1_min_pos + target1_size / 2;
% distances = sum((W - target1_center).^2, 2);
%======== TARGET 2========%
% target2_center = target2_min_pos + target2_size / 2;
% distances = sum((W - target2_center).^2, 2);
%======== TARGET 3========%
% target3_center = target3_min_pos + target3_size / 2;
% distances = sum((W - target3_center).^2, 2);
%======== TARGET 4========%
target4_center = target4_min_pos + target4_size / 2;
distances = sum((W - target4_center).^2, 2);

[~, idx_start] = max(distances);


%======== PATH: T4 -> T2 -> T1 -> T3 -> T4 ========%
% x_current = [0.3, 0.0, 0.48]; %==== [0.307, 0.000, 0.487] ====%
% x_current = [0.34,0.54,0.06];
% x_current = [0.54,-0.02,0.02];
x_current = [0.34,-0.58,0.06];

max_steps = 20;
trajectory = zeros(max_steps + 1, 3);
trajectory(1, :) = x_current;
x_start = x_current;
fprintf('Starting simulation from [x=%.2f, y=%.2f, z=%.2f]\n', ...
    x_current(1), x_current(2), x_current(3));

for k = 1:max_steps
    % if all(x_current >= target1_min_pos) && all(x_current <= (target1_min_pos + target1_size))
    % if all(x_current >= target2_min_pos) && all(x_current <= (target2_min_pos + target2_size))
    % if all(x_current >= target3_min_pos) && all(x_current <= (target3_min_pos + target3_size))
    if all(x_current >= target4_min_pos) && all(x_current <= (target4_min_pos + target4_size))
        fprintf('Reached target set in %d steps!\n', k-1);
        trajectory = trajectory(1:k, :); 
        break;
    end

    [is_member, loc] = ismember(x_current, W, 'rows');
    
    if ~is_member
        fprintf('Error: State [%.2f, %.2f, %.2f] at step %d is not in the winning domain!\n', ...
            x_current(1), x_current(2), x_current(3), k);
        fprintf('This can happen if the winning domain is empty or has "holes". Aborting.\n');
        trajectory = trajectory(1:k, :);
        break;
    end

    u_current = U(loc, :);
    x_next_unquantized = x_current + u_current * tau;
    dist_to_w = sum((W - x_next_unquantized).^2, 2);
    [~, idx_next] = min(dist_to_w);

    x_current = W(idx_next, :);
    trajectory(k+1, :) = x_current;
    
    if k == max_steps
        fprintf('Simulation ended after %d steps without reaching target.\n', max_steps);
    end
end

% ============================================================
% EXPORT TRAJECTORY TO CSV
% ============================================================
export_filename = 'example_2_T4_trajectory.csv';
fprintf('Exporting trajectory coordinates to %s...\n', export_filename);
T_export = array2table(trajectory, 'VariableNames', {'x', 'y', 'z'});
writetable(T_export, export_filename);
fprintf('Export successful.\n');
% ============================================================

figure;
hold on;
plotCuboid(target1_min_pos, target1_size, 'r', 0.1, 'Target Set');
plotCuboid(target2_min_pos, target2_size, 'r', 0.1, 'Target Set');
plotCuboid(target3_min_pos, target3_size, 'r', 0.1, 'Target Set');
plotCuboid(target4_min_pos, target4_size, 'r', 0.1, 'Target Set');

plotCuboid(obstacle1_min_pos, obstacle1_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle2_min_pos, obstacle2_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle3_min_pos, obstacle3_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle4_min_pos, obstacle4_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle5_min_pos, obstacle5_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle6_min_pos, obstacle6_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle7_min_pos, obstacle7_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle8_min_pos, obstacle8_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle9_min_pos, obstacle9_size, [0.5 0.5 0.5], 0.8, 'Obstacle');
plotCuboid(obstacle10_min_pos, obstacle10_size, [0.5 0.5 0.5], 0.8, 'Obstacle');

plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), ...
    'g-o', 'LineWidth', 2, 'MarkerFaceColor', 'g', 'MarkerSize', 4, 'DisplayName', 'Trajectory');
plot3(x_start(1), x_start(2), x_start(3), ...
    'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'DisplayName', 'Start Point');
plot3(x_current(1), x_current(2), x_current(3), ...
    'kh', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', 'End Point');
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

% ============================================================
% FUNCTION
% ============================================================
function h = plotCuboid(min_pos, size, color, transparency, name)

    x = min_pos(1) + [0 size(1) 0 size(1) 0 size(1) 0 size(1)];
    y = min_pos(2) + [0 0 size(2) size(2) 0 0 size(2) size(2)];
    z = min_pos(3) + [0 0 0 0 size(3) size(3) size(3) size(3)];

    faces = [1 2 4 3;
             5 6 8 7;
             1 2 6 5;
             3 4 8 7;
             1 3 7 5;
             2 4 8 6];
         
    h = patch('Vertices', [x' y' z'], 'Faces', faces, ...
              'FaceColor', color, 'FaceAlpha', transparency, ...
              'EdgeColor', 'k', 'DisplayName', name);
end
% ============================================================
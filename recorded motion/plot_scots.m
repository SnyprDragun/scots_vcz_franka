% PLOT_ROBOT_DATA.M
%
% This script reads data from a CSV file with robot joint measurements
% and plots the state and torque for each of the 7 joints against
% the elapsed time in a 7x2 subplot layout.
%
% FIXES IN THIS VERSION:
% - Uses 'VariableNamingRule', 'preserve' in readtable to match column
%   names exactly, fixing the "Column not found" warnings.
% - Data access uses the syntax T{:, 'Column Name'} to handle spaces in headers.
%
% TO RUN THIS SCRIPT:
% 1. Ensure 'robot_data.csv' is in the same directory.
% 2. Open the MATLAB Command Window and type: plot_robot_data

% --- 1. Configuration ---
filename = 'uncertain_case_1_joint_ee.csv'; % Change this if your file name is different

% --- 2. Data Loading and Validation ---
if exist(filename, 'file') ~= 2
    error('File not found: %s. Please ensure the file exists in the current MATLAB directory.', filename);
end

try
    % Read the CSV file into a table. We use VariableNamingRule='preserve'
    % to keep the original headers (e.g., 'joint_1 state'), which is necessary
    % for robustness and directly addresses the reported warning.
    data = readtable(filename, 'VariableNamingRule', 'preserve');
catch ME
    error('Failed to read the CSV file. Error: %s', ME.message);
end

% Extract the time vector. We can safely use dot notation for 'elapsed_time'
% as it's a valid and unique variable name without spaces.
elapsed_time = data.elapsed_time;

% Define the number of joints to plot (1 to 7)
num_joints = 7;

% --- 3. Plotting Setup ---
figure('Name', 'Robot Joint Variables vs. Time', 'NumberTitle', 'on');
set(gcf, 'Position', [100, 50, 1000, 900]); % Set figure size for better visibility

% --- 4. Loop through joints and create subplots ---
for i = 1:num_joints
    % Generate column names dynamically matching the *preserved* header format
    joint_state_col = sprintf('joint_%d state', i);
    joint_torque_col = sprintf('joint_%d torque', i);

    % --- Subplot for Joint State (Left Column) ---
    % Position calculation: (i-1)*2 + 1 -> 1, 3, 5, 7, 9, 11, 13
    subplot(num_joints, 2, (i-1)*2 + 1);
    
    % Check if column exists before plotting
    if ismember(joint_state_col, data.Properties.VariableNames)
        % Access data using cell-array indexing {:, column_name} for preserved names
        joint_state_data = data{:, joint_state_col};
        plot(elapsed_time, joint_state_data, 'b', 'LineWidth', 1.5);
        
        % Add titles and labels
        title(sprintf('Joint %d State', i), 'FontSize', 10);
        ylabel('State (Rad/Deg)', 'FontSize', 9);
        grid on;
        
        % Only label the x-axis for the last plot in the left column
        if i == num_joints
            xlabel('Elapsed Time (s)', 'FontSize', 10);
        end
    else
        warning('Column not found: %s', joint_state_col);
    end

    % --- Subplot for Joint Torque (Right Column) ---
    % Position calculation: (i-1)*2 + 2 -> 2, 4, 6, 8, 10, 12, 14
    subplot(num_joints, 2, (i-1)*2 + 2);
    
    % Check if column exists before plotting
    if ismember(joint_torque_col, data.Properties.VariableNames)
        % Access data using cell-array indexing {:, column_name} for preserved names
        joint_torque_data = data{:, joint_torque_col};
        plot(elapsed_time, joint_torque_data, 'r', 'LineWidth', 1.5);
        
        % Add titles and labels
        title(sprintf('Joint %d Torque', i), 'FontSize', 10);
        ylabel('Torque (Nm)', 'FontSize', 9);
        grid on;
        
        % Only label the x-axis for the last plot in the right column
        if i == num_joints
            xlabel('Elapsed Time (s)', 'FontSize', 10);
        end
    else
        warning('Column not found: %s', joint_torque_col);
    end
end

% --- 5. Final Adjustments ---
% Optional: Adjust the spacing between subplots
sgtitle('Robot Joint Motion Analysis (State vs. Torque)', 'FontSize', 14, 'FontWeight', 'bold');

disp('Plotting complete. Check the generated figure window.');
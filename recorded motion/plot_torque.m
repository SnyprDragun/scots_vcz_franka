% PLOT_TORQUE_DATA.M
%
% This script reads a CSV file containing 7 torque measurements (tau_1 to tau_7)
% and plots them against the timestamp in a 7x1 subplot layout.
%
% CHANGES:
% - Uses 'readmatrix' to import data directly into a numeric matrix.
% - Uses matrix indexing (data(:, index)) instead of table column names.
% - Updated default filename to 'torque_log.csv'.
%
% TO RUN THIS SCRIPT:
% 1. Ensure 'torque_log.csv' is in the same directory.
% 2. Open the MATLAB Command Window and type: plot_torque_data

% --- 1. Configuration ---
filename = 'uncertain_case_1_effort.csv'; % Updated filename
num_torques = 7;
% We no longer need 'time_col_name' as we use column index 1

% --- 2. Data Loading and Validation ---
if exist(filename, 'file') ~= 2
    error('File not found: %s. Please ensure the file exists in the current MATLAB directory.', filename);
end

try
    % Read the CSV file directly into a numeric matrix.
    % 'NumHeaderLines', 1 tells MATLAB to skip the first row (the headers).
    data = readmatrix(filename, 'NumHeaderLines', 1);
catch ME
    error('Failed to read the CSV file as a matrix. Error: %s', ME.message);
end

% Basic validation: Check if we have enough columns (Time + 7 Torques = 8 columns)
expected_cols = num_torques + 1;
if size(data, 2) < expected_cols
    error('The CSV data has %d columns, but %d columns were expected (Time + 7 Torques).', size(data, 2), expected_cols);
end

% Extract the time vector (Column 1 of the matrix)
elapsed_time = data(:, 1);


% --- 3. Plotting Setup ---
figure('Name', 'Joint Torque Analysis vs. Time', 'NumberTitle', 'on');
set(gcf, 'Position', [150, 50, 700, 900]); % Set figure size for tall, single-column plots

% --- 4. Loop through torques and create subplots (7x1 layout) ---
for i = 1:num_torques
    % The torque data columns start after the time column (Column 2 is tau_1, Column 3 is tau_2, etc.)
    torque_col_index = i + 1; 

    % Create subplot in the 7x1 grid
    subplot(num_torques, 1, i);
    
    % Access data using matrix indexing (all rows, specific column)
    torque_data = data(:, torque_col_index);
    
    plot(elapsed_time, torque_data, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5); % Custom reddish-orange color
    
    % Add titles and labels
    title(sprintf('Joint %d Torque (\\tau_{%d})', i, i), 'FontSize', 10);
    ylabel('Torque (Nm)', 'FontSize', 9);
    grid on;
    
    % Only label the x-axis for the last subplot
    if i == num_torques
        xlabel('Time (s)', 'FontSize', 10);
    end
end

% --- 5. Final Adjustments ---
sgtitle('Individual Joint Torque Profiles', 'FontSize', 14, 'FontWeight', 'bold');

disp('Torque plotting script complete. Using "readmatrix" to load data.');
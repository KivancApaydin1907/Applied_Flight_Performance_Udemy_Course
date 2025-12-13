%% Processing Drag Polar Data (K Factor)
% Description: This script handles inconsistent raw data sources for the
% Drag-Due-to-Lift factor (K). It harmonizes different Mach ranges into a
% common grid using safe extrapolation and creates a structured database.

clear; clc; close all;

%% 1. Load Existing Database (Prevent Overwrite)
% ******************************
data_filename = 'T38_Aero_Data.mat';

if isfile(data_filename)
    load(data_filename);
    fprintf('Existing database loaded: %s. Appending Drag data...\n', data_filename);
else
    % If the file doesn't exist, initialize a new structure to avoid errors later
    T38_Aerodynamics = struct();
    warning('No existing database found. Creating a new structure.');
end

%% 2. Load Raw Data & Visualization
% ******************************
% List of raw data files (assumed to be in the path)
file_list = {'CL035.mat', 'CL04.mat', 'CL05.mat', 'CL06.mat', 'CL07.mat'};
legend_labels = {'CL <= 0.35', 'CL = 0.4', 'CL = 0.5', 'CL = 0.6', 'CL = 0.7'};
% Pre-allocate a cell array to store raw matrices. 
% We use a 'cell array' because raw matrices might have different row counts.
raw_data = cell(1, length(file_list)); 

figure('Name', 'K Factor Raw Data Visualization', 'Color', 'w');
hold on; grid on; grid minor;
title('Drag-Due-to-Lift Factor (K) - Raw Data');
xlabel('Mach Number'); 
ylabel('K Factor'); % Typo fixed here

% Load and plot in a loop 
for i = 1:length(file_list)
    if isfile(file_list{i})
        % Load file into a temporary structure
        temp = load(file_list{i});
        % Dynamic Field Access:
        % The variable name inside the .mat file matches the filename (minus .mat).
        % We construct the name dynamically to extract the data.
        varName = file_list{i}(1:end-4); 
        raw_data{i} = temp.(varName); 
        
        plot(raw_data{i}(:,1), raw_data{i}(:,2), 'LineWidth', 1.5, 'DisplayName', legend_labels{i});
    else
        warning('File %s not found. Skipping...', file_list{i});
    end
end
legend show;

%% 3. Data Harmonization (The Core Logic)
% ******************************
% Problem: Raw data vectors have different Mach start/end points.
% Solution: Interpolate to a common grid and fill missing ends (Nearest Neighbor).

Mach_common = 0:0.01:1.2; % Common Mach Grid
CL_vector   = [0.35; 0.4; 0.5; 0.6; 0.7]; 

% Helper Function: Linear Interpolation + Nearest Extrapolation
% 1. interp1(..., 'linear', NaN): Interpolates linearly inside the range, returns NaN outside.
% 2. fillmissing(..., 'nearest'): Replaces those NaNs with the nearest valid value.
% This effectively creates a "flat extrapolation" preventing wild oscillations.
safe_interp = @(x, y, xq) fillmissing(interp1(x, y, xq, 'linear', NaN), 'nearest');

% Pre-allocate matrix for speed
n_mach = length(Mach_common);
n_cl   = length(CL_vector);
lookup_drag_due_to_lift = zeros(n_cl, n_mach);

% Process each dataset into the matrix rows
for i = 1:n_cl
    % Guard Clause: Check if the raw data actually exists for this index.
    % "~isempty" means "if not empty". This prevents crashing if a file was missing.
    if ~isempty(raw_data{i}) 
        % Apply the safe interpolation and store in the i-th row
        lookup_drag_due_to_lift(i, :) = safe_interp(raw_data{i}(:, 1), raw_data{i}(:, 2), Mach_common);
    end
end

%% 4. Verification (Example Query)
% ******************************
CL_current   = 0.1; 
Mach_current = 0.1;

% Logic: If CL is below 0.35, clamp it to 0.35 (Since K is constant for low CL)
cl_query = max(min(CL_vector), CL_current); 

K_factor = interp2(Mach_common, CL_vector, lookup_drag_due_to_lift, Mach_current, cl_query, 'linear');

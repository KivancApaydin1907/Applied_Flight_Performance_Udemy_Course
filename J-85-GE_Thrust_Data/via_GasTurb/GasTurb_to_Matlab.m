%% GASTURB TO MATLAB IMPORTER (BATCH PROCESSING)
% =========================================================================
%  Script Name:   GasTurb_to_Matlab.m
%  Author:        Kivanc Apaydin
%  Date:          2025
%  Description:   
%     This script reads multi-block Excel exports from GasTurb 15.
%     It processes "Dry" (Military) and "Wet" (Afterburner) data separately,
%     merges the data blocks, and generates structured 3D lookup tables for
%     Flight Dynamics simulations.
%
%  Inputs:
%     - J85_Reheat_Off.xlsx : Dry Power Data
%     - J85_Reheat.xlsx     : Wet Power Data
%
%  Outputs:
%     - J85_Engine_Performance.mat : Structured data for Simulink/MATLAB
% =========================================================================
clear; clc; close all;

%% 1. CONFIGURATION
% Define input files. Make sure these exist in the current folder.
files = struct();
files.Dry = 'J85_Reheat_Off.xlsx';  % Dry (Military) Data
files.Wet = 'J85_Reheat.xlsx';       % Wet (Afterburner) Data

% Define output filename
output_mat = 'J85_Engine_Performance.mat';

%% 2. MAIN PROCESSING LOOP
% Loop through the 'files' structure to process Dry and Wet modes separately
modes = fieldnames(files); % {'Dry', 'Wet'}
EngineData = struct();

fprintf('================================================\n');
fprintf('   GASTURB DATA PROCESSOR STARTED\n');
fprintf('================================================\n');

for i = 1:length(modes)
    mode = modes{i};
    filename = files.(mode);
    
    fprintf('>> Processing Mode: %-5s | File: %s\n', mode, filename);
    
    % Check if file exists before calling function
    if ~isfile(filename)
        error('Error: File "%s" not found in the current directory.', filename);
    end
    
    % Call the local function to read and process the file
    [M_vec, h_vec, Thrust_Map, SFC_Map] = process_gasturb_file(filename);
    
    % Store in the main structure
    EngineData.(mode).Mach     = M_vec;
    EngineData.(mode).Altitude = h_vec;
    EngineData.(mode).Thrust   = Thrust_Map; % [kN]
    EngineData.(mode).SFC      = SFC_Map;    % [g/(kN*s)] (Raw GasTurb Unit)
end

%% 3. SAVE DATA
% Saving as a struct makes it easy to load in Simulink/MATLAB later
fprintf('------------------------------------------------\n');
fprintf('>> Saving processed data to "%s"...\n', output_mat);
save(output_mat, 'EngineData');
fprintf('>> SUCCESS: Data processing complete.\n');

%% 4. VISUALIZATION (QUALITY CHECK)
fprintf('>> Generating Verification Plots...\n');

% Figure 1: 3D Surface Plots Comparison
fig1 = figure('Name', 'J85 Thrust Envelopes', 'Color', 'w', 'Position', [100, 100, 1200, 500]);

% Subplot 1: Dry Thrust
subplot(1, 2, 1);
surf(EngineData.Dry.Mach, EngineData.Dry.Altitude/1000, EngineData.Dry.Thrust);
title('DRY Thrust (Reheat OFF)', 'FontSize', 12);
xlabel('Mach Number'); ylabel('Altitude (km)'); zlabel('Thrust (kN)');
shading interp; colorbar; view(145, 30); grid on;

% Set color limits to match Wet data for visual comparison
max_thrust = max(EngineData.Wet.Thrust(:));
clim([0 max_thrust]); 

% Subplot 2: Wet Thrust
subplot(1, 2, 2);
surf(EngineData.Wet.Mach, EngineData.Wet.Altitude/1000, EngineData.Wet.Thrust);
title('WET Thrust (Reheat ON)', 'FontSize', 12);
xlabel('Mach Number'); ylabel('Altitude (km)'); zlabel('Thrust (kN)');
shading interp; colorbar; view(145, 30); grid on;
clim([0 max_thrust]); 

fprintf('>> Done.\n');

%% =========================================================================
% LOCAL FUNCTION: READ AND PARSE GASTURB FILE
% =========================================================================
function [M_Grid, h_Grid, Thrust_Map, SFC_Map] = process_gasturb_file(filename)
    % 1. Read Raw Data
    try
        raw_data = readcell(filename);
    catch ME
        error('Failed to read excel file. Ensure it is not open in Excel.\nError: %s', ME.message);
    end
    
    row_headers = string(raw_data(:,1));
    
    % 2. Find Index Blocks (Robust Search)
    idx_Alt_List    = find(contains(row_headers, 'Altitude', 'IgnoreCase', true));
    idx_Mach_List   = find(contains(row_headers, 'Mach Number', 'IgnoreCase', true));
    idx_Thrust_List = find(contains(row_headers, 'Net Thrust', 'IgnoreCase', true));
    idx_SFC_Source_List = find(contains(row_headers, 'Sp. Fuel Consumption', 'IgnoreCase', true));
    
    n_blocks = length(idx_Alt_List);
    if n_blocks == 0
        error('No data blocks found in %s. Check Excel format.', filename);
    end
    
    % 3. Loop and Merge Data Blocks
    All_Alt     = [];
    All_Mach    = [];
    All_Thrust  = [];
    All_SFC_Raw = [];
    
    start_col = 3; % GasTurb data usually starts at column 3
    
    for k = 1:n_blocks
        r_alt    = idx_Alt_List(k);
        
        % Find the specific rows belonging to THIS altitude block
        r_mach   = idx_Mach_List(find(idx_Mach_List > r_alt, 1));
        r_thrust = idx_Thrust_List(find(idx_Thrust_List > r_alt, 1));
        r_sfc    = idx_SFC_Source_List(find(idx_SFC_Source_List > r_alt, 1));
        
        vals_alt    = cell2mat(raw_data(r_alt, start_col:end));
        vals_mach   = cell2mat(raw_data(r_mach, start_col:end));
        vals_thrust = cell2mat(raw_data(r_thrust, start_col:end));
        vals_sfc    = cell2mat(raw_data(r_sfc, start_col:end));
        
        % Handle potential empty cells or mismatches at end of rows
        min_len = min([length(vals_mach), length(vals_thrust), length(vals_sfc)]);
        
        % If Altitude is a single value, repeat it to match array length
        if length(vals_alt) < min_len
            vals_alt = repmat(vals_alt(1), 1, min_len);
        end
        
        All_Alt     = [All_Alt,    vals_alt(1:min_len)];
        All_Mach    = [All_Mach,   vals_mach(1:min_len)];
        All_Thrust  = [All_Thrust, vals_thrust(1:min_len)];
        All_SFC_Raw = [All_SFC_Raw, vals_sfc(1:min_len)];
    end
    
    % 4. Gridding (Scattered -> Grid)
    % Create a structured meshgrid for look-up tables
    h_vec_unique = unique(All_Alt);
    M_vec_unique = unique(All_Mach);
    
    [M_Grid, h_Grid] = meshgrid(M_vec_unique, h_vec_unique);
    
    % Interpolants
    % Use 'nearest' for extrapolation to avoid wild values outside envelope
    F_Thrust = scatteredInterpolant(All_Mach', All_Alt', All_Thrust', 'linear', 'nearest');
    F_SFC    = scatteredInterpolant(All_Mach', All_Alt', All_SFC_Raw', 'linear', 'nearest');
    
    Thrust_Map = F_Thrust(M_Grid, h_Grid);
    SFC_Map = F_SFC(M_Grid, h_Grid);
end
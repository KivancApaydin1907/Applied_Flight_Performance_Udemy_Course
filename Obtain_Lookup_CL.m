%% Reading Data from OpenVSP and Generating Lookup Table
% Description: This script imports aerodynamic coefficients from OpenVSP 
% text output, reshapes the data into a structured grid, and visualizes 
% the C_L surface. Finally, it stores the processed data into a struct.

clear; clc;

%% 1. Import and Process Data
% ******************************
filename = 'T38_OpenVSP_CFD_Results.txt';

% Check if file exists to prevent errors
if ~isfile(filename)
    error('File %s not found. Please ensure the data file is in the directory.', filename);
end

data = readtable(filename); % Read the aerodynamic data
raw_CL = data.CLtot;        % Extract Lift Coefficient (CL) column

% Define the simulation grid parameters (Based on OpenVSP Setup)
alpha_vector = 0:1:18;      % Angle of Attack range (deg)
mach_vector  = 0.2:0.2:0.8; % Mach number range

n_alpha = length(alpha_vector); 
n_mach  = length(mach_vector);

% Reshape linear data into a Matrix (Rows: Alpha, Cols: Mach)
% Note: Ensure OpenVSP export order matches this reshape logic.
Lookup_CL = reshape(raw_CL, n_alpha, n_mach); 

%% 2. Visualization
% ******************************
figure('Name', 'T-38 Aerodynamic Database Visualization', 'Color', 'w');

% PLOT FIX: Use vectors for axes to show real physical values, not indices.
s = surf(mach_vector, alpha_vector, Lookup_CL); 

s.FaceColor = 'interp'; % Smooth color transition
s.EdgeColor = 'none';   % Remove grid lines for cleaner look
colorbar;

% Labeling with units
xlabel('Mach Number');
ylabel('Angle of Attack (deg)');
zlabel('Lift Coefficient (C_L)');
title('T-38 Lift Coefficient Distribution (C_L - \alpha - Mach)');

% Esthetic adjustments
grid on; grid minor;
view(3); % Set default 3D view
axis tight; % Fit axes to data limits

%% 3. Save to Structured Data (Database Update)
% ******************************
data_filename = 'T38_Aero_Data.mat';

if isfile(data_filename)
    load(data_filename);
    fprintf('Existing database loaded: %s\n', data_filename);
else
    T38_Aerodynamics = struct();
    fprintf('New database created.\n');
end

% Storing in a categorized structure for easy access in simulation
T38_Aerodynamics.Lift.CL_Matrix    = Lookup_CL;
T38_Aerodynamics.Lift.Alpha_Vector = alpha_vector;
T38_Aerodynamics.Lift.Mach_Vector  = mach_vector;
T38_Aerodynamics.Lift.Description  = 'OpenVSP CFD Results. Reshaped from raw text output.';

% Metadata for version control
T38_Aerodynamics.Meta.LastUpdate   = datetime("now");
T38_Aerodynamics.Meta.Author       = 'Kivanc'; 

save(data_filename, 'T38_Aerodynamics');
fprintf('Aerodynamic data successfully saved to %s\n', data_filename);
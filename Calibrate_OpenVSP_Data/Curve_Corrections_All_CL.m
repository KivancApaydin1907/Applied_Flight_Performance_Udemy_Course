%% 1. INITIALIZATION & DATA LOADING
clear; clc; close all;

% Load Experimental Data (Only Mach 0.4 exists)
load('mach04.mat');

% Load OpenVSP Results
data = readtable('T38_OpenVSP_CFD_Results.txt');
CL_raw = data.CLtot;

%% 2. DATA PROCESSING
n_alpha = 19; 
n_mach  = height(data) / n_alpha; 
CL_Grid = reshape(CL_raw, n_alpha, n_mach);

% Create Angle of Attack Vector (Column)
Alpha_Vec = linspace(0, 18, n_alpha)';

% Define Mach numbers for the columns (Assuming order 0.2, 0.4, 0.6, 0.8)
Mach_List = [0.2, 0.4, 0.6, 0.8]; 

%% 3. CALIBRATION (Finding the "Rule" from Mach 0.4)
fprintf('--- Calibrating Model using Mach 0.4 Data ---\n');

% Extract the Mach 0.4 VSP data (Column 2)
CL_vsp_m04 = CL_Grid(:, 2); 

% Interpolate Experimental Data to match Alpha_Vec
CL_exp_interp = interp1(mach04(:,1), mach04(:,2), Alpha_Vec, 'linear', 'extrap');

% Optimization to find best k and trigger
cost_function = @(p) calculate_error(p, Alpha_Vec, CL_vsp_m04, CL_exp_interp);
initial_guess = [0.003, 9]; 
options = optimset('Display', 'off', 'TolX', 1e-6); % Turn off iteration display

best_params = fminsearch(cost_function, initial_guess, options);

% SAVE THE "UNIVERSAL RULE"
k_universal = best_params(1);
trigger_universal = best_params(2);

fprintf('Calibration Complete.\n');
fprintf('  > Universal Trigger Angle: %.2f deg\n', trigger_universal);
fprintf('  > Universal Stiffness (k): %.6f\n', k_universal);

%% 4. EXTRAPOLATION (Applying the Rule to All Machs)
% We will now create a new matrix with corrected values
CL_Grid_Corrected = zeros(size(CL_Grid));

% Loop through all Mach columns (0.2, 0.4, 0.6, 0.8)
for i = 1:n_mach
    % 1. Get linear VSP data
    CL_linear = CL_Grid(:, i);
    
    % 2. Calculate Penalty using the UNIVERSAL constants
    penalty = zeros(size(Alpha_Vec));
    mask = Alpha_Vec > trigger_universal;
    penalty(mask) = k_universal * (Alpha_Vec(mask) - trigger_universal).^2;
    
    % 3. Apply Correction
    CL_Grid_Corrected(:, i) = CL_linear - penalty;
end

%% 5. VISUALIZATION 
figure('Name', 'Mach 0.4 Stall Analysis', 'Color', 'w');
hold on; grid on;

% Plot 1: Original OpenVSP Data (Cyan)
p1 = plot(Alpha_Vec, CL_vsp_m04, 'c', 'LineWidth', 1.5);

% Plot 2: Experimental Data (Blue)
p2 = plot(mach04(:,1), mach04(:,2), 'b', 'LineWidth', 1.5);

% Plot 3: Corrected Model (Green Dashed)
p3 = plot(Alpha_Vec, CL_Grid_Corrected(:, 2), 'r--', 'LineWidth', 2);

% Annotations
xlabel('Angle of Attack (\alpha) [deg]', 'FontSize', 11);
ylabel('Lift Coefficient (C_L)', 'FontSize', 11);
title('T-38 Talon: Lift Coefficient vs AoA (Mach 0.4)', 'FontSize', 12);

% Add a visual marker for the trigger point
xline(trigger_universal, 'k:', 'Label', 'Linearity Shift Point');

% Legend & Formatting
legend([p1, p2, p3], ...
       {'OpenVSP (Inviscid/Linear)', ...
        'Experimental Data (Real)', ...
        'Corrected Model (Viscous Approx)'}, ...
       'Location', 'southeast');

hold off;

%% 6. 3D VISUALIZATION (The "Carpet Plot")
figure('Name', '3D Aerodynamic Model', 'Color', 'w');

% 1. Plot the Surface
% X = Mach, Y = Alpha, Z = CL (Corrected)
s = surf(Mach_List, Alpha_Vec, CL_Grid_Corrected);

% 2. Make it look "Pro" (Styling)
s.FaceColor = 'interp';    % Smooths the colors (gradients)
s.EdgeColor = 'none';      % Removes the black wireframe lines

% 3. Color and Axes
c = colorbar;
c.Label.String = 'Lift Coefficient (C_L)';
c.Label.FontSize = 11;

% 4. Labels
xlabel('Mach Number', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angle of Attack (deg)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Lift Coefficient (C_L)', 'FontSize', 12, 'FontWeight', 'bold');
title('T-38 Calibrated C_L Matrix', 'FontSize', 14);

% 5. Set the Camera View
view(3); 
grid on; grid minor;
axis tight; % Fit axes to data limits

%% 7. Save to Structured Data (Database Update)
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
T38_Aerodynamics.Lift.CL_Matrix    = CL_Grid_Corrected;
T38_Aerodynamics.Lift.Alpha_Vector = Alpha_Vec;
T38_Aerodynamics.Lift.Mach_Vector  = Mach_List;
T38_Aerodynamics.Lift.Description  = 'OpenVSP "Corrected" CFD Results. Reshaped from raw text output.';

% Metadata for version control
T38_Aerodynamics.Meta.LastUpdate   = datetime("now");
T38_Aerodynamics.Meta.Author       = 'Kivanc'; 

save(data_filename, 'T38_Aerodynamics');
fprintf('Aerodynamic data successfully saved to %s\n', data_filename);


% --- HELPER FUNCTION ---
function total_error = calculate_error(params, alpha, cl_vsp, cl_target)
    k = params(1);
    trig = params(2);
    
    % Penalty Logic
    penalty = zeros(size(alpha));
    mask = alpha > trig;
    penalty(mask) = k * (alpha(mask) - trig).^2;
    
    cl_model = cl_vsp - penalty;
    
    % Error Calculation
    error_vector = (cl_target - cl_model);
    total_error = sum(error_vector.^2);
end
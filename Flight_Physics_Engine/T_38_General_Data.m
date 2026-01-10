function Airplane = T_38_General_Data()
% T_38_GENERAL_DATA - Aircraft Configuration Generator for Northrop T-38 Talon
%
%   Output:
%       Airplane (struct): A structure containing all necessary physical,
%                          aerodynamic, and propulsion data for simulation.
%
%   Dependencies:
%       - T38_Aero_Data.mat: Aerodynamic lookup tables.
%       - J85_Engine_Performance.mat: Propulsion cycle data (Dry & Wet).
%
%   Units:
%       SI System (kg, m, s, N, rad) is used throughout the structure.
%
%   Author: Kivanc Apaydin
%   Date:   2025

    % Initialize the main structure
    Airplane.Name = 'Northrop T-38 Talon';

    %% 1. GEOMETRY & MASS PROPERTIES
    % Basic geometric reference values used for coefficient denormalization
    Airplane.Geo = struct();
    Airplane.Geo.S_ref = 15.8;      % Wing Reference Area [m^2]
    Airplane.Geo.c_bar = 2.36;      % Mean Aerodynamic Chord [m]
    
    % Mass and Inertia (T-38 Configuration)
    Airplane.Mass = struct();
    Airplane.Mass.Initial = 4536;   % Takeoff Mass [kg]
    Airplane.Mass.Iyy     = 35080;  % Pitch Moment of Inertia [kg*m^2]

    %% 2. AERODYNAMICS (LOOKUP TABLES)
    % Load aerodynamic coefficients generated from CFD/OpenVSP
    if ~isfile('T38_Aero_Data.mat')
        error('Error: "T38_Aero_Data.mat" not found. Please run the Aero Database script first.');
    end
    
    raw_aero = load('T38_Aero_Data.mat'); 
    aero = raw_aero.T38_Aerodynamics;
    
    % Create Gridded Interpolants for Fast Lookup (Linear Interpolation)
    % F_CL = f(Alpha, Mach)
    Airplane.Aero.F_CL  = griddedInterpolant({aero.Lift.Alpha_Vector, aero.Lift.Mach_Vector}, ...
                                             aero.Lift.CL_Matrix, 'linear', 'nearest');
    
    % F_K (Drag Polar Factor) = f(CL, Mach)
    Airplane.Aero.F_K   = griddedInterpolant({aero.Drag.CL_Vector, aero.Drag.Mach_Vector}, ...
                                             aero.Drag.K_Matrix, 'linear', 'nearest');
    
    % F_Cd0 (Zero-Lift Drag) = f(Mach)
    Airplane.Aero.F_Cd0 = griddedInterpolant(aero.Drag.Cd0_MachVector, ...
                                             aero.Drag.Cd0_Vector, 'linear', 'nearest');
    
    % Stability Derivatives
    Airplane.Aero.Cm0      = aero.Moment.Cm0;
    Airplane.Aero.Cm_alpha = aero.Moment.Cm_alpha;
    Airplane.Aero.Cm_delta = aero.Moment.Cm_delta;
    Airplane.Aero.CL_delta = aero.Lift.CL_delta;
    
    % Operational Limits (For Stall Logic)
    Airplane.Aero.Limits.CL_Max_Vec = max(aero.Lift.CL_Matrix, [], 1);
    Airplane.Aero.Limits.Mach_Vec   = aero.Lift.Mach_Vector;

    %% 3. PROPULSION SYSTEM (J85-GE-5)
    % Load high-fidelity engine deck (GasTurb Data)
    if ~isfile('J85_Engine_Performance.mat')
        error('Error: "J85_Engine_Performance.mat" not found. Please run the Propulsion Data Processor first.');
    end
    
    raw_eng = load('J85_Engine_Performance.mat'); 
    EngData = raw_eng.EngineData; 
    
    % --- GRID PREPARATION ---
    % griddedInterpolant requires 1D vectors for grid definition, not 2D meshgrids.
    % We extract the unique vectors defining the table axes.
    h_vec = EngData.Dry.Altitude(:, 1);  % Altitude Axis [m]
    M_vec = EngData.Dry.Mach(1, :);      % Mach Number Axis
    
    % --- UNIT CONVERSION FACTORS ---
    % GasTurb SFC Output: [g / (kN * s)]
    % Simulation Requirement: [kg / (N * s)]
    % Conversion: (1 g -> 0.001 kg) / (1 kN -> 1000 N) = 1e-6
    SFC_SI_Factor = 1e-6; 
    
    % Initialize Engine Sub-structures
    Airplane.Engine = struct();
    Airplane.Engine.Count   = 2;  % Twin-Engine Aircraft
    Airplane.Engine.epsilon = 0;  % Thrust Vector Angle relative to body x-axis [rad]
    
    % --- DRY POWER MAPS (MILITARY THRUST) ---
    Airplane.Engine.Dry.F_Thrust = griddedInterpolant({h_vec, M_vec}, ...
                                                       EngData.Dry.Thrust, 'linear', 'nearest');
                                                   
    Airplane.Engine.Dry.F_SFC    = griddedInterpolant({h_vec, M_vec}, ...
                                                       EngData.Dry.SFC * SFC_SI_Factor, 'linear', 'nearest');

    % --- WET POWER MAPS (AFTERBURNER) ---
    % Note: Assuming Wet and Dry maps share the same grid definition (h_vec, M_vec)
    % If grids differ in the future, extract h_vec_wet and M_vec_wet separately.
    
    Airplane.Engine.Wet.F_Thrust = griddedInterpolant({h_vec, M_vec}, ...
                                                       EngData.Wet.Thrust, 'linear', 'nearest');
                                                   
    Airplane.Engine.Wet.F_SFC    = griddedInterpolant({h_vec, M_vec}, ...
                                                       EngData.Wet.SFC * SFC_SI_Factor, 'linear', 'nearest');
end
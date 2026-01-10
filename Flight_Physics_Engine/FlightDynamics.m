function Output = FlightDynamics(State, AC, Env, Controls)
%% FLIGHT DYNAMICS ENGINE (T-38 TALON)
%  Author:      Kıvanç Apaydın
%  Description: 3-DOF Longitudinal Physics Engine.
%               Calculates Aerodynamics, Propulsion, Ground Physics,
%               and State Derivatives for Time Integration.
%  
%  Inputs:
%     State:    [Struct] Current motion variables (u, w, q, theta, pos, mass)
%     AC:       [Struct] Aircraft Data (Geo, Aero, Engine, Mass)
%     Env:      [Struct] Environment (rho, a, g, mu)
%     Controls: [Struct] Pilot Inputs (Throttle, Elevator)
%
%  Outputs:
%     Output:   [Struct] Contains Derivatives, Forces, Moments, and Logs.
% -------------------------------------------------------------------------

    %% 1. INITIALIZATION & UNPACKING
    % --- Aircraft Geometry & Mass ---
    S_ref   = AC.Geo.S_ref; 
    c_bar   = AC.Geo.c_bar;
    I_yy    = AC.Mass.Iyy;
    epsilon = AC.Engine.epsilon; % Thrust inclination angle
    
    % --- State Vectors ---
    x_E   = State.x_E; 
    z_E   = State.z_E;     % Note: z_E is positive DOWN
    h     = -z_E;          % Altitude (positive UP)
    u     = State.u; 
    w     = State.w; 
    theta = State.theta; 
    q     = State.q; 
    m     = State.m;

    % --- Environment ---
    rho = Env.rho; 
    a   = Env.a; 
    g   = Env.g; 
    mu  = Env.mu; % Friction Coefficient (Rolling or Braking)

    % --- Aerodynamic Derivatives ---
    CL_delta = AC.Aero.CL_delta; 
    Cm0      = AC.Aero.Cm0;      
    Cma      = AC.Aero.Cm_alpha;
    Cmd      = AC.Aero.Cm_delta;

    % --- Control Inputs ---
    delta_e = Controls.ElevatorDeflection;
    throttle_setting = Controls.ThrottleSetting;

    if isfield(Controls, 'SpeedBrake'), d_sb = Controls.SpeedBrake; else, d_sb = 0; end
    if isfield(Controls, 'Gear'),       d_gear = Controls.Gear;     else, d_gear = 0; end

    %% 2. KINEMATICS
    % Velocity & Air Data
    V = sqrt(u^2 + w^2);
    V_Matrix = [u; w];
    Mach_Number = V / a;
    alpha = atan2(w, u);   % [rad] Angle of Attack 
    gamma = theta - alpha; % [rad] Flight Path Angle 
    Q_dyn = 0.5*rho*V^2;   % [Pa] Dynamic Pressure 

    % Coordinate Transformations
    % L_BE: Body to Earth (Rotation Matrix)
    L_BE = [cos(theta) -sin(theta); 
            sin(theta)  cos(theta)]; 
    L_EB = L_BE'; 
    % L_BS: Stability to Body
    L_BS = [cos(alpha) -sin(alpha);
            sin(alpha)  cos(alpha)];

    %% 3. AERODYNAMICS
    % --- Lift Coefficient (Linear Model + Lookup Base) ---
    CL_base = AC.Aero.F_CL(rad2deg(alpha), Mach_Number);
    CL      = CL_base + (CL_delta * delta_e);
    
    % --- Drag Coefficient (Polar Model) ---
    CD0 = AC.Aero.F_Cd0(Mach_Number);
    K   = AC.Aero.F_K(CL, Mach_Number);
    CD_SpeedBrake = 0.025 * d_sb;
    CD_Gear       = 0.020 * d_gear;
    CD  = CD0 + K * CL^2 + CD_SpeedBrake + CD_Gear;
    
    % --- Moment Coefficient ---
    Cm  = Cm0 + (Cma * alpha) + (Cmd * delta_e);

    % --- Dimensional Forces & Moments ---
    L   = Q_dyn * S_ref * CL;
    D   = Q_dyn * S_ref * CD;
    M_Aero = Q_dyn * S_ref * c_bar * Cm;

    % --- Stall Speed Calculation (Dynamic) ---
    % Prevents extrapolation errors at low Mach
    Query_Mach = max(Mach_Number, min(AC.Aero.Limits.Mach_Vec)); 
    Current_CL_max = interp1(AC.Aero.Limits.Mach_Vec, ...
                             AC.Aero.Limits.CL_Max_Vec, ...
                             Query_Mach, 'linear');
    V_stall = sqrt(2*m*g / (rho*S_ref*Current_CL_max));

    %% 4. PROPULSION (J85-GE-5 Engine Model)
    % Logic: Interpolates Thrust/SFC maps based on Altitude & Mach.
    % Handles transition between Dry (Military) and Wet (Afterburner) power.
    
    % Clamp Altitude (Prevent negative values on ground)
    Alt_Input = max(0, h);
    
    % A. Retrieve MAX Thrust Values [kN] from Maps
    T_max_dry_kN = AC.Engine.Dry.F_Thrust(Alt_Input, Mach_Number);
    T_max_wet_kN = AC.Engine.Wet.F_Thrust(Alt_Input, Mach_Number);
    
    SFC_dry_val  = AC.Engine.Dry.F_SFC(Alt_Input, Mach_Number);
    SFC_wet_val  = AC.Engine.Wet.F_SFC(Alt_Input, Mach_Number);
    
    % B. Throttle Logic
    AB_Threshold = 0.90; % Throttle > 0.90 engages Afterburner
    
    if throttle_setting <= AB_Threshold
        % --- DRY MODE ---
        % Map 0.0-0.9 input to 0-100% Dry Thrust
        pct_power = throttle_setting / AB_Threshold; 
        
        T_single_kN = T_max_dry_kN * pct_power;
        SFC_current = SFC_dry_val; 
        AB_Status   = 0;
    else
        % --- WET MODE (AFTERBURNER) ---
        % Map 0.9-1.0 input to transition from Dry Max to Wet Max
        pct_ab = (throttle_setting - AB_Threshold) / (1.0 - AB_Threshold);
        
        % Linear Interpolation
        T_single_kN = T_max_dry_kN + (T_max_wet_kN - T_max_dry_kN) * pct_ab;
        SFC_current = SFC_dry_val + (SFC_wet_val - SFC_dry_val) * pct_ab;
        AB_Status   = 1;
    end
    
    % C. Final Propulsion Outputs
    % Convert kN to Newtons and multiply by engine count
    T_Total = (T_single_kN * 1000) * AC.Engine.Count; 
    
    % Fuel Flow [kg/s] (Convention: Negative for mass loss)
    m_dot_fuel = -(T_Total * SFC_current);

    %% 5. EQUATIONS OF MOTION
    
    W = m*g;

    F_stability = [-D; -L];
    T_Matrix = [T_Total*cos(epsilon); -T_Total*sin(epsilon)];
    m_Matrix = [-m*g*sin(theta); m*g*cos(theta)];
    Coriolis_Acc = [-q*w; q*u];

    % Total Aeropropulsive Force in Body Frame
    F_aerop_body = L_BS*F_stability + T_Matrix;
    
    % Newton's Second Law: F = ma -> a = F/m
    a_Matrix = (F_aerop_body + m_Matrix)/m + Coriolis_Acc;
    a_Matrix_E = L_EB*a_Matrix; % just for plotting

    % Transform Velocity to Earth Frame (For Position Integration)
    V_E_Matrix = L_EB*V_Matrix;

    N = -m*a_Matrix_E(2);

    % Modelling the ground
    if N >= 0 || -z_E > 0.01 
        N = 0;
        F_Friction = 0;
    else
        F_Friction = N*mu;
        Ground_Force_Total = [F_Friction; N];
        a_Matrix_E = a_Matrix_E + Ground_Force_Total/m; 
        a_Matrix = L_BE*a_Matrix_E;
    end
    
    %% 6. OUTPUT PACKAGING
    
    % Derivatives (For Integration)
    Output.Derivatives.x_dot_E = V_E_Matrix(1);
    Output.Derivatives.z_dot_E = V_E_Matrix(2);
    Output.Derivatives.u_dot = a_Matrix(1);
    Output.Derivatives.w_dot = a_Matrix(2);
    Output.Derivatives.theta_dot = q;
    Output.Derivatives.q_dot = M_Aero/I_yy;
    Output.Derivatives.m_dot = m_dot_fuel;

    % State Variables (For Analysis/Plotting)
    Output.States.Alpha = alpha;
    Output.States.Gamma = gamma;
    Output.States.Delta_e = delta_e;
    Output.States.Throttle_Setting = throttle_setting;
    Output.States.AB_Status = AB_Status;
    Output.States.SFC = SFC_current;
    Output.States.V = V;
    Output.States.Mach_Number = Mach_Number;
    Output.States.Vstall = V_stall;

    Output.States.u_E = V_E_Matrix(1);
    Output.States.w_E = V_E_Matrix(2);
    Output.States.u_dot_E = a_Matrix_E(1); 
    Output.States.w_dot_E = a_Matrix_E(2); 
    
    % Force & Moment Variables (For Analysis/Plotting)
    Output.Forces.W = W;
    Output.Forces.L = L;
    Output.Forces.D = D;
    Output.Forces.T = T_Total;
    Output.Forces.N = N;
    Output.Forces.F_Friction = F_Friction;
    Output.Moments.M_Aero = M_Aero;
end
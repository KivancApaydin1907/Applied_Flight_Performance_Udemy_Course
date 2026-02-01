%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & AUTOPILOT SIMULATION
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     A high-fidelity, 3-DOF point-mass mission simulation of the T-38 Talon
%     supersonic trainer. The simulation covers the entire flight envelope:
%     1. Takeoff & Rotation
%     2. Climb (Constant Mach)
%     3. Cruise (Cascade Altitude Hold)
%     4. Descent (Constant Alpha)
%     5. Approach (Gamma/Alpha Constraint)
%     6. Precision Flare & Touchdown
%     7. Rollout & Braking
%
%  DEPENDENCIES:
%     - FlightDynamics.m  (Physics Engine)
%     - RunPID.m          (Controller Logic)
%     - atmosisa.m        (Standard Atmosphere)
%     - T_38_General_Data (Aircraft Database)
% ========================================================================

clearvars; clc; 

%% ========================================================================
%  PHASE 1: INITIALIZATION
%  ========================================================================
%  Objective: Setup simulation clock, load aircraft data, and define ICs.
% -------------------------------------------------------------------------
fprintf('>> SIMULATION STARTED.\n');

% --- 1. SIMULATION PARAMETERS ---
dt   = 0.01;                % [s] Integration Time Step (High fidelity)
i    = 1;                   % Step Counter
time = 0;                   % [s] Master Clock

% --- 2. AIRCRAFT & ENVIRONMENT ---
AC = T_38_General_Data();   % Load Aerodynamic, Geometric & Mass Data

% --- 3. INITIAL CONDITIONS (ICs) ---
x_E(i)   = 0;               % [m]     Runway Start Position
z_E(i)   = 0;               % [m]     Ground Level (Altitude = -z_E)
u(i)     = 0.5;             % [m/s]   Initial Taxi Speed (Avoids singularity)
w(i)     = 0;               % [m/s]   Vertical Speed
theta(i) = 0;               % [rad]   Pitch Attitude
q(i)     = 0;               % [rad/s] Pitch Rate
m(i)     = AC.Mass.Initial; % [kg]    Takeoff Mass

% --- 4. FIRST STEP CALCULATION ---
% Initialize atmosphere and state vectors
[~, a(i), ~, rho(i)] = atmosisa(-z_E(i));
Env   = struct('rho', rho(i), 'a', a(i), 'g', 9.81, 'mu', 0.04);
State = struct('m', m(i), 'x_E', x_E(i), 'z_E', -z_E(i), ...
               'u', u(i), 'w', w(i), 'theta', theta(i), 'q', q(i));

% Initialize Controls (Takeoff Configuration)
Controls.ElevatorDeflection = 0;
Controls.ThrottleSetting    = 1.0; % Full Afterburner
Controls.Gear               = 1.0; 

% Run Physics Engine for Step 0
Output    = FlightDynamics(State, AC, Env, Controls);
SimLog(i) = Output;

%% ========================================================================
%  PHASE 2: GROUND ROLL ACCELERATION
%  ========================================================================
%  Objective: Accelerate to rotation speed (Vr = 1.1 * Vstall).
%  Dynamics:  Friction active, Stick Neutral, Full Power.
% -------------------------------------------------------------------------
fprintf('Phase 02: Ground Roll (Accelerating)...\n');

% --- MISSION EVENTS TRACKER ---
MissionMarkers = struct();
MissionMarkers.Start = 1;

% Calculate Stall Speed reference
V_Stall_Ref = SimLog(i).States.Vstall; 

while SimLog(i).States.V < (V_Stall_Ref * 1.1)
    
    % [Integration] Euler Method
    Derv = SimLog(i).Derivatives;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt;
    time(i+1)  = time(i)  + dt;

    V_ground = sqrt(u(i+1)^2 + w(i+1)^2);

    u(i + 1) = V_ground * cos(theta(i + 1));
    w(i + 1) = V_ground * sin(theta(i + 1));

    % [Environment]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1);
    Env.a   = a(i+1);
    
    % [Control] Stick Fixed, Full Power
    Controls.ElevatorDeflection = 0;
    Controls.ThrottleSetting    = 1.0;
    Controls.Gear               = 1.0;
    
    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);
    
    i = i + 1; 
    SimLog(i) = Output;
end

% =========================================================================
% CHECKPOINT: SAVE STATE FOR TUNER
% =========================================================================
% We save the current index 'i' and all state arrays. 
% This allows the tuner to pick up exactly where we left off.
Checkpoint.StateVector = struct('m',m(i),'x_E',x_E(i),'z_E',z_E(i),...
                                'u',u(i),'w',w(i),'theta',theta(i),'q',q(i));
Checkpoint.AC   = AC;           % Aircraft Data
Checkpoint.Env  = Env;          % Last known Environment
Checkpoint.Time = time(i);      % Simulation Clock
Checkpoint.dt   = dt;           % Time Step

save('Rotation_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint saved to "Rotation_Checkpoint.mat"\n');

%% ========================================================================
%  PHASE 3: ROTATION
%  ========================================================================
%  Objective: Rotate aircraft to lift-off attitude (12 deg).
%  Condition: Continue until Lift > Weight (Normal Force < 0).
% -------------------------------------------------------------------------
fprintf('Phase 03: Rotation (Nose Up)...\n');

% --- CONTROLLER CONFIGURATION ---
% PD Control for short-duration rotation maneuver
ControlConfig.Rot.Kp   = -1.24522;              
ControlConfig.Rot.Ki   =  0.0;            
ControlConfig.Rot.Kd   =  0.40295;           
ControlConfig.Rot.Max  = deg2rad(15);      
ControlConfig.Rot.Min  = -deg2rad(15);
Target_Rotation_Theta  = deg2rad(12);      

% --- STATE INITIALIZATION ---
RotState = struct('Integrator', 0, 'PrevError', 0);

% Loop checks if Ground Normal Force is still pushing up
while SimLog(i).Forces.N < 0

    % [Integration]
    Derv = SimLog(i).Derivatives;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt; 
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt;
    time(i+1)  = time(i)  + dt;

    V_ground = sqrt(u(i+1)^2 + w(i+1)^2);

    u(i + 1) = V_ground * cos(theta(i + 1));
    w(i + 1) = V_ground * sin(theta(i + 1));

    % [Sensors]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1);
    Env.a   = a(i+1);

    % [Control Logic]
    [delta_cmd, RotState] = RunPID(Target_Rotation_Theta, theta(i+1), dt, ...
                               ControlConfig.Rot, RotState, q(i+1));

    Controls.ElevatorDeflection = delta_cmd;
    Controls.ThrottleSetting    = 1.0;
    Controls.Gear               = 1.0;

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; 
    SimLog(i) = Output;
end

%% ========================================================================
%  PHASE 4: AIRBORNE / INITIAL CLIMB
%  ========================================================================
%  Objective: Clear obstacle height (15m) while holding rotation attitude.
% -------------------------------------------------------------------------
fprintf('Phase 04: Airborne (Initial Climb)...\n');

% Continue using the same Rotation Controller Configuration (Hold 12 deg)
% RotState is preserved to prevent actuator jumps.

while -z_E(i) < 15
    % [Integration]
    Derv = SimLog(i).Derivatives;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt;
    time(i+1)  = time(i)  + dt;

    % [Environment & Sensors]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1);
    Env.a   = a(i+1);

    % [Control Logic]
    [delta_cmd, RotState] = RunPID(Target_Rotation_Theta, theta(i+1), dt, ...
                               ControlConfig.Rot, RotState, q(i+1));

    Controls.ElevatorDeflection = delta_cmd;
    Controls.ThrottleSetting    = 1.0;
    Controls.Gear               = 1.0;

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; 
    SimLog(i) = Output;
end
MissionMarkers.Takeoff_End = i;

%% ========================================================================
%  CHECKPOINT: SAVE STATE FOR CLIMB TUNER
%  ========================================================================
%  Purpose: Freezes the simulation exactly at the end of the transition/
%  start of the climb. The "Tuner_Climb.m" script will load this file 
%  to repeatedly optimize the climb gains without re-running takeoff.
% -------------------------------------------------------------------------
Checkpoint.StateVector = struct('m',m(i),'x_E',x_E(i),'z_E',z_E(i),...
                                'u',u(i),'w',w(i),'theta',theta(i),'q',q(i));
Checkpoint.AC   = AC;           % Aircraft Constants
Checkpoint.Env  = Env;          % Current Atmosphere (Density, Speed of Sound)
Checkpoint.Time = time(i);      % Simulation Clock
Checkpoint.dt   = 0.1;          % Suggested dt for Climb (slower than rotation)

% This captures the exact throttle (e.g., 1.0) used at the end of airborne
Checkpoint.LastControls = Controls;

save('Climb_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint saved to "Climb_Checkpoint.mat" (t = %.2f s)\n', time(i));

%% ========================================================================
%  PHASE 5: CLIMB (CONSTANT MACH)
%  ========================================================================
%  Objective: Climb to 11,000m maintaining Mach 0.6.
%  Strategy:  Decoupled PID (Pitch -> Theta, Throttle -> Mach).
% -------------------------------------------------------------------------
fprintf('Phase 05: Climb (Constant Mach)...\n');
dt = 0.1; 

% --- CONFIGURATION ---
Controls.Gear      = 0.0;          % Gear Retracted
Target_Climb_Mach  = 0.6;          % Target Mach
Target_Pitch_Angle = deg2rad(10);  % Target Attitude

% [A] PITCH CONTROLLER (Attitude Hold)
ControlConfig.Pitch.Kp = -35.0134;
ControlConfig.Pitch.Ki =  0.0245;
ControlConfig.Pitch.Kd =  24.3466;
ControlConfig.Pitch.Max  = deg2rad(20);
ControlConfig.Pitch.Min  = -deg2rad(20);

% [B] SPEED CONTROLLER (Auto-Throttle)
ControlConfig.Speed.Kp =  649.2745;
ControlConfig.Speed.Ki =  -0.0015;
ControlConfig.Speed.Kd =  2.8893;
ControlConfig.Speed.Max  = 1.0;          
ControlConfig.Speed.Min  = 0.0;         
ControlConfig.Speed.Base = 0.5212; 

% --- STATE INITIALIZATION ---
PitchState = struct('Integrator', 0, 'PrevError', 0);
SpeedState = struct('Integrator', 0, 'PrevError', 0);

% --- SIMULATION LOOP ---
while -z_E(i) < 11000
    % [Integration]
    Derv = SimLog(i).Derivatives;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt;
    time(i+1)  = time(i)  + dt;

    % [Environment & Sensors]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1); 
    Env.a   = a(i+1);

    current_theta = theta(i+1);
    current_q     = q(i+1);
    V_current     = sqrt(u(i+1)^2 + w(i+1)^2);
    current_mach  = V_current / Env.a;
    V_E_Matrix    = [u(i+1); w(i+1)];

    % [Control Logic]
    % 1. Pitch Control
    [delta_cmd, PitchState] = RunPID(Target_Pitch_Angle, current_theta, dt, ...
                                 ControlConfig.Pitch, PitchState, current_q);
    % 2. Speed Control
    [throttle_cmd, SpeedState] = RunPID(Target_Climb_Mach, current_mach, dt, ...
                                        ControlConfig.Speed, SpeedState);

    Controls.ElevatorDeflection = delta_cmd;
    Controls.ThrottleSetting    = throttle_cmd;

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1), ...
               'V_E_Matrix', V_E_Matrix);
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; 
    SimLog(i) = Output;

    if -z_E(i) < 0, error("CRASH IN CLIMB"); end
end
MissionMarkers.Climb_End = i;

%% ========================================================================
%  CHECKPOINT: SAVE STATE FOR CRUISE TUNER
%  ========================================================================
Checkpoint.StateVector = struct('m',m(i),'x_E',x_E(i),'z_E',z_E(i),...
                                'u',u(i),'w',w(i),'theta',theta(i),'q',q(i));
Checkpoint.AC   = AC;
Checkpoint.Env  = Env;
Checkpoint.Time = time(i);
Checkpoint.dt   = 0.1;
Checkpoint.LastControls = Controls; 

save('Cruise_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint saved to "Cruise_Checkpoint.mat" (t = %.2f s)\n', time(i));

%% ========================================================================
%  PHASE 6: CRUISE (CASCADE ALTITUDE HOLD)
%  ========================================================================
%  Objective: Accelerate to Mach 0.8 and hold 11,000m.
%  Strategy:  Cascade Loop (Altitude -> Pitch -> Elevator).
% -------------------------------------------------------------------------
fprintf('Phase 06: Cruise (Altitude Hold)...\n');
dt = 0.1; 

% --- FLIGHT ENVELOPE TARGETS ---
Target_Alt  = 11000;        % [m] Service Ceiling
Target_Mach = 0.8;          % [Mach] Cruise Speed

% --- CONTROLLER CONFIGURATION ---
% [A] ALTITUDE CONTROLLER (Outer Loop)
ControlConfig.Alt.Kp   =  0.00157706;   
ControlConfig.Alt.Ki   =  0.00011752;   
ControlConfig.Alt.Kd   = -0.00210368;     
ControlConfig.Alt.Max  =  deg2rad(5);
ControlConfig.Alt.Min  = -deg2rad(5);    

% [B] PITCH CONTROLLER (Inner Loop)
ControlConfig.Pitch.Kp  = -35.6016;   
ControlConfig.Pitch.Ki  =  0.0244;   
ControlConfig.Pitch.Kd  =  24.3107;    
ControlConfig.Pitch.Max = deg2rad(20);
ControlConfig.Pitch.Min = -deg2rad(20);

% [C] SPEED CONTROLLER (Auto-Throttle)
ControlConfig.Speed.Kp   =  649.2745;
ControlConfig.Speed.Ki   = -0.0015;
ControlConfig.Speed.Kd   =  2.8893;
ControlConfig.Speed.Max  = 1.0;
ControlConfig.Speed.Min  = 0.0;
ControlConfig.Speed.Base = 0.1997;  

% --- STATE INITIALIZATION ---
AltState   = struct('Integrator', 0, 'PrevError', 0);
PitchState = struct('Integrator', 0, 'PrevError', 0);
SpeedState = struct('Integrator', 0, 'PrevError', 0);

% --- FUEL LOGIC ---
Design_Empty_Mass = 3500; 
Reserve_Fuel      = 250;
Target_Mass_End   = Design_Empty_Mass + Reserve_Fuel;

% --- SIMULATION LOOP ---
while m(i) > Target_Mass_End
    % [Integration]
    Derv = SimLog(i).Derivatives;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt; 
    time(i+1)  = time(i)  + dt;

    % [Environment & Sensors]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1); 
    Env.a   = a(i+1);

    current_theta = theta(i+1);
    current_q     = q(i+1);
    current_alt   = -z_E(i+1);
    V_current     = sqrt(u(i+1)^2 + w(i+1)^2);
    current_mach  = V_current / Env.a;

    % [Control Logic]
    % 1. Outer Loop (Altitude -> Pitch Ref)
    alt_rate = -Derv.z_dot_E; 
    [theta_ref_cruise, AltState] = RunPID(Target_Alt, current_alt, dt, ...
                                          ControlConfig.Alt, AltState, -alt_rate);
    % 2. Inner Loop (Pitch Ref -> Elevator)
    [delta_cmd, PitchState] = RunPID(theta_ref_cruise, current_theta, dt, ...
                                 ControlConfig.Pitch, PitchState, current_q);
    % 3. Speed Control
    [throttle_cmd, SpeedState] = RunPID(Target_Mach, current_mach, dt, ...
                                        ControlConfig.Speed, SpeedState);

    Controls.ElevatorDeflection = delta_cmd;
    Controls.ThrottleSetting    = throttle_cmd;

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; 
    SimLog(i) = Output;

    if -z_E(i) < 0, error("CRASH IN CRUISE"); end
end
MissionMarkers.Cruise_End = i;

%% ========================================================================
%  CHECKPOINT: SAVE STATE FOR DESCENT TUNER
%  ========================================================================
Checkpoint.StateVector = struct('m',m(i),'x_E',x_E(i),'z_E',z_E(i),...
                                'u',u(i),'w',w(i),'theta',theta(i),'q',q(i));
Checkpoint.AC   = AC;
Checkpoint.Env  = Env;
Checkpoint.Time = time(i);
Checkpoint.dt   = 0.1;

% Note: We don't strictly need 'LastControls' here because Descent 
% forces Throttle to 0.0 anyway.
save('Descent_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint saved to "Descent_Checkpoint.mat" (t = %.2f s)\n', time(i));

%% ========================================================================
%  PHASE 7: DESCENT (CONSTANT ALPHA MODE)
%  ========================================================================
%  Objective: Descend to 500m maintaining 8.0 deg Angle of Attack.
%  Strategy:  Cascade Loop (Alpha Error -> Pitch Ref -> Elevator).
% -------------------------------------------------------------------------
fprintf('Phase 07: Descent (Constant Alpha Mode)...\n');
dt = 0.1;
Target_Alt_End = 500; % [m] Hand-off altitude

% --- CONTROLLER CONFIGURATION ---
% [A] ALPHA CONTROLLER (Outer Loop)
ControlConfig.Alpha.Kp   = 0.537675;   
ControlConfig.Alpha.Ki   = 0.009969;
ControlConfig.Alpha.Kd   = 0.043239;    
ControlConfig.Alpha.Min  = deg2rad(-10); 
ControlConfig.Alpha.Max  = deg2rad(10);  

% [B] PITCH CONTROLLER (Inner Loop)
ControlConfig.Pitch.Kp  = -35.6016;   
ControlConfig.Pitch.Ki  =  0.0244;   
ControlConfig.Pitch.Kd  =  24.3107;   
ControlConfig.Pitch.Max = deg2rad(20);
ControlConfig.Pitch.Min = -deg2rad(20);

% --- STATE INITIALIZATION ---
AlphaState = struct('Integrator', 0, 'PrevError', 0);
PitchState = struct('Integrator', 0, 'PrevError', 0);
Target_Alpha = deg2rad(8.0); 

% --- SIMULATION LOOP ---
while -z_E(i) > Target_Alt_End
    % [Integration]
    Derv = SimLog(i).Derivatives;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt; 
    time(i+1)  = time(i)  + dt;

    % [Environment & Sensors]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1); Env.a = a(i+1);

    current_u     = u(i+1);
    current_w     = w(i+1);
    current_theta = theta(i+1);
    current_q     = q(i+1);
    current_alpha = atan2(current_w, current_u);
    current_gamma = current_theta - current_alpha;

    % [Control Logic]
    % 1. Outer Loop: Alpha -> Ref Theta
    [Ref_Theta, AlphaState] = RunPID(Target_Alpha, current_alpha, dt, ...
                                     ControlConfig.Alpha, AlphaState);
    % 2. Inner Loop: Ref Theta -> Elevator
    [delta_cmd, PitchState] = RunPID(Ref_Theta, current_theta, dt, ...
                                     ControlConfig.Pitch, PitchState, current_q);

    Controls.ElevatorDeflection = delta_cmd;
    Controls.ThrottleSetting    = 0.0; % Idle
    Controls.SpeedBrake         = 1.0; % Full Drag
    Controls.Gear               = 0.0; 

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; SimLog(i) = Output;

    % Termination Check
    if -z_E(i) <= Target_Alt_End
         V_Total = sqrt(u(i)^2 + w(i)^2);
         fprintf('>> DESCENT COMPLETE. H: %.1f m, V: %.1f kts\n', -z_E(i), V_Total*1.94);
         break;
    end
    % if mod(i, 10) == 0  % Print every 50 steps
    % fprintf('Alt: %.0f m | α: %.2f° (target: %.2f°) | θ: %.2f° | γ: %.2f° | δe: %.2f°\n', ...
    %     -z_E(i), rad2deg(current_alpha), rad2deg(Target_Alpha), ...
    %     rad2deg(current_theta), rad2deg(current_gamma), rad2deg(delta_cmd));
    % end
end

%% ========================================================================
%  CHECKPOINT: SAVE STATE FOR APPROACH TUNER
%  ========================================================================
Checkpoint.StateVector = struct('m',m(i),'x_E',x_E(i),'z_E',z_E(i),...
                                'u',u(i),'w',w(i),'theta',theta(i),'q',q(i));
Checkpoint.AC   = AC;
Checkpoint.Env  = Env;
Checkpoint.Time = time(i);
Checkpoint.dt   = 0.01;

save('Approach_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint saved to "Approach_Checkpoint.mat" (t = %.2f s)\n', time(i));


%% ========================================================================
%  PHASE 8: CONSTRAINED APPROACH (GAMMA / ALPHA LOGIC)
%  ========================================================================
%  Objective: Track -4 deg Gamma while limiting Alpha < 12 deg.
%  Strategy:  Pitch controls Gamma. Energy managed via Brakes/Throttle.
% -------------------------------------------------------------------------
fprintf('Phase 08: Constrained Approach (Gamma -7 / Max Alpha 12)...\n');
dt = 0.01; 
Handoff_Height = 30; % [m] Transition to Flare

% --- CONTROLLER CONFIGURATION ---
% [A] GAMMA CONTROLLER (Outer Loop)
ControlConfig.Gamma.Kp   = 0.516779;
ControlConfig.Gamma.Ki   = 0.037596;
ControlConfig.Gamma.Kd   = 0.385513;
ControlConfig.Gamma.Max  = deg2rad(5.0); 
ControlConfig.Gamma.Min  = deg2rad(-10.0);

% [B] PITCH CONTROLLER (Inner Loop)
ControlConfig.Pitch.Kp = -35.0134;
ControlConfig.Pitch.Ki =  0.0245;
ControlConfig.Pitch.Kd =  24.3466;  
ControlConfig.Pitch.Max = deg2rad(20);
ControlConfig.Pitch.Min = -deg2rad(20);

% --- STATE INITIALIZATION ---
PitchState = struct('Integrator', 0, 'PrevError', 0);
GammaState   = struct('Integrator', 0, 'PrevError', 0);
Target_Gamma = deg2rad(-4.0); 

% --- SIMULATION LOOP ---
while -z_E(i) > Handoff_Height
    % [Integration]
    Derv = SimLog(i).Derivatives;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt; 
    time(i+1)  = time(i)  + dt;

    % [Environment & Sensors]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1); Env.a = a(i+1);

    current_u     = u(i+1); 
    current_w     = w(i+1);
    current_theta = theta(i+1); 
    current_q     = q(i+1);
    current_alpha = atan2(-current_w, current_u);
    current_gamma = current_theta - current_alpha;
    Alpha_Deg     = rad2deg(current_alpha);

    % [Control Logic]
    % 1. Flight Path Control (Gamma -> Pitch)
    [Ref_Theta, GammaState] = RunPID(Target_Gamma, current_gamma, dt, ...
                                     ControlConfig.Gamma, GammaState);

    % 2. Attitude Control (Pitch -> Elevator)
    [delta_cmd, PitchState] = RunPID(Ref_Theta, current_theta, dt, ...
                                    ControlConfig.Pitch, PitchState, current_q);

    % 3. Energy Management (Alpha Limiter Logic)
    Throttle_Cmd   = 0.0;
    SpeedBrake_Cmd = 0.5;

    if Alpha_Deg > 12.0,     Throttle_Cmd = 1.0;
    elseif Alpha_Deg > 11.5, Throttle_Cmd = 0.5;
    end

    if Alpha_Deg < 10.0,     SpeedBrake_Cmd = 1.0;
    elseif Alpha_Deg > 11.0, SpeedBrake_Cmd = 0.0;
    end

    Controls.ElevatorDeflection = delta_cmd;
    Controls.ThrottleSetting    = Throttle_Cmd;
    Controls.SpeedBrake         = SpeedBrake_Cmd;
    Controls.Gear               = 1.0; 

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; SimLog(i) = Output;

    if Alpha_Deg > 18, fprintf('!! HARD STALL !!\n'); break; end
    
    % if mod(i, 10) == 0  % Print every 50 steps
    % fprintf('Alt: %.0f m | α: %.2f° (target: %.2f°) | θ: %.2f° | γ: %.2f° | δe: %.2f°\n', ...
    %     -z_E(i), rad2deg(current_alpha), rad2deg(Target_Alpha), ...
    %     rad2deg(current_theta), rad2deg(current_gamma), rad2deg(delta_cmd));
    % end   
end
fprintf('==============================================\n');
fprintf(' >> PHASE 8 COMPLETE (APPROACH).\n');
fprintf(' >> Handoff Alt: %.1f m | Gamma: %.2f deg\n', -z_E(i), rad2deg(current_gamma));
fprintf('==============================================\n');
MissionMarkers.Descent_End = i;

%% ========================================================================
%  PHASE 9: PRECISION FLARE
%  ========================================================================
%  Objective: Arrest sink rate to < 2.0 m/s for safe touchdown.
%  Strategy:  Static Pitch Hold + PID Throttle Controller.
% -------------------------------------------------------------------------
fprintf('Phase 09: Precision Flare...\n');

% --- CONTROLLER CONFIGURATION ---
% [A] THROTTLE CONTROLLER (Sink Rate)
ControlConfig.Throttle.Kp   = 0.0592;
ControlConfig.Throttle.Ki   = 0.0085;
ControlConfig.Throttle.Kd   = 0.0147;
ControlConfig.Throttle.Base =  0.5138; 
ControlConfig.Throttle.Max  =  1.0;
ControlConfig.Throttle.Min  =  0.0;

% [B] PITCH CONTROLLER (Inner Loop)
ControlConfig.Pitch.Kp  = -4.5945;
ControlConfig.Pitch.Ki  =  0.0226; 
ControlConfig.Pitch.Kd  =  14.2681;   
ControlConfig.Pitch.Max =  deg2rad(20);
ControlConfig.Pitch.Min = -deg2rad(20);

% --- STATE & TARGETS ---
PitchState = struct('Integrator', 0, 'PrevError', 0);
ThrotState = struct('Integrator', 0, 'PrevError', 0); 
Target_Theta    = deg2rad(6.5);  % Landing Attitude
Target_SinkRate = -0.6;          % [m/s] 

% --- SIMULATION LOOP ---
while -z_E(i) > 0.1
    % [Integration]
    Derv = SimLog(i).Derivatives;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    m(i+1)     = m(i)     + Derv.m_dot * dt; 
    time(i+1)  = time(i)  + dt;

    % [Environment & Sensors]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1); Env.a = a(i+1);

    Current_Alt      = -z_E(i+1);
    Current_SinkRate = -Derv.z_dot_E;
    current_theta    = theta(i+1); 
    current_q        = q(i+1);

    % [Control Logic]
    % 1. Throttle (Sink Rate Control)
    [Throttle_Cmd, ThrotState] = RunPID(Target_SinkRate, Current_SinkRate, dt, ...
                                        ControlConfig.Throttle, ThrotState);
    % Cut power right before touchdown
    if Current_Alt < 0.3, Throttle_Cmd = 0.0; end

    % 2. Pitch (Attitude Hold)
    [Elevator_Cmd, PitchState] = RunPID(Target_Theta, current_theta, dt, ...
                                        ControlConfig.Pitch, PitchState, current_q);

    Controls.ElevatorDeflection = Elevator_Cmd;
    Controls.ThrottleSetting    = Throttle_Cmd;
    Controls.SpeedBrake         = 0.5; 
    Controls.Gear               = 1.0; 

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; SimLog(i) = Output;
end

% --- TOUCHDOWN REPORT ---
Impact_Speed_Kts = sqrt(u(end)^2 + w(end)^2) * 1.94;
Sink_Rate_MS     = Derv.z_dot_E;
fprintf('\n==============================================\n');
fprintf(' >> TOUCHDOWN CONFIRMED.\n');
fprintf(' >> Impact Speed: %.1f kts\n', Impact_Speed_Kts);
fprintf(' >> Sink Rate:    %.2f m/s\n', Sink_Rate_MS);
fprintf(' >> Pitch Angle:  %.2f deg\n', rad2deg(theta(i)));
fprintf('==============================================\n');
if Sink_Rate_MS < 1.0
    fprintf(' >> RESULT: BUTTER (Excellent).\n');
elseif Sink_Rate_MS < 2.0
    fprintf(' >> RESULT: SAFE LANDING.\n');
else
    fprintf(' >> RESULT: HARD LANDING.\n');
end

%% ========================================================================
%  CHECKPOINT: SAVE STATE FOR DEROTATION TUNER
%  ========================================================================
Checkpoint.StateVector = struct('m',m(i),'x_E',x_E(i),'z_E',z_E(i),...
                                'u',u(i),'w',w(i),'theta',theta(i),'q',q(i));
Checkpoint.AC   = AC;
Checkpoint.Env  = Env;
Checkpoint.Time = time(i);
Checkpoint.dt   = 0.01; 

save('Derotation_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint saved to "Derotation_Checkpoint.mat" (t = %.2f s)\n', time(i));

%% ========================================================================
%  PHASE 10: DEROTATION (NOSE LOWERING)
%  ========================================================================
%  Objective: Controlled nose-down to runway (Active Suspension).
% -------------------------------------------------------------------------
fprintf('Phase 10: Derotation (Nose Lowering)...\n');

z_E(i) = 0;
% --- CONFIGURATION ---
Target_Derot_Theta      =  0;
ControlConfig.Derot.Kp  = -3.26077;             
ControlConfig.Derot.Ki  =  0.0;             
ControlConfig.Derot.Kd  =  2.78721 ;             
ControlConfig.Derot.Max =  deg2rad(20);     
ControlConfig.Derot.Min = -deg2rad(20);     

DerotState = struct('Integrator', 0, 'PrevError', 0);

% --- SIMULATION LOOP ---
while theta(i) > 0
    % [Integration]
    Derv = SimLog(i).Derivatives;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = w(i)     + Derv.w_dot * dt;       
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = z_E(i)   + Derv.z_dot_E * dt;     
    q(i+1)     = q(i)     + Derv.q_dot * dt;
    theta(i+1) = theta(i) + Derv.theta_dot * dt;
    m(i+1)     = m(i);   
    time(i+1)  = time(i)  + dt;

    V_ground = sqrt(u(i+1)^2 + w(i+1)^2);

    u(i + 1) = V_ground * cos(theta(i + 1));
    w(i + 1) = V_ground * sin(theta(i + 1));

    % [Environment]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1);
    Env.a   = a(i+1);

    % [Control Logic]
    [Elevator_Cmd, DerotState] = RunPID(Target_Derot_Theta, theta(i+1), dt, ...
                                        ControlConfig.Derot, DerotState, q(i+1));

    Controls.ElevatorDeflection = Elevator_Cmd;
    Controls.ThrottleSetting    = 0.0;  % Idle
    Controls.SpeedBrake         = 1.0;  % Full Aero Braking
    Controls.Gear               = 1.0;

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    % [Ground Constraint Safety]
    if z_E(i+1) > 0 
        z_E(i+1) = 0; w(i+1) = 0; 
    end

    i = i + 1; 
    SimLog(i) = Output;    
end
fprintf(' >> Nose Gear Touchdown (Theta = %.2f deg)\n', rad2deg(theta(i)));
fprintf(' >> Touchdown Angular Velocity (q = %.2f rad/s)\n', q(i));

%% ========================================================================
%  PHASE 11: FULL BRAKING (GROUND ROLL)
%  ========================================================================
%  Objective: Maximize deceleration to full stop.
%  Dynamics:  3-Point Kinematic Lock (No rotation).
% -------------------------------------------------------------------------
fprintf('Phase 11: Ground Roll (Max Braking)...\n');
Mu_Braking = 0.50;  % Braking Coefficient (Dry Concrete)
z_E(i) = 0;

while SimLog(i).States.V > 0.5

    % [Integration]
    Derv = SimLog(i).Derivatives;
    x_E(i+1)   = x_E(i)   + Derv.x_dot_E * dt;
    z_E(i+1)   = 0;
    u(i+1)     = u(i)     + Derv.u_dot * dt;
    w(i+1)     = 0;
    theta(i+1) = 0;
    q(i+1)     = 0;
    m(i+1)     = m(i)     + Derv.m_dot * dt;
    time(i+1)  = time(i)  + dt;

    V_ground = sqrt(u(i+1)^2 + w(i+1)^2);

    u(i + 1) = V_ground * cos(theta(i + 1));
    w(i + 1) = V_ground * sin(theta(i + 1));

    % [Environment]
    [~, a(i+1), ~, rho(i+1)] = atmosisa(-z_E(i+1));
    Env.rho = rho(i+1);
    Env.a   = a(i+1);
    Env.mu  = Mu_Braking;

    % [Control Logic]
    % Stick full forward (20 deg) to load nose gear
    Controls.ElevatorDeflection = deg2rad(20); 
    Controls.ThrottleSetting    = 0.0;
    Controls.SpeedBrake         = 1.0; 
    Controls.Gear               = 1.0;

    % [Physics]
    State = struct('m', m(i+1), 'x_E', x_E(i+1), 'z_E', z_E(i+1), ...
               'u', u(i+1), 'w', w(i+1), 'theta', theta(i+1), 'q', q(i+1));
    Output = FlightDynamics(State, AC, Env, Controls);

    i = i + 1; 
    SimLog(i) = Output;
end

%% ========================================================================
%  12. FINAL MISSION PERFORMANCE REPORT
%  ========================================================================
fprintf('\n');
fprintf('=================================================================\n');
fprintf('                  MISSION PERFORMANCE SUMMARY                    \n');
fprintf('=================================================================\n');
fprintf('| %-12s | %-12s | %-15s |\n', 'SEGMENT', 'DURATION', 'DISTANCE');
fprintf('|--------------|--------------|-----------------|\n');

% --- 1. TAKEOFF ---
T_TO = time(MissionMarkers.Takeoff_End) - time(MissionMarkers.Start);
X_TO = x_E(MissionMarkers.Takeoff_End)  - x_E(MissionMarkers.Start);
fprintf('| %-12s | %6.1f s     | %8.1f m      |\n', 'TAKEOFF', T_TO, X_TO);

% --- 2. CLIMB ---
T_CL = time(MissionMarkers.Climb_End) - time(MissionMarkers.Takeoff_End);
X_CL = x_E(MissionMarkers.Climb_End)  - x_E(MissionMarkers.Takeoff_End);
fprintf('| %-12s | %6.1f min   | %8.1f km     |\n', 'CLIMB', T_CL/60, X_CL/1000);

% --- 3. CRUISE ---
T_CR = time(MissionMarkers.Cruise_End) - time(MissionMarkers.Climb_End);
X_CR = x_E(MissionMarkers.Cruise_End)  - x_E(MissionMarkers.Climb_End);
fprintf('| %-12s | %6.1f min   | %8.1f km     |\n', 'CRUISE', T_CR/60, X_CR/1000);

% --- 4. DESCENT ---
T_DE = time(MissionMarkers.Descent_End) - time(MissionMarkers.Cruise_End);
X_DE = x_E(MissionMarkers.Descent_End)  - x_E(MissionMarkers.Cruise_End);
fprintf('| %-12s | %6.1f min   | %8.1f km     |\n', 'DESCENT', T_DE/60, X_DE/1000);

% --- 5. LANDING ---
T_LA = time(i) - time(MissionMarkers.Descent_End);
X_LA = x_E(i)  - x_E(MissionMarkers.Descent_End);
fprintf('| %-12s | %6.1f s     | %8.1f m      |\n', 'LANDING', T_LA, X_LA);

fprintf('=================================================================\n');
fprintf('TOTAL MISSION TIME: %.2f min\n', time(i)/60);
fprintf('TOTAL RANGE:        %.2f km\n', x_E(i)/1000);
fprintf('=================================================================\n');

%% ========================================================================
%  13. DATA POST-PROCESSING
%  ========================================================================
fprintf('Phase: Simulation Complete. Processing Data...\n');

% --- Unpack State Variables ---
States           = [SimLog.States]; 
alpha            = [States.Alpha];
gamma            = [States.Gamma];
delta_e          = [States.Delta_e];         
throttle_setting = [States.Throttle_Setting];
SFC              = [States.SFC];
V                = [States.V];
Mach_Number      = [States.Mach_Number];
u_dot_E          = [States.u_dot_E];         
w_dot_E          = [States.w_dot_E];         

% --- Unpack Forces & Moments ---
Forces  = [SimLog.Forces]; 
W       = [Forces.W];
L       = [Forces.L];
D       = [Forces.D];
T       = [Forces.T];
N       = [Forces.N];
Moments = [SimLog.Moments]; 
M_Aero  = [Moments.M_Aero];

% --- Unpack Derivatives ---
Dervs   = [SimLog.Derivatives];
x_dot_E = [Dervs.x_dot_E];
z_dot_E = [Dervs.z_dot_E]; 
m_dot   = [Dervs.m_dot]; 

%% ========================================================================
%  14. VISUALIZATION
%  ========================================================================
time_h = time/3600; % Convert seconds to hours

figure('Name', 'Mission Profile Analysis', 'Color', [0.1 0.1 0.1]);
t = tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, 'T-38 Talon - Mission Profile Analysis', 'Color', 'w');

% --- Attitudes & Control ---
nexttile; plot(time_h, rad2deg(alpha), 'LineWidth', 1.5);
title('Angle of Attack (\alpha)'); ylabel('deg'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, rad2deg(theta), 'LineWidth', 1.5);
title('Pitch Angle (\theta)'); ylabel('deg'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, rad2deg(gamma), 'LineWidth', 1.5);
title('Flight Path Angle (\gamma)'); ylabel('deg'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, rad2deg(delta_e), 'LineWidth', 1.2);
title('Elevator Deflection'); ylabel('deg'); xlabel('Time (h)'); grid on; grid minor;

% --- Dynamics (Forces) ---
nexttile; plot(time_h, L, time_h, D, time_h, W, time_h, T, time_h, N);
title('Forces'); legend('Lift', 'Drag', 'Weight', 'Thrust', 'Normal'); 
xlabel('Time (h)'); ylabel('Force (N)'); grid on; grid minor; 

nexttile; plot(time_h, rad2deg(M_Aero));
title('Aerodynamic Moment'); ylabel('N*m'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, SFC*1000);
title('Specific Fuel Consumption (SFC)'); ylabel('g/(s*kN)'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, -m_dot);
title('Total Fuel Flow (Mass Rate)'); ylabel('kg/s'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, throttle_setting, 'LineWidth', 1.2);
title('Throttle Setting (PID Output)'); ylabel('%'); xlabel('Time (h)'); ylim([0 1.1]); grid on; grid minor;

% --- Kinematics & Performance ---
nexttile; plot(time_h, u_dot_E);
title('Longitudinal Accel (X_E)'); ylabel('m/s^2'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, -w_dot_E); 
title('Vertical Accel (Z_E)'); ylabel('m/s^2'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, x_dot_E);
title('Ground Speed'); ylabel('m/s'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, -z_dot_E); 
title('Climb Rate (ROC)'); ylabel('m/s'); xlabel('Time (h)'); grid on; grid minor;

% --- Flight Envelop ---
nexttile; plot(time_h, V, 'LineWidth', 1.5);
title('True Airspeed (TAS)'); ylabel('m/s'); xlabel('Time (h)'); grid on; grid minor;

nexttile; plot(time_h, Mach_Number, 'LineWidth', 1.5);
title('Mach Number'); ylabel('M'); xlabel('Time (h)'); grid on; grid minor;

% --- Trajectory ---
nexttile([1 2]); 
plot(x_E/1000, -z_E, 'LineWidth', 2); 
title('Mission Trajectory (Profile View)'); 
xlabel('Distance (km)'); ylabel('Altitude (m)'); 
xlim([0, max(x_E/1000)*1.01]); 
grid on; grid minor;

%% ========================================================================
%  INTERACTIVE POP-OUT MODULE
% =========================================================================
%  Description: 
%     Enables "Click-to-Expand" functionality for all subplots.
%     Clicking on any specific graph opens it in a new, dedicated window 
%     for detailed inspection without affecting the main dashboard.
% =========================================================================

% 1. Retrieve all axes handles within the active figure
all_axes = findall(gcf, 'Type', 'axes');

% 2. Assign the callback function to each axis and its children (lines)
for i = 1:length(all_axes)
    % Attach listener to the axis background
    all_axes(i).ButtonDownFcn = @expandChart;
    
    % Attach listener to the plotted data elements (lines, patches, etc.)
    % This ensures the click registers even if the user clicks directly on a line.
    set(all_axes(i).Children, 'ButtonDownFcn', @expandChart); 
end

fprintf('>> VISUALIZATION: Interactive Mode Enabled. Click on any plot to inspect in detail.\n');

% =========================================================================
%  LOCAL FUNCTION: EXPAND CHART CALLBACK
% =========================================================================
function expandChart(src, ~)
    % expandChart - Callback function to handle mouse clicks on plots.
    %
    % Inputs:
    %   src : The graphic object that was clicked (Axis object or Primitive).
    %   ~   : Event data (unused).
    
    % --- 1. Identify Target Axis ---
    % If the user clicks on a Line/Surface, get its Parent (the Axis).
    % If the user clicks on the whitespace, the src is already the Axis.
    if isa(src, 'matlab.graphics.chart.primitive.Line') || ...
       isa(src, 'matlab.graphics.chart.primitive.Surface') || ...
       isa(src, 'matlab.graphics.chart.primitive.Patch')
        target_ax = src.Parent;
    else
        target_ax = src;
    end
    
    % --- 2. Create New Detailed View Window ---
    % Initialize a new figure with Dark Mode styling to match the theme.
    f_new = figure('Color', [0.15 0.15 0.15], ...
                   'Name', 'Detailed Analysis View', ...
                   'NumberTitle', 'off', ...
                   'MenuBar', 'figure', ... % Keep standard menu for saving/printing
                   'ToolBar', 'auto', ...
                   'Units', 'normalized', ...
                   'Position', [0.15 0.15 0.7 0.7]); % Center screen, 70% scale
    
    % --- 3. Clone the Graph ---
    % Copy the target axis object into the new figure.
    ax_new = copyobj(target_ax, f_new);
    
    % --- 4. Adjust Layout & Styling ---
    % Reset position to fill the new figure (detach from tiled layout constraints)
    ax_new.Position = [0.10 0.12 0.85 0.80];
    
    if isprop(ax_new, 'Layout')
        ax_new.Layout = []; % Clear previous tile configuration
    end
    
    % Enhance visibility for detailed inspection
    ax_new.FontSize = 13; % Larger font for readability
    if ~isempty(ax_new.Title)
        ax_new.Title.FontSize = 15;
        ax_new.Title.Color = [1 1 1]; % Ensure title is white
    end
    
    % Preserve high-contrast colors
    ax_new.XColor = [0.9 0.9 0.9];
    ax_new.YColor = [0.9 0.9 0.9];
    ax_new.GridAlpha = 0.3;
    ax_new.MinorGridAlpha = 0.1;
    
    % Force grids on
    grid(ax_new, 'on'); 
    grid(ax_new, 'minor');
end

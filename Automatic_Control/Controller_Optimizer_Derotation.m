%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:  DEROTATION TUNER (NOSE GEAR LOWERING)
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script optimizes the "Derotation" phase, which occurs immediately
%     after main gear touchdown. The aircraft pivots around the main wheels,
%     acting as an inverted pendulum controlled by the elevator.
%
%     Objective:
%     - Lower the nose landing gear (NLG) to the runway gently.
%     - Impact Pitch Rate (q) must be less than 3 deg/s to prevent structural
%       damage (NLG Collapse).
%     - Prevent "Floating": The nose must drop before aerodynamic control
%       authority is lost due to decelerating airspeed.
%
%     Control Strategy (Active Damping):
%     - The elevator is used to generate a nose-up moment that opposes the 
%       gravity-induced nose drop, cushioning the impact.
%
%  DEPENDENCIES:
%     - FlightDynamics.m
%     - RunPID.m
%     - Derotation_Checkpoint.mat (Generated at Touchdown)
% ========================================================================
clearvars; clc; close all;

%% ========================================================================
%  1. INITIALIZATION & CONFIGURATION
%  ========================================================================
fprintf('----------------------------------------------------------\n');
fprintf('>> T-38 DEROTATION TUNER (NOSE CUSHION) INITIATED.\n');
fprintf('----------------------------------------------------------\n');

if ~isfile('Derotation_Checkpoint.mat')
    error('>> ERROR: Checkpoint file "Derotation_Checkpoint.mat" not found.');
end
load('Derotation_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint Loaded. Touchdown Time: %.2f s\n', Checkpoint.Time);

% --- TUNER TARGETS ---
TunerConfig.Target_Theta    = deg2rad(-0.1); % [rad] Geometric Zero (Nose on ground)
TunerConfig.Impact_Limit_Q  = -0.05;         % [rad/s] ~3 deg/s (Structural Limit)

% --- OPTIMIZATION VECTOR ---
% G(1): Kp (Proportional) -> Position Error Gain (Guide to 0)
% G(2): Ki (Integral)     -> Locked to 0 (Not needed for transient event)
% G(3): Kd (Derivative)   -> Damping Gain (The "Shock Absorber")
% Initial Guess: High Kd expected to fight gravity acceleration.
InitialGuess = [-1.5, 0.0, -4.0]; 

% Solver Options
Opts = optimset('Display', 'iter', ...
                'MaxIter', 500, ...
                'TolX', 1e-4, ...
                'TolFun', 1e-4);

%% ========================================================================
%  2. EXECUTE OPTIMIZATION
%  ========================================================================
fprintf('>> Starting Optimization Loop (Objective: Soft Nose Impact)...\n');

CostFunc = @(Gains) EvaluateDerotationPerformance(Gains, Checkpoint, TunerConfig);

[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGains, Opts);

%% ========================================================================
%  3. REPORT RESULTS
%  ========================================================================
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION COMPLETE.\n');
fprintf('==========================================================\n');
fprintf('Final Cost: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('DEROTATION CONTROLLER (Active Damping):\n');
fprintf('   Kp: %8.5f [Position Guide]\n', OptimalGains(1));
fprintf('   Ki: %8.5f [Locked]\n', 0.0);
fprintf('   Kd: %8.5f [Virtual Damper]\n', OptimalGains(3));
fprintf('==========================================================\n');

VisualizeResult(OptimalGains, Checkpoint, TunerConfig);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateDerotationPerformance(Gains, CP, Config)

    % --- 1. SETUP CONTROLLER ---
    C.Kp  = Gains(1); 
    C.Ki  = 0;          % Integrator not useful for this short dynamic event
    C.Kd  = Gains(3);
    
    % Authority Limits
    C.Max = deg2rad(10);  % Don't pitch up excessively (tailstrike risk)
    C.Min = -deg2rad(20); % Full trailing edge up authority available
    
    % --- 2. SIMULATION SETUP ---
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = CP.dt;
    PIDState = struct('Integrator', 0, 'PrevError', 0);
    
    % Configuration for Ground Roll
    Controls = struct('ElevatorDeflection',0, ...
                      'ThrottleSetting',   0.0, ... % Idle
                      'Gear',              1.0, ... % Down
                      'SpeedBrake',        1.0);    % Deployed (Max Drag)
    
    Step = 0; 
    MaxSteps = 600; % Time-out limit
    
    Total_Tracking_Error = 0;
    Saturation_Penalty   = 0;
    
    % --- 3. PHYSICS LOOP (PIVOT DYNAMICS) ---
    % Run until Nose Gear touches ground (Theta <= 0)
    while State.theta > 0 && Step < MaxSteps
        Step = Step + 1;
        
        % [Sensors]
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        
        % [Control Law]
        [Delta_Cmd, PIDState] = RunPID(Config.Target_Theta, State.theta, dt, ...
                                       C, PIDState, State.q);
        Controls.ElevatorDeflection = Delta_Cmd;
        
        % [Physics]
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        % [Integration - Ground Constraints]
        State.x_E   = State.x_E   + Derv.x_dot_E * dt;
        
        % CONSTRAINT: Main Gear is pivot point. z_E locked to 0.
        State.z_E   = 0; 
        
        % Deceleration
        State.u     = State.u     + Derv.u_dot * dt;
        
        % Rotational Dynamics (The Inverted Pendulum)
        State.theta = State.theta + Derv.theta_dot * dt;
        State.q     = State.q     + Derv.q_dot * dt;
        State.m     = State.m     + Derv.m_dot * dt;
        
        % Update Velocity Vector based on Geometry
        V_ground = sqrt(State.u^2); 
        State.u  = V_ground * cos(State.theta);
        State.w  = V_ground * sin(State.theta); % Induced vertical velocity at CG
        
        % --- COST ACCUMULATION ---
        % 1. Tracking Error (Guide to 0)
        Total_Tracking_Error = Total_Tracking_Error + abs(State.theta - 0); 
        
        % 2. Actuator Saturation (Smoothness)
        if abs(Delta_Cmd) >= (deg2rad(20) - 0.01)
            Saturation_Penalty = Saturation_Penalty + 10; 
        end
    end
    
    % --- 4. IMPACT ANALYSIS (THE SLAM BARRIER) ---
    Impact_Q = State.q; % Pitch rate at the moment of contact [rad/s]
    
    Slam_Penalty = 0;
    % CONSTRAINT: Impact Pitch Rate must be > -0.05 rad/s (-3 deg/s)
    % Note: Q is negative when pitching down.
    if Impact_Q < Config.Impact_Limit_Q
        % Exponential Penalty for breaking the nose gear
        Slam_Penalty = 1e6 * abs(Impact_Q - Config.Impact_Limit_Q);
    end
    
    % Timeout Penalty (Floating Nose)
    Time_Penalty = 0;
    if Step >= MaxSteps
        Time_Penalty = 10000; 
    end
    
    J = Total_Tracking_Error + Saturation_Penalty + Slam_Penalty + Time_Penalty;
end

%% ========================================================================
%  LOCAL FUNCTION: VISUALIZATION
%  ========================================================================
function VisualizeResult(Gains, CP, Config)
    fprintf('>> Generating Visualization...\n');
    
    % Reconstruct Controller
    C.Kp = Gains(1); C.Ki = 0; C.Kd = Gains(3);
    C.Max = deg2rad(10); C.Min = -deg2rad(20); 
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = CP.dt;
    PIDState = struct('Integrator', 0, 'PrevError', 0);
    Controls = struct('ElevatorDeflection',0,'ThrottleSetting',0.0,'Gear',1.0,'SpeedBrake',1.0);
    
    T_Hist = []; Theta_Hist = []; Q_Hist = []; Elev_Hist = [];
    Step = 0;
    
    % Re-Simulate
    while State.theta > 0 && Step < 600
        Step = Step + 1;
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        
        [Delta_Cmd, PIDState] = RunPID(Config.Target_Theta, State.theta, dt, C, PIDState, State.q);
        Controls.ElevatorDeflection = Delta_Cmd;
        
        Log = FlightDynamics(State, AC, Env, Controls); Derv = Log.Derivatives;
        
        State.x_E   = State.x_E   + Derv.x_dot_E * dt;
        State.z_E   = 0; 
        State.u     = State.u     + Derv.u_dot * dt;
        State.theta = State.theta + Derv.theta_dot * dt;
        State.q     = State.q     + Derv.q_dot * dt;
        
        V_ground = sqrt(State.u^2);
        State.u  = V_ground * cos(State.theta);
        State.w  = V_ground * sin(State.theta);
        
        T_Hist(Step)     = Step*dt;
        Theta_Hist(Step) = rad2deg(State.theta);
        Q_Hist(Step)     = State.q;
        Elev_Hist(Step)  = rad2deg(Delta_Cmd);
    end
    
    % --- PLOTTING ---
    figure('Color','w', 'Name', 'Derotation Analysis');
    
    
    % 1. Pitch Attitude
    subplot(3,1,1); 
    plot(T_Hist, Theta_Hist, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); grid on;
    ylabel('Pitch (deg)'); title('Nose Drop Trajectory');
    
    % 2. Pitch Rate (Impact Force)
    subplot(3,1,2); 
    plot(T_Hist, Q_Hist, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); grid on;
    yline(Config.Impact_Limit_Q, '--r', 'Structural Limit (-3 deg/s)', 'LineWidth', 1.5); 
    ylabel('Pitch Rate q (rad/s)');
    title(sprintf('Impact Severity: %.4f rad/s', Q_Hist(end)));
    
    % 3. Elevator (The "Cushion")
    subplot(3,1,3); 
    plot(T_Hist, Elev_Hist, 'LineWidth', 1.5, 'Color', [0.9290 0.6940 0.1250]); grid on;
    ylabel('Elevator (deg)'); xlabel('Time (s)');
    title('Active Damping Control Input');
end
%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:  APPROACH TUNER (GAMMA TRACKING + ALPHA PROTECTION)
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script tunes the "Constrained Approach" autopilot. Unlike cruise,
%     approach requires precise Flight Path Angle (Gamma) tracking to intercept
%     the glide slope, while strictly respecting Angle of Attack (Alpha) limits.
%
%     Control Strategy:
%     1. Gamma Controller (Outer Loop):
%        - Input: Gamma Error (Target -7 deg)
%        - Output: Pitch Attitude Reference (Theta_Ref)
%
%     2. Pitch Controller (Inner Loop):
%        - Stabilizes the aircraft to the Theta_Ref.
%
%     3. Energy Management (Envelope Protection):
%        - Rule-Based Logic overrides Throttle and Speed Brakes based on Alpha.
%        - IF Alpha > 12 deg: Max Throttle (Stall Prevention).
%        - IF Alpha < 10 deg: Deploy Speed Brakes (Drag Management).
%
%  DEPENDENCIES:
%     - FlightDynamics.m
%     - RunPID.m
%     - Approach_Checkpoint.mat (Generated at end of Descent Phase)
% ========================================================================
clearvars; clc; close all;

%% ========================================================================
%  1. INITIALIZATION & CONFIGURATION
%  ========================================================================
fprintf('----------------------------------------------------------\n');
fprintf('>> T-38 APPROACH CONTROLLER TUNER INITIATED.\n');
fprintf('----------------------------------------------------------\n');

if ~isfile('Approach_Checkpoint.mat')
    error('>> ERROR: Checkpoint file "Approach_Checkpoint.mat" not found.');
end
load('Approach_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint Loaded. Simulation Time: %.2f s\n', Checkpoint.Time);

% --- TUNER TARGETS ---
TunerConfig.Target_Gamma = deg2rad(-7.0); % [rad] Steep approach path
TunerConfig.Max_Alpha    = deg2rad(12.0); % [rad] Stall Warning Limit

% --- INHERITED GAINS (FROZEN) ---
% Pitch Inner Loop gains from previous phases.
Inherited.Pitch.Kp = -35.6016;   
Inherited.Pitch.Ki =  0.0244;   
Inherited.Pitch.Kd =  24.3107;   

% --- OPTIMIZATION VECTOR ---
% G(1-3): Gamma PID [Kp, Ki, Kd]
% Note: Gamma response is slower than Pitch but faster than Altitude.
InitialGuess = [0.5, 0.05, 0.3]; 

% Solver Options
Opts = optimset('Display', 'iter', ...
                'MaxIter', 400, ...
                'TolX', 1e-4, ...
                'TolFun', 1e-4);

%% ========================================================================
%  2. EXECUTE OPTIMIZATION
%  ========================================================================
fprintf('>> Starting Optimization Loop (Constraint: Alpha < 12 deg)...\n');

CostFunc = @(Gains) EvaluateApproachPerformance(Gains, Checkpoint, TunerConfig, Inherited);

[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGuess, Opts);

%% ========================================================================
%  3. REPORT RESULTS
%  ========================================================================
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION COMPLETE.\n');
fprintf('==========================================================\n');
fprintf('Final Cost: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('GAMMA CONTROLLER (Outer Loop):\n');
fprintf('   Kp: %8.6f [Theta_Ref per Radian Gamma]\n', OptimalGains(1));
fprintf('   Ki: %8.6f\n', OptimalGains(2));
fprintf('   Kd: %8.6f\n', OptimalGains(3));
fprintf('==========================================================\n');

VisualizePerformance(OptimalGains, Checkpoint, TunerConfig, Inherited);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateApproachPerformance(Gains, CP, Config, FixedGains)

    % --- 1. SETUP CONTROLLERS ---
    % [A] Gamma Controller (Tuning Variables)
    C.Gamma.Kp  = Gains(1); 
    C.Gamma.Ki  = Gains(2); 
    C.Gamma.Kd  = Gains(3);
    % Limit the Authority: The autopilot cannot request extreme pitch attitudes
    % just to fix the flight path.
    C.Gamma.Max = deg2rad(5.0);  
    C.Gamma.Min = deg2rad(-10.0);

    % [B] Pitch Controller (Fixed / Inner Loop)
    C.Pitch.Kp  = FixedGains.Pitch.Kp;
    C.Pitch.Ki  = FixedGains.Pitch.Ki;
    C.Pitch.Kd  = FixedGains.Pitch.Kd;
    C.Pitch.Max = deg2rad(20); 
    C.Pitch.Min = -deg2rad(20);

    % --- 2. FEASIBILITY CHECKS ---
    % Physics Check: Gamma Kp must be POSITIVE.
    % Logic: Target Gamma (-7) < Current Gamma (-5) -> Error is Negative.
    % To Descend more, we need to Pitch Down (Negative Theta).
    % Therefore: Neg Error * Pos Gain = Neg Output.
    if C.Gamma.Kp < 0; J = 1e15; return; end

    % --- 3. SIMULATION INITIALIZATION ---
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; 
    dt = 0.01; % High precision for approach physics
    
    GammaState = struct('Integrator', 0, 'PrevError', 0);
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    
    Total_Gamma_Error = 0; 
    Stall_Penalty     = 0;
    
    MaxSteps = 1000; % 10 seconds evaluation window
    
    % --- 4. FLIGHT LOOP ---
    for k = 1:MaxSteps
        % [Sensors]
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        Current_Gamma = atan2(-State.w, State.u);
        
        % Calculate Alpha (Theta = Gamma + Alpha) -> Alpha = Theta - Gamma
        Current_Alpha_Rad = State.theta - Current_Gamma;
        Current_Alpha_Deg = rad2deg(Current_Alpha_Rad);
        
        % --- CASCADE CONTROL LOGIC ---
        
        % 1. OUTER LOOP: Gamma -> Pitch Reference
        [Ref_Theta, GammaState] = RunPID(Config.Target_Gamma, Current_Gamma, dt, ...
                                         C.Gamma, GammaState);
        
        % Safety Limiter (Saturate the Pitch Command)
        Ref_Theta = max(min(Ref_Theta, C.Gamma.Max), C.Gamma.Min);
        
        % 2. INNER LOOP: Pitch Reference -> Elevator
        [Elevator, PitchState] = RunPID(Ref_Theta, State.theta, dt, ...
                                        C.Pitch, PitchState, State.q);
        
        % 3. ENERGY MANAGEMENT (ALPHA PROTECTION)
        % This is a "Rule-Based" supervisor that overrides throttle/brakes.
        Throttle_Cmd   = 0.0; 
        SpeedBrake_Cmd = 0.5;
        
        % High Alpha Protection (Stall Prevention)
        if Current_Alpha_Deg > 12.0
            Throttle_Cmd = 1.0; % Max Power
            % Punish the optimizer heavily if we hit this condition.
            % We want the PID to fly smooth enough to avoid this emergency.
            Stall_Penalty = Stall_Penalty + 500; 
        elseif Current_Alpha_Deg > 11.5
            Throttle_Cmd = 0.5; % Pre-emptive Power
        end
        
        % Low Alpha (Drag Management)
        if Current_Alpha_Deg < 10.0
            SpeedBrake_Cmd = 1.0; % Add Drag to increase Alpha
        elseif Current_Alpha_Deg > 11.0
            SpeedBrake_Cmd = 0.0; % Remove Drag
        end
        
        % [Actuator Output]
        Controls.ElevatorDeflection = Elevator;
        Controls.ThrottleSetting    = Throttle_Cmd;
        Controls.SpeedBrake         = SpeedBrake_Cmd;
        Controls.Gear               = 1.0; 
        
        % [Physics]
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        % [Integration]
        State.x_E   = State.x_E   + Derv.x_dot_E*dt;
        State.z_E   = State.z_E   + Derv.z_dot_E*dt;
        State.u     = State.u     + Derv.u_dot*dt;
        State.w     = State.w     + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt;
        State.q     = State.q     + Derv.q_dot*dt;
        
        % [Cost Function]
        Total_Gamma_Error = Total_Gamma_Error + abs(Config.Target_Gamma - Current_Gamma) * 100;
        
        % Ground Collision Check
        if State.z_E > 0; J = 1e15; return; end
    end
    
    J = Total_Gamma_Error + Stall_Penalty;
end

%% ========================================================================
%  LOCAL FUNCTION: VISUALIZATION
%  ========================================================================
function VisualizePerformance(Gains, CP, Config, FixedGains)
    fprintf('>> Generating Visualization...\n');
    
    % Reconstruct Controllers
    C.Gamma.Kp = Gains(1); C.Gamma.Ki = Gains(2); C.Gamma.Kd = Gains(3);
    C.Gamma.Max = deg2rad(5.0); C.Gamma.Min = deg2rad(-10.0);
    C.Pitch = FixedGains.Pitch;
    C.Pitch.Max = deg2rad(20); C.Pitch.Min = -deg2rad(20);
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.01;
    GammaState = struct('Integrator', 0, 'PrevError', 0);
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    
    Hist_T = []; Hist_Gamma = []; Hist_Alpha = []; Hist_Alt = [];
    
    for k = 1:1000
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        Current_Gamma = atan2(-State.w, State.u);
        Current_Alpha_Deg = rad2deg(State.theta - Current_Gamma);
        
        [Ref_Theta, GammaState] = RunPID(Config.Target_Gamma, Current_Gamma, dt, C.Gamma, GammaState);
        Ref_Theta = max(min(Ref_Theta, C.Gamma.Max), C.Gamma.Min);
        [Elevator, PitchState] = RunPID(Ref_Theta, State.theta, dt, C.Pitch, PitchState, State.q);
        
        % Protection Logic Replay
        Throttle_Cmd = 0.0; SpeedBrake_Cmd = 0.5;
        if Current_Alpha_Deg > 12.0, Throttle_Cmd = 1.0;
        elseif Current_Alpha_Deg > 11.5, Throttle_Cmd = 0.5; end
        if Current_Alpha_Deg < 10.0, SpeedBrake_Cmd = 1.0;
        elseif Current_Alpha_Deg > 11.0, SpeedBrake_Cmd = 0.0; end
        
        Controls.ElevatorDeflection = Elevator; Controls.ThrottleSetting = Throttle_Cmd;
        Controls.SpeedBrake = SpeedBrake_Cmd; Controls.Gear = 1.0;
        
        Log = FlightDynamics(State, AC, Env, Controls); Derv = Log.Derivatives;
        State.x_E = State.x_E + Derv.x_dot_E*dt; State.z_E = State.z_E + Derv.z_dot_E*dt;
        State.u = State.u + Derv.u_dot*dt; State.w = State.w + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt; State.q = State.q + Derv.q_dot*dt;
        
        Hist_T(k)     = k*dt;
        Hist_Gamma(k) = rad2deg(Current_Gamma);
        Hist_Alpha(k) = Current_Alpha_Deg;
        Hist_Alt(k)   = -State.z_E;
        
        if State.z_E > 0, break; end
    end
    
    figure('Name','Approach Optimization Result','Color','w');
    
    % 1. Flight Path Angle
    subplot(3,1,1); 
    plot(Hist_T, Hist_Gamma, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); grid on;
    yline(rad2deg(Config.Target_Gamma), '--r', 'Target Gamma (-7 deg)', 'LineWidth', 1.5); 
    ylabel('Gamma (deg)'); title('Flight Path Tracking');
    
    % 2. Alpha Protection (Crucial)
    subplot(3,1,2); 
    plot(Hist_T, Hist_Alpha, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); grid on;
    yline(12, '--k', 'Stall Warning (12 deg)', 'LineWidth', 1.5); 
    ylabel('Alpha (deg)'); title('AoA Envelope Protection');
    
    % 3. Altitude Profile
    subplot(3,1,3); 
    plot(Hist_T, Hist_Alt, 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880]); grid on;
    ylabel('Altitude (m)'); xlabel('Time (s)'); title('Approach Descent Profile');
end
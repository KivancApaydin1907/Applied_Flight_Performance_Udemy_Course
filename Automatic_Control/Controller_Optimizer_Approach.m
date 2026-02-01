%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:  APPROACH TUNER (GAMMA TRACKING + ALPHA PROTECTION)
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script tunes the "Constrained Approach" autopilot. 
%     - Target: Track -4 deg Gamma (Flight Path Angle).
%     - Constraint: Keep Alpha below 12 deg.
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
TunerConfig.Target_Gamma = deg2rad(-4.0); % [rad] Safe Approach Angle (-4 deg)
TunerConfig.Max_Alpha    = deg2rad(12.0); % [rad] Stall Warning Limit

% --- INHERITED GAINS (Inner Loop - Pitch) ---
Inherited.Pitch.Kp = -35.0134;   
Inherited.Pitch.Ki =  0.0245;   
Inherited.Pitch.Kd =  24.3466;   

% --- OPTIMIZATION VECTOR ---
% G(1-3): Gamma PID [Kp, Ki, Kd]
% Updated Initial Guess to start searching in the "Right Direction"
InitialGuess = [1.5, 0.1, 0.5]; 

% Solver Options
Opts = optimset('Display', 'iter', ...
                'MaxIter', 400, ...
                'TolX', 1e-4, ...
                'TolFun', 1e-4);

%% ========================================================================
%  2. EXECUTE OPTIMIZATION
%  ========================================================================
fprintf('>> Starting Optimization Loop (Objective: Track Gamma -4, Min Error)...\n');
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
    C.Gamma.Kp  = Gains(1); 
    C.Gamma.Ki  = Gains(2); 
    C.Gamma.Kd  = Gains(3);
    C.Gamma.Max = deg2rad(5.0);  
    C.Gamma.Min = deg2rad(-10.0);
    
    C.Pitch     = FixedGains.Pitch;
    C.Pitch.Max = deg2rad(20); 
    C.Pitch.Min = -deg2rad(20);
    
    % --- 2. FEASIBILITY CHECKS ---
    % 1. Kp must be POSITIVE (Neg Error -> Pitch Down -> Neg Theta)
    % 2. Ki must be POSITIVE (Accumulated Error logic)
    if C.Gamma.Kp < 0 || C.Gamma.Ki < 0 
        J = 1e15; return; 
    end
    
    % --- 3. SIMULATION INITIALIZATION ---
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; 
    dt = 0.01; 
    
    GammaState = struct('Integrator', 0, 'PrevError', 0);
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    
    Total_Gamma_Error = 0; 
    Stall_Penalty     = 0;
    
    MaxSteps = 1000; % 10 seconds evaluation
    
    % --- 4. FLIGHT LOOP ---
    for k = 1:MaxSteps
        % [Sensors]
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);

        Current_Alpha_Rad = atan2(State.w, State.u); 
        Current_Gamma = State.theta - Current_Alpha_Rad;
        Current_Alpha_Deg = rad2deg(Current_Alpha_Rad);
        
        % --- CASCADE CONTROL LOGIC ---
        
        % 1. OUTER LOOP: Gamma -> Pitch Reference
        [Ref_Theta, GammaState] = RunPID(Config.Target_Gamma, Current_Gamma, dt, ...
                                         C.Gamma, GammaState);
              
        % 2. INNER LOOP: Pitch Reference -> Elevator
        [Elevator, PitchState] = RunPID(Ref_Theta, State.theta, dt, ...
                                        C.Pitch, PitchState, State.q);
        
        % 3. ALPHA PROTECTION (Rule Based)
        Throttle_Cmd   = 0.0; 
        SpeedBrake_Cmd = 0.5;
        
        % Stall Protection
        if Current_Alpha_Deg > 12.0
            Throttle_Cmd = 1.0; 
            Stall_Penalty = Stall_Penalty + 500;
        elseif Current_Alpha_Deg > 11.5
            Throttle_Cmd = 0.5; 
        end
        
        % Drag Management
        if Current_Alpha_Deg < 8.0
            SpeedBrake_Cmd = 1.0; 
        elseif Current_Alpha_Deg > 10.0
            SpeedBrake_Cmd = 0.0; 
        end
        
        % [Actuators]
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
        
        % [Cost Calculation]
        Total_Gamma_Error = Total_Gamma_Error + abs(Config.Target_Gamma - Current_Gamma) * 100;
        
        % Crash Check
        if State.z_E > 0; J = 1e15; return; end
    end
    
    J = Total_Gamma_Error + Stall_Penalty;
end

%% ========================================================================
%  LOCAL FUNCTION: VISUALIZATION
%  ========================================================================
function VisualizePerformance(Gains, CP, Config, FixedGains)
    fprintf('>> Generating Visualization...\n');
    
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
        
        Current_Alpha_Rad = atan2(State.w, State.u);
        Current_Gamma = State.theta - Current_Alpha_Rad;
        Current_Alpha_Deg = rad2deg(Current_Alpha_Rad);
        
        [Ref_Theta, GammaState] = RunPID(Config.Target_Gamma, Current_Gamma, dt, C.Gamma, GammaState);
        Ref_Theta = max(min(Ref_Theta, C.Gamma.Max), C.Gamma.Min);
        [Elevator, PitchState] = RunPID(Ref_Theta, State.theta, dt, C.Pitch, PitchState, State.q);
        
        Throttle_Cmd = 0.0; SpeedBrake_Cmd = 0.5;
        if Current_Alpha_Deg > 12.0, Throttle_Cmd = 1.0;
        elseif Current_Alpha_Deg > 11.5, Throttle_Cmd = 0.5; end
        if Current_Alpha_Deg < 8.0, SpeedBrake_Cmd = 1.0;
        elseif Current_Alpha_Deg > 10.0, SpeedBrake_Cmd = 0.0; end
        
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
    subplot(3,1,1); 
    plot(Hist_T, Hist_Gamma, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); grid on;
    yline(rad2deg(Config.Target_Gamma), '--r', 'Target Gamma (-4 deg)', 'LineWidth', 1.5); 
    ylabel('Gamma (deg)'); title('Flight Path Tracking');
    
    subplot(3,1,2); 
    plot(Hist_T, Hist_Alpha, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); grid on;
    yline(12, '--k', 'Stall Warning (12 deg)', 'LineWidth', 1.5); 
    ylabel('Alpha (deg)'); title('AoA Envelope Protection');
    
    subplot(3,1,3); 
    plot(Hist_T, Hist_Alt, 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880]); grid on;
    ylabel('Altitude (m)'); xlabel('Time (s)'); title('Approach Descent Profile');
end

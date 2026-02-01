%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:  CASCADE TUNER - DESCENT PHASE (CONSTANT ALPHA MODE)
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script tunes the descent autopilot which operates in a "Constant 
%     Angle of Attack (Alpha)" mode. 
%
%     Objective:
%     - Maintain a high Angle of Attack (8.0 deg) to maximize induced drag.
%     - Minimize Control Jitter (High Frequency Oscillation).
%     - Eliminate Overshoot (Integral Windup Prevention).
%     - Prevent "Ballooning" (unintended climbing).
%
%     Control Architecture (Cascade):
%     1. Outer Loop (Alpha Controller):
%        - Input: Alpha Error (Target - Actual)
%        - Output: Pitch Attitude Reference (Theta_Ref)
%     2. Inner Loop (Pitch Controller):
%        - Input: Pitch Error
%        - Output: Elevator Deflection
%        - Gains: Inherited from previous flight phases (Frozen).
%
%  DEPENDENCIES:
%     - FlightDynamics.m
%     - RunPID.m
%     - Descent_Checkpoint.mat (Generated at end of Cruise Phase)
% ========================================================================
clearvars; clc; close all;

%% ========================================================================
%  1. INITIALIZATION & CONFIGURATION
%  ========================================================================
fprintf('----------------------------------------------------------\n');
fprintf('>> T-38 DESCENT CONTROLLER TUNER (ALPHA HOLD) INITIATED.\n');
fprintf('----------------------------------------------------------\n');

if ~isfile('Descent_Checkpoint.mat')
    error('>> ERROR: Checkpoint file "Descent_Checkpoint.mat" not found.');
end
load('Descent_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint Loaded. Simulation Time: %.2f s\n', Checkpoint.Time);

% --- TUNER TARGETS ---
TunerConfig.Target_Alpha = deg2rad(8.0); % [rad] Target AoA for High Drag Descent

% --- INHERITED GAINS (FROZEN) ---
% Using the robust pitch gains obtained from the Climb/Cruise analysis.
Inherited.Pitch.Kp = -35.6016;     
Inherited.Pitch.Ki =  0.0244;   
Inherited.Pitch.Kd =  24.3107; 

% --- OPTIMIZATION VECTOR ---
% G(1-3): Alpha PID [Kp, Ki, Kd]
% Kp (Medium), Ki (Very Low -> Anti-Windup), Kd (Low -> Anti-Jitter)
InitialGuess = [0.5, 0.01, 0.1]; 

% --- SOLVER OPTIONS ---
Opts = optimset('Display', 'iter', ...
                'MaxIter', 500, ... 
                'MaxFunEvals', 2000, ...
                'TolX', 1e-4, ...
                'TolFun', 1e-4);

%% ========================================================================
%  2. EXECUTE OPTIMIZATION
%  ========================================================================
fprintf('>> Starting Optimization Loop (Minimize Tracking Error + Ballooning)...\n');
fprintf('>> Objectives: Smoothness > speed, No Overshoot.\n');

CostFunc = @(Gains) EvaluateDescentPerformance(Gains, Checkpoint, TunerConfig, Inherited);
[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGuess, Opts);

%% ========================================================================
%  3. REPORT RESULTS
%  ========================================================================
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION COMPLETE.\n');
fprintf('==========================================================\n');
fprintf('Final Cost: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('ALPHA CONTROLLER (Outer Loop):\n');
fprintf('   Kp: %8.6f [Theta_Ref per Radian Alpha]\n', OptimalGains(1));
fprintf('   Ki: %8.6f\n', OptimalGains(2));
fprintf('   Kd: %8.6f\n', OptimalGains(3));
fprintf('==========================================================\n');

VisualizePerformance(OptimalGains, Checkpoint, TunerConfig, Inherited);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateDescentPerformance(Gains, CP, Config, FixedGains)
    % --- 1. SETUP CONTROLLERS ---
    % [A] Alpha Controller (Tuning Variables)
    C.Alpha.Kp  = Gains(1); 
    C.Alpha.Ki  = Gains(2); 
    C.Alpha.Kd  = Gains(3);
    C.Alpha.Max = deg2rad(15); % Max Pitch Reference Command
    C.Alpha.Min = deg2rad(-15);
    
    % [B] Pitch Controller (Fixed / Inner Loop)
    C.Pitch = FixedGains.Pitch;
    C.Pitch.Max = deg2rad(20); 
    C.Pitch.Min = -deg2rad(20);
    
    % --- 2. FEASIBILITY CHECKS ---
    % Physics Check: Kp must be positive.
    if any(Gains < 0)
         J = 1e15; return; 
    end
    
    % --- 3. SIMULATION INITIALIZATION ---
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; 
    dt = 0.1;
    
    AlphaState = struct('Integrator', 0, 'PrevError', 0);
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    
    % Cost Accumulators
    Total_Error_Cost    = 0; 
    Climb_Penalty       = 0;     
    Oscillation_Penalty = 0; 
    Overshoot_Penalty   = 0;
    
    Start_Alt = -State.z_E; 
    MaxSteps  = 600; 
    Prev_Cmd = 0; 
    
    % --- 4. FLIGHT LOOP ---
    for k = 1:MaxSteps
        % [Sensors]
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        Current_Alpha = atan2(State.w, State.u);
        
        % [Control Logic]
        [Ref_Theta, AlphaState] = RunPID(Config.Target_Alpha, Current_Alpha, dt, C.Alpha, AlphaState);
        [Delta_Cmd, PitchState] = RunPID(Ref_Theta, State.theta, dt, C.Pitch, PitchState, State.q);
        
        % [Actuators]
        Controls.ElevatorDeflection = Delta_Cmd;
        Controls.ThrottleSetting    = 0.0; % Idle
        Controls.SpeedBrake         = 1.0; % Full Drag
        Controls.Gear               = 0.0;
        
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
        
        % --- COST CALCULATION (REVISED LOGIC) ---
        
        Error = Config.Target_Alpha - Current_Alpha;
        
        % 1. Tracking Error (Time Weighted)
        % We forgive initial errors, but punish steady-state errors heavily.
        Time_Weight = (k / MaxSteps)^2; 
        Total_Error_Cost = Total_Error_Cost + abs(Error) * 100 * Time_Weight;
        
        % 2. OVERSHOOT PENALTY (Anti-Windup)
        % If Alpha exceeds Target by 0.5 deg, apply massive penalty.
        % This forces Ki to stay low.
        if Current_Alpha > (Config.Target_Alpha + deg2rad(0.5))
            Overshoot_Penalty = Overshoot_Penalty + abs(Error) * 5000;
        end
        
        % 3. BALLOONING (Trajectory Monotonicity)
        Current_Alt = -State.z_E;
        if Current_Alt > (Start_Alt + 2) && k > 50 
            Climb_Penalty = Climb_Penalty + (Current_Alt - Start_Alt) * 500;
        end
        
        % 4. CONTROL SMOOTHNESS (Anti-Jitter)
        if k > 1
             Oscillation_Penalty = Oscillation_Penalty + abs(Delta_Cmd - Prev_Cmd) * 500;
        end
        Prev_Cmd = Delta_Cmd;
        
        % Safety Termination
        if State.z_E > 0; J = 1e15; return; end
    end
    
    J = Total_Error_Cost + Climb_Penalty + Oscillation_Penalty + Overshoot_Penalty;
end

%% ========================================================================
%  LOCAL FUNCTION: VISUALIZATION
%  ========================================================================
function VisualizePerformance(Gains, CP, Config, FixedGains)
    fprintf('>> Generating Visualization...\n');
    
    C.Alpha.Kp = Gains(1); C.Alpha.Ki = Gains(2); C.Alpha.Kd = Gains(3);
    C.Alpha.Max = deg2rad(15); C.Alpha.Min = deg2rad(-15);
    
    C.Pitch = FixedGains.Pitch;
    C.Pitch.Max = deg2rad(20); C.Pitch.Min = -deg2rad(20);
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.1;
    AlphaState = struct('Integrator', 0, 'PrevError', 0);
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    
    Hist_T = []; Hist_Alpha = []; Hist_Theta = []; Hist_Alt = [];
    
    for k = 1:600
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        Current_Alpha = atan2(State.w, State.u);
        
        [Ref_Theta, AlphaState] = RunPID(Config.Target_Alpha, Current_Alpha, dt, C.Alpha, AlphaState);
        [Delta_Cmd, PitchState] = RunPID(Ref_Theta, State.theta, dt, C.Pitch, PitchState, State.q);
        
        Controls.ElevatorDeflection = Delta_Cmd;
        Controls.ThrottleSetting = 0.0; Controls.SpeedBrake = 1.0; Controls.Gear = 0.0;
        
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        State.x_E = State.x_E + Derv.x_dot_E*dt;
        State.z_E = State.z_E + Derv.z_dot_E*dt;
        State.u   = State.u   + Derv.u_dot*dt;
        State.w   = State.w   + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt;
        State.q   = State.q   + Derv.q_dot*dt;
        
        Hist_T(k) = k*dt;
        Hist_Alpha(k) = rad2deg(Current_Alpha);
        Hist_Theta(k) = rad2deg(State.theta);
        Hist_Alt(k) = -State.z_E;
        
        if State.z_E > 0, break; end
    end
    
    figure('Name','Descent Optimization Result','Color','w');
    
    subplot(3,1,1); 
    plot(Hist_T, Hist_Alpha, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); grid on;
    yline(rad2deg(Config.Target_Alpha), '--k', 'Target AoA (8.0 deg)', 'LineWidth', 1.5); 
    ylabel('Angle of Attack \alpha (deg)'); 
    title('Descent Mode: AoA Hold Performance');
    
    subplot(3,1,2); 
    plot(Hist_T, Hist_Theta, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); grid on;
    ylabel('Pitch Attitude \theta (deg)'); 
    title('Inner Loop Response');
    
    subplot(3,1,3); 
    plot(Hist_T, Hist_Alt, 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880]); grid on;
    ylabel('Altitude (m)'); xlabel('Time (s)');
    title('Descent Trajectory');
end

%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:  MIMO AUTOMATIC TUNER - CLIMB PHASE
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script performs a Multi-Input Multi-Output (MIMO) optimization 
%     to tune the autopilot for the Climb Phase. Unlike the rotation phase, 
%     climb dynamics involve strong coupling between Pitch Attitude and 
%     Airspeed.
%
%     Optimization Targets (7 Variables):
%     1-3. Pitch Controller Gains (Kp, Ki, Kd) -> Controls Elevator
%     4-6. Speed Controller Gains (Kp, Ki, Kd) -> Controls Throttle
%     7.   Base Throttle Setting (Trim)        -> Feed-forward term
%
%     The solver minimizes a cost function comprising:
%     - Pitch Tracking Error (Reference: 10 deg)
%     - Mach Number Tracking Error (Reference: Mach 0.6)
%     - Actuator Saturation Penalty (Throttle Bang-Bang prevention)
%
%  DEPENDENCIES:
%     - FlightDynamics.m
%     - RunPID.m
%     - atmosisa.m
%     - Climb_Checkpoint.mat (Generated at the end of Rotation phase)
% ========================================================================
clearvars; clc; close all;

%% ========================================================================
%  1. INITIALIZATION & CONFIGURATION
%  ========================================================================
fprintf('----------------------------------------------------------\n');
fprintf('>> T-38 CLIMB CONTROLLER TUNER (MIMO) INITIATED.\n');
fprintf('----------------------------------------------------------\n');

if ~isfile('Climb_Checkpoint.mat')
    error('>> ERROR: Checkpoint file "Climb_Checkpoint.mat" not found.');
end
load('Climb_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint Loaded. Simulation Time: %.2f s\n', Checkpoint.Time);

% --- TUNER TARGETS ---
TunerConfig.Target_Mach  = 0.6;          % Maintain Constant Mach
TunerConfig.Target_Pitch = deg2rad(10);  % Maintain Constant Attitude

% --- OPTIMIZATION VECTOR DEFINITION ---
% The optimizer solves for a vector 'G' of length 7:
% G(1-3): Pitch [Kp, Ki, Kd]
% G(4-6): Speed [Kp, Ki, Kd]
% G(7):   Base Throttle (Trim Optimization)

% Initial Guess (Based on approximate flight physics)
% Note: Base Throttle guess is 0.6 (60%)
InitialGuess = [-18.0,  0.08, 9.2, ...    % Pitch Gains
                 533.0, -0.01, -43.0, ...  % Speed Gains
                 0.6];                     % Base Throttle

% Solver Options
Opts = optimset('Display', 'iter', ...
                'MaxIter', 400, ...
                'TolX', 1e-4, ...
                'TolFun', 1e-4);

%% ========================================================================
%  2. EXECUTE MIMO OPTIMIZATION
%  ========================================================================
fprintf('>> Starting Optimization Loop (Objective: Minimize Coupled Error)...\n');

CostFunc = @(Gains) EvaluateClimbPerformance(Gains, Checkpoint, TunerConfig);

[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGuess, Opts);

%% ========================================================================
%  3. REPORT RESULTS
%  ========================================================================
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION COMPLETE.\n');
fprintf('==========================================================\n');
fprintf('Final Cost: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('1. PITCH CONTROLLER (Attitude Hold):\n');
fprintf('   Kp: %8.4f\n', OptimalGains(1));
fprintf('   Ki: %8.4f\n', OptimalGains(2));
fprintf('   Kd: %8.4f\n', OptimalGains(3));
fprintf('----------------------------------------------------------\n');
fprintf('2. SPEED CONTROLLER (Auto-Throttle):\n');
fprintf('   Kp: %8.4f\n', OptimalGains(4));
fprintf('   Ki: %8.4f\n', OptimalGains(5));
fprintf('   Kd: %8.4f\n', OptimalGains(6));
fprintf('----------------------------------------------------------\n');
fprintf('3. TRIM OPTIMIZATION:\n');
fprintf('   Base Throttle: %6.2f%% (Feed-Forward)\n', OptimalGains(7)*100);
fprintf('==========================================================\n');

VisualizePerformance(OptimalGains, Checkpoint, TunerConfig);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateClimbPerformance(Gains, CP, Config)
    
    % --- 1. DECOMPOSE OPTIMIZATION VECTOR ---
    % Pitch Controller Setup
    C.Pitch.Kp  = Gains(1); 
    C.Pitch.Ki  = Gains(2); 
    C.Pitch.Kd  = Gains(3);
    C.Pitch.Max = deg2rad(20); 
    C.Pitch.Min = -deg2rad(20);
    
    % Speed Controller Setup
    C.Speed.Kp  = Gains(4); 
    C.Speed.Ki  = Gains(5); 
    C.Speed.Kd  = Gains(6);
    C.Speed.Max = 1.0; 
    C.Speed.Min = 0.0; 
    
    % Trim Optimization (Base Throttle)
    C.Speed.Base = Gains(7); 
    
    % --- 2. STABILITY & FEASIBILITY BARRIERS ---
    % Physics Logic Checks:
    % - Pitch Kp must be Negative (Pull stick back -> Elevator Up -> Theta Up)
    % - Speed Kp must be Positive (More throttle -> More speed)
    % - Base Throttle must be realistic [0, 1]
    if C.Pitch.Kp > 0 || C.Speed.Kp < 0 || C.Speed.Base < 0 || C.Speed.Base > 1.0
         J = 1e15; % Infinite Penalty
         return; 
    end
    
    % --- 3. SIMULATION INITIALIZATION ---
    State = CP.StateVector; 
    AC    = CP.AC; 
    Env   = CP.Env; 
    dt    = 0.1; % Use larger dt for climb (slower dynamics)
    
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    SpeedState = struct('Integrator', 0, 'PrevError', 0);
    
    Total_Pitch_Error  = 0; 
    Total_Speed_Error  = 0; 
    Saturation_Penalty = 0;
    
    MaxSteps = 600; % Approx 60 seconds of flight evaluation
    
    % --- 4. FLIGHT LOOP ---
    for k = 1:MaxSteps
        % [Sensors]
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        V_tot = sqrt(State.u^2 + State.w^2);
        Mach  = V_tot / Env.a;
        
        % [Control Law]
        [Elev_Cmd, PitchState]  = RunPID(Config.Target_Pitch, State.theta, dt, C.Pitch, PitchState, State.q);
        [Throt_Cmd, SpeedState] = RunPID(Config.Target_Mach, Mach, dt, C.Speed, SpeedState);
        
        % [Actuator Dynamics Constraint]
        % Penalize "Bang-Bang" throttle behavior (0% <-> 100% oscillation)
        % This ensures smooth engine operation.
        if Throt_Cmd >= 0.99 || Throt_Cmd <= 0.01
            Saturation_Penalty = Saturation_Penalty + 10; 
        end
        
        % [Physics Step]
        Controls.ElevatorDeflection = Elev_Cmd;
        Controls.ThrottleSetting    = Throt_Cmd;
        Controls.Gear               = 0; % Retracted
        
        Log = FlightDynamics(State, AC, Env, Controls);
        
        % [Integration]
        Derv = Log.Derivatives;
        State.x_E   = State.x_E   + Derv.x_dot_E*dt;
        State.z_E   = State.z_E   + Derv.z_dot_E*dt;
        State.u     = State.u     + Derv.u_dot*dt;
        State.w     = State.w     + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt;
        State.q     = State.q     + Derv.q_dot*dt;
        
        % [Cost Accumulation]
        % Scale errors to normalize their impact on the cost function
        Total_Pitch_Error = Total_Pitch_Error + abs(Config.Target_Pitch - State.theta) * 100;
        Total_Speed_Error = Total_Speed_Error + abs(Config.Target_Mach - Mach) * 100;
        
        % [Safety Check]
        % Check for Ground Collision (z_E > 0 implies Altitude < 0)
        if State.z_E > 0
            J = 1e15; return; 
        end
    end
    
    J = Total_Pitch_Error + Total_Speed_Error + Saturation_Penalty;
end

%% ========================================================================
%  LOCAL FUNCTION: VISUALIZATION
%  ========================================================================
function VisualizePerformance(Gains, CP, Config)
    fprintf('>> Generating Visualization...\n');
    
    % Reconstruct Controllers
    C.Pitch.Kp = Gains(1); C.Pitch.Ki = Gains(2); C.Pitch.Kd = Gains(3);
    C.Pitch.Max = deg2rad(20); C.Pitch.Min = -deg2rad(20);
    
    C.Speed.Kp = Gains(4); C.Speed.Ki = Gains(5); C.Speed.Kd = Gains(6);
    C.Speed.Max = 1.0; C.Speed.Min = 0.0; 
    C.Speed.Base = Gains(7); 
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.1; 
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    SpeedState = struct('Integrator', 0, 'PrevError', 0);
    
    % Data Recording
    Hist_T = []; 
    Hist_Pitch = []; 
    Hist_Mach = []; 
    Hist_Throttle = [];
    
    MaxSteps = 600;
    
    for k = 1:MaxSteps
        % Environment
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        V_tot = sqrt(State.u^2 + State.w^2);
        Mach  = V_tot / Env.a;
        
        % Control
        [Elev_Cmd, PitchState]  = RunPID(Config.Target_Pitch, State.theta, dt, C.Pitch, PitchState, State.q);
        [Throt_Cmd, SpeedState] = RunPID(Config.Target_Mach, Mach, dt, C.Speed, SpeedState);
        
        % Physics
        Controls.ElevatorDeflection = Elev_Cmd;
        Controls.ThrottleSetting    = Throt_Cmd;
        Controls.Gear               = 0;
        Log = FlightDynamics(State, AC, Env, Controls);
        
        % Integration
        Derv = Log.Derivatives;
        State.x_E   = State.x_E   + Derv.x_dot_E*dt;
        State.z_E   = State.z_E   + Derv.z_dot_E*dt;
        State.u     = State.u     + Derv.u_dot*dt;
        State.w     = State.w     + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt;
        State.q     = State.q     + Derv.q_dot*dt;
        
        % Store
        Hist_T(k)        = k*dt;
        Hist_Pitch(k)    = rad2deg(State.theta);
        Hist_Mach(k)     = Mach;
        Hist_Throttle(k) = Throt_Cmd * 100;
        
        if State.z_E > 0, break; end
    end
    
    % Plotting
    figure('Name','MIMO Optimization Result','Color','w');
    
    subplot(3,1,1); 
    plot(Hist_T, Hist_Pitch, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); grid on;
    yline(rad2deg(Config.Target_Pitch), '--r', 'Target (10 deg)', 'LineWidth', 1.5); 
    ylabel('Pitch (deg)'); title('Pitch Tracking Response');
    
    subplot(3,1,2); 
    plot(Hist_T, Hist_Mach, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); grid on;
    yline(Config.Target_Mach, '--r', 'Target (Mach 0.6)', 'LineWidth', 1.5); 
    ylabel('Mach'); title('Airspeed Tracking Response');
    
    subplot(3,1,3); 
    plot(Hist_T, Hist_Throttle, 'LineWidth', 1.5, 'Color', '#77AC30'); grid on;
    yline(Gains(7)*100, '--k', sprintf('Optimized Trim (%.1f%%)', Gains(7)*100), 'LineWidth', 1.5); 
    ylabel('Throttle (%)'); xlabel('Time (s)'); title('Auto-Throttle Activity');
    ylim([0 100]);
end
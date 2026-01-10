%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:  PRECISION LANDING TUNER (FLARE & TOUCHDOWN)
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script optimizes the automatic landing (Flare) logic. The objective
%     is to guide the aircraft from the approach threshold to a safe touchdown.
%
%     Control Strategy (MIMO):
%     1. Pitch Controller (Attitude Hold):
%        - Maintains a specific nose-up attitude (+6.0 deg) to ensure main
%          gear touchdown first (avoiding "wheelbarrowing").
%
%     2. Throttle Controller (Sink Rate Arrest):
%        - Modulates engine power to maintain a safe descent rate (-1.2 m/s).
%        - Prevents "Hard Landings" (Sink > -2.0 m/s) and "Floating".
%
%     Optimization Criteria:
%     - Touchdown Vertical Velocity (Minimize deviation from -1.2 m/s)
%     - Touchdown Pitch Attitude (Minimize deviation from 6.0 deg)
%
%  DEPENDENCIES:
%     - FlightDynamics.m
%     - RunPID.m
%     - Flare_Checkpoint.mat (Generated at end of Approach Phase)
% ========================================================================
clearvars; clc; close all;

%% ========================================================================
%  1. INITIALIZATION & CONFIGURATION
%  ========================================================================
fprintf('----------------------------------------------------------\n');
fprintf('>> T-38 FLARE & LANDING TUNER INITIATED.\n');
fprintf('----------------------------------------------------------\n');

if ~isfile('Flare_Checkpoint.mat')
    error('>> ERROR: Checkpoint file "Flare_Checkpoint.mat" not found.');
end
load('Flare_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint Loaded. Simulation Time: %.2f s\n', Checkpoint.Time);

% --- TUNER TARGETS ---
TunerConfig.Target_Theta    = deg2rad(6.0); % [rad] Ideal Touchdown Attitude
TunerConfig.Target_SinkRate = -1.2;         % [m/s] Ideal Touchdown Velocity
TunerConfig.Hard_Limit      = -2.5;         % [m/s] Structural Damage Limit

% --- OPTIMIZATION VECTOR ---
% G(1-3): Pitch PID [Kp, Ki, Kd]
% G(4-6): Throttle PD+Base [Kp, Kd, Base_Setting]
% Initial Guess Logic:
% - Pitch needs to be stiff (High Kp) to fight ground effect pitching moments.
% - Throttle Kp usually Negative (Sink Rate too high -> Error Negative -> Need Power).
InitialGuess = [-18.0, 0.08, 9.2, ...   % Pitch Gains
                -5.0, -1.0, 0.35];      % Throttle Gains

% Solver Options (High precision required)
Opts = optimset('Display', 'iter', ...
                'MaxIter', 500, ...
                'TolX', 1e-5, ...
                'TolFun', 1e-5);

%% ========================================================================
%  2. EXECUTE OPTIMIZATION
%  ========================================================================
fprintf('>> Starting Optimization Loop (Objective: Butter Landing)...\n');

CostFunc = @(Gains) EvaluateFlarePerformance(Gains, Checkpoint, TunerConfig);

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
fprintf('2. THROTTLE CONTROLLER (Sink Rate Arrest):\n');
fprintf('   Kp: %8.4f [Power per m/s error]\n', OptimalGains(4));
fprintf('   Kd: %8.4f\n', OptimalGains(5));
fprintf('   Base: %.4f [Feed-Forward Power]\n', OptimalGains(6));
fprintf('==========================================================\n');

VisualizePerformance(OptimalGains, Checkpoint, TunerConfig);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateFlarePerformance(Gains, CP, Config)

    % --- 1. UNPACK CONTROLLERS ---
    % Pitch Controller
    C.Pitch.Kp = Gains(1); 
    C.Pitch.Ki = Gains(2); 
    C.Pitch.Kd = Gains(3);
    C.Pitch.Max = deg2rad(25); % Allow full elevator authority
    C.Pitch.Min = -deg2rad(25);
    
    % Throttle Controller (PD + Base)
    C.Throt.Kp   = Gains(4); 
    C.Throt.Kd   = Gains(5);
    C.Throt.Base = Gains(6);
    C.Throt.Max  = 1.0; 
    C.Throt.Min  = 0.0;
    
    % --- 2. FEASIBILITY CHECKS ---
    % Pitch Kp must be Negative (Pitch Up requires negative deflection convention)
    % Throttle Base must be [0,1]
    if C.Pitch.Kp > 0 || C.Throt.Base < 0 || C.Throt.Base > 1.0
         J = 1e15; return; 
    end
    
    % --- 3. SIMULATION SETUP ---
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; 
    dt = 0.01; % High Fidelity Time Step for Landing (50 Hz)
    
    PitchState      = struct('Integrator', 0, 'PrevError', 0);
    Prev_Sink_Error = 0;
    
    MaxSteps = 1000; % 20 seconds max (prevent floating forever)
    Touchdown_Flag = false;
    Final_Sink_Rate = 0;
    
    % --- 4. LANDING LOOP ---
    for k = 1:MaxSteps
        % [Sensors]
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        
        % Vertical Speed Calculation (Inertial z_dot)
        % Note: z_dot is negative for descent.
        Current_SinkRate = -State.w * cos(State.theta) + State.u * sin(State.theta); 
        
        % --- CONTROL LOGIC ---
        
        % [A] Pitch Control (Maintain Nose High)
        [Elev_Cmd, PitchState] = RunPID(Config.Target_Theta, State.theta, dt, ...
                                        C.Pitch, PitchState, State.q);
        
        % [B] Throttle Control (Arrest Sink Rate)
        Sink_Error = Current_SinkRate - Config.Target_SinkRate;
        D_Term     = (Sink_Error - Prev_Sink_Error) / dt;
        Prev_Sink_Error = Sink_Error;
        
        % PD Control Law
        Throt_Cmd = C.Throt.Base + (C.Throt.Kp * Sink_Error) + (C.Throt.Kd * D_Term);
        Throt_Cmd = max(min(Throt_Cmd, C.Throt.Max), C.Throt.Min);
        
        % [Logic] IDLE AT TOUCHDOWN
        % Cut power when radar altimeter reads < 0.2m to ensure settling.
        if -State.z_E < 0.2, Throt_Cmd = 0.0; end
        
        % [Physics]
        Controls.ElevatorDeflection = Elev_Cmd;
        Controls.ThrottleSetting    = Throt_Cmd;
        Controls.SpeedBrake         = 0.5; 
        Controls.Gear               = 1.0;
        
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        % [Integration]
        State.x_E   = State.x_E   + Derv.x_dot_E*dt;
        State.z_E   = State.z_E   + Derv.z_dot_E*dt;
        State.u     = State.u     + Derv.u_dot*dt;
        State.w     = State.w     + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt;
        State.q     = State.q     + Derv.q_dot*dt;
        
        % --- TERMINATION CHECK (TOUCHDOWN) ---
        if -State.z_E <= 0
            Touchdown_Flag = true;
            Final_Sink_Rate = Derv.z_dot_E; % Capture impact velocity
            break; 
        end
    end
    
    % --- 5. COST CALCULATION ---
    if ~Touchdown_Flag
        J = 1e15; % Penalty for Floating (Running out of runway/time)
    else
        % Sink Rate Cost (Heavily weighted)
        Sink_Penalty = abs(Final_Sink_Rate - Config.Target_SinkRate) * 1000;
        
        % Hard Landing Barrier (Crash Prevention)
        if Final_Sink_Rate < Config.Hard_Limit
             Sink_Penalty = 1e10; 
        end
        
        % Pitch Attitude Cost (Prevent Tailstrike or Nosewheel first)
        Pitch_Penalty = abs(State.theta - Config.Target_Theta) * 500;
        
        J = Sink_Penalty + Pitch_Penalty;
    end
end

%% ========================================================================
%  LOCAL FUNCTION: VISUALIZATION
%  ========================================================================
function VisualizePerformance(Gains, CP, Config)
    fprintf('>> Generating Visualization...\n');
    
    % Reconstruct Controllers
    C.Pitch.Kp = Gains(1); C.Pitch.Ki = Gains(2); C.Pitch.Kd = Gains(3);
    C.Pitch.Max = deg2rad(25); C.Pitch.Min = -deg2rad(25);
    C.Throt.Kp = Gains(4); C.Throt.Kd = Gains(5); C.Throt.Base = Gains(6);
    C.Throt.Max = 1.0; C.Throt.Min = 0.0;
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.02;
    PitchState = struct('Integrator', 0, 'PrevError', 0); Prev_Sink_Error = 0;
    
    Hist_Alt = []; Hist_Sink = []; Hist_Theta = []; Hist_Throt = [];
    Touchdown_Idx = 0;
    
    for k = 1:500
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        Current_SinkRate = -State.w * cos(State.theta) + State.u * sin(State.theta);
        
        [Elev_Cmd, PitchState] = RunPID(Config.Target_Theta, State.theta, dt, C.Pitch, PitchState, State.q);
        
        Sink_Error = Current_SinkRate - Config.Target_SinkRate;
        D_Term = (Sink_Error - Prev_Sink_Error) / dt; Prev_Sink_Error = Sink_Error;
        Throt_Cmd = max(min(C.Throt.Base + (C.Throt.Kp * Sink_Error) + (C.Throt.Kd * D_Term), 1.0), 0.0);
        if -State.z_E < 0.2, Throt_Cmd = 0.0; end
        
        Controls.ElevatorDeflection = Elev_Cmd; Controls.ThrottleSetting = Throt_Cmd;
        Controls.SpeedBrake = 0.5; Controls.Gear = 1.0;
        Log = FlightDynamics(State, AC, Env, Controls); Derv = Log.Derivatives;
        
        State.x_E = State.x_E + Derv.x_dot_E*dt; State.z_E = State.z_E + Derv.z_dot_E*dt;
        State.u = State.u + Derv.u_dot*dt; State.w = State.w + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt; State.q = State.q + Derv.q_dot*dt;
        
        Hist_Alt(k)   = -State.z_E;
        Hist_Sink(k)  = Derv.z_dot_E; % m/s
        Hist_Theta(k) = rad2deg(State.theta);
        Hist_Throt(k) = Throt_Cmd * 100;
        
        if -State.z_E <= 0
            Touchdown_Idx = k;
            break; 
        end
    end
    
    % --- RESULTS REPORT ---
    Final_Sink = Hist_Sink(end);
    fprintf('\n-----------------------------------------------\n');
    fprintf('TOUCHDOWN REPORT:\n');
    fprintf('-----------------------------------------------\n');
    fprintf('Impact Speed (Vertical): %.2f m/s (%.0f fpm)\n', Final_Sink, Final_Sink*196.85);
    fprintf('Pitch Attitude:          %.2f deg\n', Hist_Theta(end));
    
    if Final_Sink > -1.0
        fprintf('>> GRADE: BUTTER (Excellent)\n');
    elseif Final_Sink > -1.8
        fprintf('>> GRADE: SAFE (Average)\n');
    elseif Final_Sink > -2.5
        fprintf('>> GRADE: HARD (Inspection Required)\n');
    else
        fprintf('>> GRADE: CRASH (Structural Failure)\n');
    end
    fprintf('-----------------------------------------------\n');
    
    % --- PLOTTING ---
    figure('Name','Flare & Touchdown Analysis','Color','w');
    
    % 1. Altitude (The "Flare" Shape)
    subplot(3,1,1); 
    plot(Hist_Alt, 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880]); grid on;
    ylabel('Altitude (m)'); title('Flare Trajectory (Radar Alt)');
    
    % 2. Sink Rate (The "Cushion")
    subplot(3,1,2); 
    plot(Hist_Sink, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); grid on;
    yline(Config.Target_SinkRate, '--g', 'Target (-1.2 m/s)', 'LineWidth', 1.5);
    yline(Config.Hard_Limit, '--r', 'Hard Limit', 'LineWidth', 1.5);
    ylabel('Sink Rate (m/s)'); title('Vertical Velocity Arrest');
    
    % 3. Pitch Attitude (The "Geometry")
    subplot(3,1,3); 
    plot(Hist_Theta, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); grid on;
    yline(rad2deg(Config.Target_Theta), '--k', 'Target Pitch (6.0 deg)');
    ylabel('Pitch (deg)'); title('Touchdown Attitude');
end
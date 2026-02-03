%% ========================================================================
%  PROJECT:       T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:        PRECISION AUTOLAND TUNER (FLARE & TOUCHDOWN)
%  ========================================================================
%  AUTHOR:        Kivanc Apaydin
%  DATE:          02/2026
%  PLATFORM:      MATLAB R2025b
%
%  DESCRIPTION:
%     This script utilizes the Nelder-Mead simplex algorithm to optimize 
%     MIMO (Multi-Input Multi-Output) control gains for the critical 
%     Flare and Touchdown phases.
%
%     The optimizer balances three competing objectives:
%     1. Trajectory Tracking: Achieving specific sink rate at touchdown.
%     2. Geometric Safety: Preventing tail strikes (Pitch < 13 deg).
%     3. Passenger Comfort: Minimizing control surface chatter (Jerk).
%
%  DEPENDENCIES:
%     - Flare_Checkpoint.mat (Simulation state at flare initiation)
%     - FlightDynamics.m     (Nonlinear equations of motion)
%     - RunPID.m             (Discrete PID controller implementation)
% ========================================================================
clearvars; clc; close all;

%% 1. SYSTEM INITIALIZATION & CONFIGURATION
fprintf('==========================================================\n');
fprintf('>> T-38 PRECISION LANDING OPTIMIZER INITIATED\n');
fprintf('==========================================================\n');

% --- Load Simulation Checkpoint ---
if ~isfile('Flare_Checkpoint.mat')
    error('>> FATAL ERROR: Checkpoint file "Flare_Checkpoint.mat" not found.');
end
load('Flare_Checkpoint.mat', 'Checkpoint');
fprintf('>> Simulation State Loaded Successfully.\n');

% --- Performance Targets & Safety Limits ---
TunerConfig.Targets.Pitch_deg    = 7.0;         % [deg] Landing Attitude
TunerConfig.Targets.SinkRate     = -0.6;        % [m/s] Ideal Sink Rate (Butter)
TunerConfig.Limits.SinkRate_Hard = -2.5;        % [m/s] Structural Failure Limit
TunerConfig.Limits.Pitch_Max     = 13.0;        % [deg] Tail Strike Limit
TunerConfig.Limits.Elevator_Max  = deg2rad(15); % [rad] Actuator Physical Limit

% --- Optimization Vector (Initial Guess) ---
% Structure: [Pitch_Kp, Pitch_Ki, Pitch_Kd, Throt_Kp, Throt_Kd, Throt_FFW, Throt_Ki]
InitialGuess = [-10.0,  0.01, 12.0, ...   % Pitch Controller (Attitude)
                  0.15,  1.5,  0.45, ...   % Throttle Controller (Sink Rate)
                  0.005];                  % Throttle Integrator

% --- Solver Settings ---
Opts = optimset('Display', 'iter', ...
                'MaxIter', 1200, ...
                'MaxFunEvals', 2500, ...
                'TolX', 1e-6, ...
                'TolFun', 1e-6);

%% 2. EXECUTE OPTIMIZATION ENGINE
fprintf('>> Optimization Engine Started. Minimizing Cost Function...\n');
CostFunc = @(Gains) EvaluateFlarePerformance(Gains, Checkpoint, TunerConfig);
[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGuess, Opts);

%% 3. FINAL REPORT & VISUALIZATION
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION SUCCESSFUL.\n');
fprintf('==========================================================\n');
fprintf('Final Cost Function Value: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('1. PITCH CONTROLLER (Attitude Hold):\n');
fprintf('   Kp: %8.4f | Ki: %8.4f | Kd: %8.4f\n', OptimalGains(1), OptimalGains(2), OptimalGains(3));
fprintf('----------------------------------------------------------\n');
fprintf('2. THROTTLE CONTROLLER (Vertical Speed):\n');
fprintf('   Kp: %8.4f | Ki: %8.4f | Kd: %8.4f\n', OptimalGains(4), OptimalGains(7), OptimalGains(5));
fprintf('   Feed-Forward (Bias): %.4f\n', OptimalGains(6));
fprintf('==========================================================\n');

VisualizePerformance(OptimalGains, Checkpoint, TunerConfig);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateFlarePerformance(Gains, CP, Config)
    % 1. Gain Unpacking & Constraints
    C.Pitch.Kp = Gains(1); C.Pitch.Ki = Gains(2); C.Pitch.Kd = Gains(3);
    C.Pitch.Max = Config.Limits.Elevator_Max; 
    C.Pitch.Min = -Config.Limits.Elevator_Max;
    
    C.Throt.Kp   = Gains(4); C.Throt.Ki = Gains(7); 
    C.Throt.Kd   = Gains(5); C.Throt.Base = Gains(6);
    C.Throt.Max  = 1.0; C.Throt.Min = 0.0;
    
    % 2. Stability Guards (Hard Constraints)
    % Penalize unstable regions immediately to save computation time
    if C.Pitch.Kp > -0.5 || C.Throt.Base < 0.2 || C.Throt.Base > 0.8 || C.Throt.Ki < 0
         J = 1e15; return; 
    end
    
    % 3. Simulation Initialization
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.01;
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    ThrotState = struct('Integrator', 0, 'PrevError', 0);
    
    MaxSteps = 1500; 
    Touchdown_Flag = false;
    Total_Control_Effort = 0;
    Prev_Throttle = 0; Prev_Elevator = 0;
    
    % 4. Time-Domain Simulation Loop
    for k = 1:MaxSteps
        % Sensor Feedback (Kinematic Sink Rate)
        Current_SinkRate = -State.w * cos(State.theta) + State.u * sin(State.theta); 
        
        % Control Law Execution
        Target_Theta_Rad = deg2rad(Config.Targets.Pitch_deg);
        [Delta_Cmd, PitchState] = RunPID(Target_Theta_Rad, State.theta, dt, C.Pitch, PitchState, State.q);
        [Throttle_Cmd, ThrotState] = RunPID(Config.Targets.SinkRate, Current_SinkRate, dt, C.Throt, ThrotState);
        
        % Logic: Retard Throttle at Radar Altimeter Minimum (Flare Chop)
        if -State.z_E < 0.3, Throttle_Cmd = 0.0; end
        
        % Calculate Control Effort (Smoothness/Jerk Metric)
        if k > 1
            Effort_Throt = abs(Throttle_Cmd - Prev_Throttle) * 50;
            Effort_Elev  = abs(Delta_Cmd - Prev_Elevator) * 10;
            Total_Control_Effort = Total_Control_Effort + Effort_Throt + Effort_Elev;
        end
        Prev_Throttle = Throttle_Cmd; Prev_Elevator = Delta_Cmd;
        
        % Actuator & Physics Update
        Controls.ElevatorDeflection = Delta_Cmd;
        Controls.ThrottleSetting    = Throttle_Cmd;
        Controls.SpeedBrake         = 0.5; 
        Controls.Gear               = 1.0;
        
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        % Numerical Integration (Euler)
        State.x_E   = State.x_E + Derv.x_dot_E*dt; 
        State.z_E   = State.z_E + Derv.z_dot_E*dt;
        State.u     = State.u + Derv.u_dot*dt; 
        State.w     = State.w + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt; 
        State.q     = State.q + Derv.q_dot*dt;
        
        % Ground Collision Detection
        if -State.z_E <= 0, Touchdown_Flag = true; break; end
    end
    
    % 5. Cost Calculation (Weighted Penalty Function)
    if ~Touchdown_Flag
        J = 1e10 + (-State.z_E * 2000); % Penalty: Failed to Land (Floating)
    else
        % Weights prioritized: Pitch Geometry > Sink Rate > Smoothness
        
        % A. Sink Rate Error
        Sink_Err = (-Derv.z_dot_E - Config.Targets.SinkRate)^2;
        Sink_Penalty = Sink_Err * 20000;
        if Derv.z_dot_E < Config.Limits.SinkRate_Hard, Sink_Penalty = 1e12; end % Crash Penalty
        
        % B. Pitch Geometry Error (CRITICAL - Tail Strike Prevention)
        Pitch_Err = (rad2deg(State.theta) - Config.Targets.Pitch_deg)^2;
        Pitch_Penalty = Pitch_Err * 50000; 
        
        % C. Operational Penalties
        Time_Penalty = k * 150; % Discourage excessive floating
        Smooth_Penalty = Total_Control_Effort * 1000;
        
        J = Sink_Penalty + Pitch_Penalty + Time_Penalty + Smooth_Penalty;
    end
end

%% ========================================================================
%  LOCAL FUNCTION: HIGH-FIDELITY VISUALIZATION
%  ========================================================================
function VisualizePerformance(Gains, CP, Config)
    fprintf('>> Generating Engineering Plots (Standard Palette)...\n');
    
    % --- Reconstruct Control Structure ---
    C.Pitch.Kp = Gains(1); C.Pitch.Ki = Gains(2); C.Pitch.Kd = Gains(3);
    C.Pitch.Max = Config.Limits.Elevator_Max; 
    C.Pitch.Min = -Config.Limits.Elevator_Max;
    C.Throt.Kp = Gains(4); C.Throt.Ki = Gains(7); 
    C.Throt.Kd = Gains(5); C.Throt.Base = Gains(6);
    C.Throt.Max = 1.0; C.Throt.Min = 0.0;
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.01;
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    ThrotState = struct('Integrator', 0, 'PrevError', 0);
    
    % --- Data Logging Arrays ---
    Hist_Alt = []; Hist_Sink = []; Hist_Theta = []; 
    Hist_Elev = []; Hist_Throt = []; Hist_Alpha = [];
    
    % --- Verification Simulation ---
    for k = 1:2000
        Current_SinkRate = -State.w * cos(State.theta) + State.u * sin(State.theta);
        Target_Theta_Rad = deg2rad(Config.Targets.Pitch_deg);
        [Elev_Cmd, PitchState] = RunPID(Target_Theta_Rad, State.theta, dt, C.Pitch, PitchState, State.q);
        [Throt_Cmd, ThrotState] = RunPID(Config.Targets.SinkRate, Current_SinkRate, dt, C.Throt, ThrotState);
        
        if -State.z_E < 0.3, Throt_Cmd = 0.0; end
        
        Controls.ElevatorDeflection = Elev_Cmd; Controls.ThrottleSetting = Throt_Cmd;
        Controls.SpeedBrake = 0.5; Controls.Gear = 1.0;
        Log = FlightDynamics(State, AC, Env, Controls); Derv = Log.Derivatives;
        
        State.x_E = State.x_E + Derv.x_dot_E*dt; State.z_E = State.z_E + Derv.z_dot_E*dt;
        State.u = State.u + Derv.u_dot*dt; State.w = State.w + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt; State.q = State.q + Derv.q_dot*dt;
        
        Hist_Alt(k)   = -State.z_E;
        Hist_Sink(k)  = Current_SinkRate; 
        Hist_Theta(k) = rad2deg(State.theta);
        Hist_Throt(k) = Throt_Cmd * 100;
        Hist_Elev(k)  = rad2deg(Elev_Cmd);
        Hist_Alpha(k) = rad2deg(atan2(State.w, State.u));
        
        if -State.z_E <= 0, break; end
    end
    
    Final_Sink = Hist_Sink(end);
    Final_Pitch = Hist_Theta(end);
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
    
    % --- PLOTTING: WINDOW 1 (TRAJECTORY) ---
    figure('Name','Flight Dynamics & Safety Limits','Color','w');
    
    % Plot 1: Altitude
    subplot(3,1,1); 
    plot(Hist_Alt, 'b', 'LineWidth', 2); grid on;
    ylabel('Altitude (m)', 'FontWeight', 'bold');
    title('Flare Trajectory Profile', 'FontSize', 11);
    
    % Plot 2: Sink Rate (With Limits)
    subplot(3,1,2); 
    plot(Hist_Sink, 'r', 'LineWidth', 2); grid on; hold on;
    yline(Config.Targets.SinkRate, '--g', 'LineWidth', 1.5, ...
        'Label', 'Target (-0.6 m/s)', 'LabelHorizontalAlignment','left');
    yline(Config.Limits.SinkRate_Hard, '-k', 'LineWidth', 2, ...
        'Label', 'Structural Limit (-2.5 m/s)', 'LabelHorizontalAlignment','left');
    ylabel('Sink Rate (m/s)', 'FontWeight', 'bold');
    title(sprintf('Vertical Speed Cushion (Final: %.2f m/s)', Final_Sink), 'FontSize', 11);
    
    % Plot 3: Pitch (With Limits)
    subplot(3,1,3); 
    plot(Hist_Theta, 'b', 'LineWidth', 2); grid on; hold on;
    yline(Config.Targets.Pitch_deg, '--g', 'LineWidth', 1.5, ...
        'Label', sprintf('Target (%.1f deg)', Config.Targets.Pitch_deg), 'LabelHorizontalAlignment','left');
    yline(Config.Limits.Pitch_Max, '-r', 'LineWidth', 2, ...
        'Label', 'Tail Strike Limit (13.0 deg)', 'LabelHorizontalAlignment','left');
    ylabel('Pitch (deg)', 'FontWeight', 'bold');
    xlabel('Simulation Steps (dt = 0.01s)');
    title(sprintf('Touchdown Geometry (Final: %.2f deg)', Final_Pitch), 'FontSize', 11);

    % --- PLOTTING: WINDOW 2 (CONTROLS) ---
    figure('Name','Control System & Aerodynamics','Color','w');
    
    % Plot 1: Elevator
    subplot(3,1,1); 
    plot(Hist_Elev, 'm', 'LineWidth', 2); grid on; 
    ylabel('Elevator (deg)', 'FontWeight', 'bold');
    title('Pitch Control Effort', 'FontSize', 11);
    
    % Plot 2: Throttle
    subplot(3,1,2); 
    plot(Hist_Throt, 'Color', [0.9 0.6 0], 'LineWidth', 2); grid on; % Kept Orange for Throttle
    ylabel('Throttle (%)', 'FontWeight', 'bold');
    title('Power Management', 'FontSize', 11);
    
    % Plot 3: Alpha
    subplot(3,1,3);
    plot(Hist_Alpha, 'k', 'LineWidth', 2); grid on; 
    ylabel('Alpha (deg)', 'FontWeight', 'bold');
    xlabel('Simulation Steps (dt = 0.01s)');
    title('Angle of Attack Stability', 'FontSize', 11);
end


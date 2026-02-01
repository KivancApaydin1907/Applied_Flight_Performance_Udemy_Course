%% ========================================================================
%  PROJECT:     T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:      PRECISION LANDING TUNER (FLARE & TOUCHDOWN) - FINAL V5
%  ========================================================================
%  AUTHOR:      Kivanc Apaydin
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script optimizes the automatic landing logic for the T-38 Talon.
%     It utilizes a Multi-Variable (MIMO) control strategy to arrest the
%     sink rate while maintaining a safe landing attitude.
%
%  OBJECTIVES:
%     1. Achieve a "Greaser" touchdown (Target: -0.6 m/s).
%     2. Minimize control surface oscillations (Smoothness Penalty).
%     3. Prevent runway "Floating" (Time-based Penalty).
% ========================================================================
clearvars; clc; close all;

%% 1. INITIALIZATION & CONFIGURATION
fprintf('----------------------------------------------------------\n');
fprintf('>> T-38 FLARE & LANDING TUNER INITIATED (FINAL PRODUCTION).\n');
fprintf('----------------------------------------------------------\n');

if ~isfile('Flare_Checkpoint.mat')
    error('>> ERROR: Required "Flare_Checkpoint.mat" not found in path.');
end
load('Flare_Checkpoint.mat', 'Checkpoint');

% --- PERFORMANCE TARGETS ---
TunerConfig.Target_Theta    = deg2rad(6.5); % High AOA for aerodynamic braking
TunerConfig.Target_SinkRate = -0.6;         % Ideal sink rate (Butter landing)
TunerConfig.Hard_Limit      = -2.5;         % Maximum structural limit (m/s)
TunerConfig.Elev_Limit      = deg2rad(20);  % Max physical elevator deflection

% --- OPTIMIZATION VECTOR (7 GAINS) ---
% Index 1-3: Pitch PID [Kp, Ki, Kd]
% Index 4-6: Throttle PD+Base [Kp, Kd, Base]
% Index 7  : Throttle Integral [Ki]
InitialGuess = [-10.0, 0.01, 12.0, ...   
                 0.15,  1.5,  0.45, ...  
                 0.005];                

Opts = optimset('Display', 'iter', ...
                'MaxIter', 1000, ...
                'MaxFunEvals', 2000, ...
                'TolX', 1e-6, ...
                'TolFun', 1e-6);

%% 2. EXECUTE OPTIMIZATION
fprintf('>> Optimization process started. Objective: Stability & Smoothness...\n');
CostFunc = @(Gains) EvaluateFlarePerformance(Gains, Checkpoint, TunerConfig);
[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGuess, Opts);

%% 3. REPORT RESULTS
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION COMPLETE.\n');
fprintf('==========================================================\n');
fprintf('Final Optimized Cost: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('1. PITCH CONTROLLER (Nose Attitude Hold):\n');
fprintf('   Kp: %8.4f | Ki: %8.4f | Kd: %8.4f\n', OptimalGains(1), OptimalGains(2), OptimalGains(3));
fprintf('----------------------------------------------------------\n');
fprintf('2. THROTTLE CONTROLLER (Vertical Speed Control):\n');
fprintf('   Kp: %8.4f | Ki: %8.4f | Kd: %8.4f\n', OptimalGains(4), OptimalGains(7), OptimalGains(5));
fprintf('   Base Setting (FFW): %.4f\n', OptimalGains(6));
fprintf('==========================================================\n');

VisualizePerformance(OptimalGains, Checkpoint, TunerConfig);

%% LOCAL FUNCTION: PERFORMANCE EVALUATION (COST FUNCTION)
function J = EvaluateFlarePerformance(Gains, CP, Config)
    % --- Unpack Controller Gains ---
    C.Pitch.Kp = Gains(1); C.Pitch.Ki = Gains(2); C.Pitch.Kd = Gains(3);
    C.Pitch.Max = Config.Elev_Limit; C.Pitch.Min = -Config.Elev_Limit;
    
    C.Throt.Kp   = Gains(4); 
    C.Throt.Ki   = Gains(7); 
    C.Throt.Kd   = Gains(5);
    C.Throt.Base = Gains(6);
    C.Throt.Max  = 1.0; C.Throt.Min  = 0.0;
    
    % --- Feasibility Constraint Check ---
    if C.Pitch.Kp > -0.5 || C.Throt.Base < 0.2 || C.Throt.Base > 0.8 || C.Throt.Ki < 0
         J = 1e15; return; 
    end
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.01;
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    ThrotState = struct('Integrator', 0, 'PrevError', 0);
    
    MaxSteps = 1500; 
    Touchdown_Flag = false;
    Total_Control_Effort = 0;
    Prev_Throttle = 0; Prev_Elevator = 0;
    
    % --- Simulation Loop ---
    for k = 1:MaxSteps
        % Vertical Velocity Calculation
        Current_SinkRate = -State.w * cos(State.theta) + State.u * sin(State.theta); 
        
        % PID Control Laws
        [Delta_Cmd, PitchState] = RunPID(Config.Target_Theta, State.theta, dt, C.Pitch, PitchState, State.q);
        [Throttle_Cmd, ThrotState] = RunPID(Config.Target_SinkRate, Current_SinkRate, dt, C.Throt, ThrotState);
        
        % Auto-Idle Logic (Altitude < 0.3m)
        if -State.z_E < 0.3, Throttle_Cmd = 0.0; end
        
        % Compute Control Effort (Penalty for hunting/chatter)
        if k > 1
            Total_Control_Effort = Total_Control_Effort + abs(Throttle_Cmd - Prev_Throttle)*50 + abs(Delta_Cmd - Prev_Elevator)*10;
        end
        Prev_Throttle = Throttle_Cmd; Prev_Elevator = Delta_Cmd;
        
        % Physics Engine Call
        Controls.ElevatorDeflection = Delta_Cmd;
        Controls.ThrottleSetting    = Throttle_Cmd;
        Controls.SpeedBrake         = 0.5; 
        Controls.Gear               = 1.0;
        
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        % State Integration
        State.x_E   = State.x_E + Derv.x_dot_E*dt; 
        State.z_E   = State.z_E + Derv.z_dot_E*dt;
        State.u     = State.u + Derv.u_dot*dt; 
        State.w     = State.w + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt; 
        State.q     = State.q + Derv.q_dot*dt;
        
        if -State.z_E <= 0, Touchdown_Flag = true; break; end
    end
    
    % --- Cost Calculation Logic ---
    if ~Touchdown_Flag
        J = 1e10 + (-State.z_E * 2000); % Large penalty for floating or crash
    else
        % Objective 1: Sink Rate Accuracy
        Sink_Penalty = ((Derv.z_dot_E - Config.Target_SinkRate)^2) * 25000;
        if Derv.z_dot_E < Config.Hard_Limit, Sink_Penalty = 1e12; end
        
        % Objective 2: Nose-High Geometry
        Pitch_Penalty = (rad2deg(State.theta) - rad2deg(Config.Target_Theta))^2 * 4000;
        
        % Objective 3: Avoid Floating (Minimize Runway Length used)
        Time_Penalty = k * 150; 
        
        % Objective 4: Smoothness (Control oscillation penalty)
        Smooth_Penalty = Total_Control_Effort * 1000;
        
        J = Sink_Penalty + Pitch_Penalty + Time_Penalty + Smooth_Penalty;
    end
end

%% LOCAL FUNCTION: DUAL-WINDOW VISUALIZATION
function VisualizePerformance(Gains, CP, Config)
    fprintf('>> Generating High-Fidelity Results Plot...\n');
    
    % Reconstruct Controllers for Verification
    C.Pitch.Kp = Gains(1); C.Pitch.Ki = Gains(2); C.Pitch.Kd = Gains(3);
    C.Pitch.Max = Config.Elev_Limit; C.Pitch.Min = -Config.Elev_Limit;
    C.Throt.Kp = Gains(4); C.Throt.Ki = Gains(7); C.Throt.Kd = Gains(5); C.Throt.Base = Gains(6);
    C.Throt.Max = 1.0; C.Throt.Min = 0.0;
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.02;
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    ThrotState = struct('Integrator', 0, 'PrevError', 0);
    
    Hist_Alt = []; Hist_Sink = []; Hist_Theta = []; Hist_Elev = []; Hist_Throt = [];
    
    for k = 1:1500
        Current_SinkRate = -State.w * cos(State.theta) + State.u * sin(State.theta);
        [Elev_Cmd, PitchState] = RunPID(Config.Target_Theta, State.theta, dt, C.Pitch, PitchState, State.q);
        [Throt_Cmd, ThrotState] = RunPID(Config.Target_SinkRate, Current_SinkRate, dt, C.Throt, ThrotState);
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
        
        if -State.z_E <= 0, break; end
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
    fprintf('\nTOUCHDOWN PERFORMANCE REPORT:\n');
    fprintf('   Impact Velocity: %.2f m/s (%.1f fpm)\n', Final_Sink, Final_Sink*196.85);
    fprintf('   Nose Attitude  : %.2f deg\n', Hist_Theta(end));
    
    % WINDOW 1: FLIGHT TRAJECTORY & PERFORMANCE
    figure('Name','Flare & Touchdown Analysis','Color','w');
    subplot(3,1,1); 
    plot(Hist_Alt, 'Color', [0 0.6 0], 'LineWidth', 2); grid on; 
    ylabel('Altitude (m)'); title('Flare Trajectory Profile');
    
    subplot(3,1,2); 
    plot(Hist_Sink, 'r', 'LineWidth', 2); grid on; 
    ylabel('Sink Rate (m/s)'); hold on;
    yline(Config.Target_SinkRate, '--k', 'Target');
    title(sprintf('Vertical Speed Cushion (Final: %.2f m/s)', Final_Sink));
    
    subplot(3,1,3); 
    plot(Hist_Theta, 'b', 'LineWidth', 2); grid on; 
    ylabel('Pitch (deg)'); hold on;
    yline(rad2deg(Config.Target_Theta), '--k', 'Target');
    title(sprintf('Touchdown Geometry (Final: %.2f deg)', Hist_Theta(end)));

    % WINDOW 2: CONTROL SURFACE DYNAMICS
    figure('Name','Control Input Stability','Color','w');
    subplot(2,1,1); 
    plot(Hist_Elev, 'm', 'LineWidth', 2); grid on; 
    ylabel('Elevator (deg)'); title('Pitch Stability & Damping');
    
    subplot(2,1,2); 
    plot(Hist_Throt, 'color', [0.9 0.6 0.1], 'LineWidth', 2); grid on; 
    ylabel('Throttle (%)'); title('Throttle Power Management');
    xlabel('Simulation Steps (0.02s interval)');
end

%% ========================================================================
%  PROJECT: T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:  CASCADE CONTROLLER TUNER - CRUISE PHASE
%  ========================================================================
%  AUTHOR:      Kıvanç Apaydın
%  DATE:        01/2026
%  PLATFORM:    MATLAB R2025b
%
%  DESCRIPTION:
%     This script tunes the Altitude Hold autopilot using a "Cascade Control"
%     architecture.
%
%     Architecture:
%     1. Outer Loop (Altitude Controller):
%        - Input: Altitude Error
%        - Output: Pitch Attitude Command (Theta_Ref)
%        - Dynamics: Slow / Navigation Loop
%
%     2. Inner Loop (Pitch Controller):
%        - Input: Pitch Error (Theta_Ref - Theta_Actual)
%        - Output: Elevator Deflection
%        - Dynamics: Fast / Stabilization Loop (Gains frozen from Climb Phase)
%
%     Optimization Targets:
%     - Altitude PID Gains (Kp, Ki, Kd)
%     - Cruise Throttle Trim (Feed-forward Base Value)
%
%  DEPENDENCIES:
%     - FlightDynamics.m
%     - RunPID.m
%     - atmosisa.m
%     - Cruise_Checkpoint.mat (Generated at end of Climb Phase)
% ========================================================================
clearvars; clc; close all;

%% ========================================================================
%  1. INITIALIZATION & CONFIGURATION
%  ========================================================================
fprintf('----------------------------------------------------------\n');
fprintf('>> T-38 CRUISE CONTROLLER TUNER (CASCADE) INITIATED.\n');
fprintf('----------------------------------------------------------\n');

if ~isfile('Cruise_Checkpoint.mat')
    error('>> ERROR: Checkpoint file "Cruise_Checkpoint.mat" not found.');
end
load('Cruise_Checkpoint.mat', 'Checkpoint');
fprintf('>> Checkpoint Loaded. Simulation Time: %.2f s\n', Checkpoint.Time);

% --- TUNER TARGETS ---
TunerConfig.Target_Alt  = 11000;        % [m] Cruise Altitude (FL360 approx)
TunerConfig.Target_Mach = 0.8;          % [Mach] Cruise Speed

% --- INHERITED GAINS (FROZEN) ---
% These gains were solved in the previous "Climb Phase".
% In a cascade system, the inner loop must be tuned first and kept constant.
Inherited.Pitch.Kp = -18.0340;  
Inherited.Pitch.Ki =  0.0807;
Inherited.Pitch.Kd =  9.2617;
Inherited.Speed.Kp =  533.4933;
Inherited.Speed.Ki = -0.0163;
Inherited.Speed.Kd = -43.4132;

% --- OPTIMIZATION VECTOR ---
% G(1-3): Altitude PID [Kp, Ki, Kd]
% G(4):   Cruise Trim Throttle (Base Setting)
% Initial Guess logic: Altitude Kp must be small to avoid aggressive pitch commands.
InitialGuess = [0.0005, 0.00001, 0.001, 0.4]; 

% Solver Options
Opts = optimset('Display', 'iter', ...
                'MaxIter', 400, ...
                'TolX', 1e-5, ...
                'TolFun', 1e-5);

%% ========================================================================
%  2. EXECUTE CASCADE OPTIMIZATION
%  ========================================================================
fprintf('>> Starting Optimization Loop (Objective: Minimize Altitude Error)...\n');

CostFunc = @(Gains) EvaluateCruisePerformance(Gains, Checkpoint, TunerConfig, Inherited);

[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGuess, Opts);

%% ========================================================================
%  3. REPORT RESULTS
%  ========================================================================
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION COMPLETE.\n');
fprintf('==========================================================\n');
fprintf('Final Cost: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('1. ALTITUDE CONTROLLER (Outer Loop):\n');
fprintf('   Kp: %10.8f [Theta_Ref per Meter]\n', OptimalGains(1));
fprintf('   Ki: %10.8f\n', OptimalGains(2));
fprintf('   Kd: %10.8f\n', OptimalGains(3));
fprintf('----------------------------------------------------------\n');
fprintf('2. TRIM SETTING:\n');
fprintf('   Cruise Throttle: %.2f%%\n', OptimalGains(4)*100);
fprintf('==========================================================\n');

VisualizePerformance(OptimalGains, Checkpoint, TunerConfig, Inherited);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateCruisePerformance(Gains, CP, Config, FixedGains)
    
    % --- 1. SETUP CONTROLLERS ---
    % [A] Altitude Controller (Tuning Variables)
    C.Alt.Kp  = Gains(1); 
    C.Alt.Ki  = Gains(2); 
    C.Alt.Kd  = Gains(3);
    % Limit Authority: The autopilot can only request +/- 5 deg pitch change
    % to correct altitude errors. This ensures passenger comfort.
    C.Alt.Max = deg2rad(5);  
    C.Alt.Min = -deg2rad(5); 

    % [B] Pitch Controller (Fixed / Inner Loop)
    C.Pitch.Kp  = FixedGains.Pitch.Kp;
    C.Pitch.Ki  = FixedGains.Pitch.Ki;
    C.Pitch.Kd  = FixedGains.Pitch.Kd;
    C.Pitch.Max = deg2rad(20); 
    C.Pitch.Min = -deg2rad(20);

    % [C] Speed Controller (Fixed)
    C.Speed.Kp  = FixedGains.Speed.Kp;
    C.Speed.Ki  = FixedGains.Speed.Ki;
    C.Speed.Kd  = FixedGains.Speed.Kd;
    C.Speed.Max = 1.0; 
    C.Speed.Min = 0.0;
    
    % [D] Trim (Tuning Variable)
    C.Speed.Base = Gains(4); 
    
    % --- 2. FEASIBILITY CHECKS ---
    % Alt Kp must be positive (Error > 0 means we are low -> Pitch Up)
    if C.Alt.Kp < 0 || C.Speed.Base < 0 || C.Speed.Base > 1.0
         J = 1e15; return; 
    end
    
    % --- 3. SIMULATION INITIALIZATION ---
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; 
    dt = 0.1;
    
    AltState   = struct('Integrator', 0, 'PrevError', 0);
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    SpeedState = struct('Integrator', 0, 'PrevError', 0);
    
    Total_Alt_Error = 0; 
    Saturation_Penalty = 0;
    
    MaxSteps = 1000; % 100 seconds (Cruise settling takes time)
    
    % --- 4. FLIGHT LOOP ---
    for k = 1:MaxSteps
        % [Sensors]
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        V_tot = sqrt(State.u^2 + State.w^2);
        Mach  = V_tot / Env.a;
        Current_Alt = -State.z_E;
        
        % --- CASCADE CONTROL LOGIC ---
        
        % 1. OUTER LOOP: Altitude -> Pitch Reference
        % We use vertical speed (-z_dot) as the derivative term for better damping.
        % Vertical Speed approx = -w*cos(theta) + u*sin(theta)
        ClimbRate = -State.w * cos(State.theta) + State.u * sin(State.theta); 
        
        [Theta_Ref_Delta, AltState] = RunPID(Config.Target_Alt, Current_Alt, dt, ...
                                             C.Alt, AltState, ClimbRate);
                                         
        % The output of the Altitude Loop is the COMMAND for the Pitch Loop.
        Theta_Command = Theta_Ref_Delta; 
        
        % 2. INNER LOOP: Pitch Reference -> Elevator
        % The target is Theta_Command, the Feedback is State.theta
        [Elev_Cmd, PitchState] = RunPID(Theta_Command, State.theta, dt, ...
                                        C.Pitch, PitchState, State.q);
        
        % 3. INDEPENDENT LOOP: Speed -> Throttle
        [Throt_Cmd, SpeedState] = RunPID(Config.Target_Mach, Mach, dt, ...
                                         C.Speed, SpeedState);
        
        % [Actuator Constraint]
        if Throt_Cmd >= 0.99 || Throt_Cmd <= 0.01
            Saturation_Penalty = Saturation_Penalty + 10; 
        end
        
        % [Physics]
        Controls.ElevatorDeflection = Elev_Cmd;
        Controls.ThrottleSetting    = Throt_Cmd;
        Controls.Gear               = 0;
        Log = FlightDynamics(State, AC, Env, Controls);
        
        % [Integration]
        Derv = Log.Derivatives;
        State.x_E   = State.x_E   + Derv.x_dot_E*dt;
        State.z_E   = State.z_E   + Derv.z_dot_E*dt;
        State.u     = State.u     + Derv.u_dot*dt;
        State.w     = State.w     + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt;
        State.q     = State.q     + Derv.q_dot*dt;
        
        % [Cost Function]
        % Weighted heavily on Altitude Error
        Total_Alt_Error = Total_Alt_Error + abs(Config.Target_Alt - Current_Alt);
        
        if State.z_E > 0; J = 1e15; return; end
    end
    
    J = Total_Alt_Error + Saturation_Penalty;
end

%% ========================================================================
%  LOCAL FUNCTION: VISUALIZATION
%  ========================================================================
function VisualizePerformance(Gains, CP, Config, FixedGains)
    fprintf('>> Generating Visualization...\n');
    
    % Reconstruct Controllers
    C.Alt.Kp = Gains(1); C.Alt.Ki = Gains(2); C.Alt.Kd = Gains(3);
    C.Alt.Max = deg2rad(5); C.Alt.Min = -deg2rad(5);
    
    C.Pitch = FixedGains.Pitch;
    C.Pitch.Max = deg2rad(20); C.Pitch.Min = -deg2rad(20);
    
    C.Speed = FixedGains.Speed;
    C.Speed.Max = 1.0; C.Speed.Min = 0.0;
    C.Speed.Base = Gains(4); 
    
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = 0.1;
    AltState   = struct('Integrator', 0, 'PrevError', 0);
    PitchState = struct('Integrator', 0, 'PrevError', 0);
    SpeedState = struct('Integrator', 0, 'PrevError', 0);
    
    % History Arrays
    Hist_T = []; Hist_Alt = []; Hist_Pitch = []; Hist_PitchCmd = []; Hist_Mach = [];
    
    for k = 1:1000
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        V_tot = sqrt(State.u^2 + State.w^2);
        Mach  = V_tot / Env.a;
        Current_Alt = -State.z_E;
        
        % Cascade Logic
        ClimbRate = -State.w * cos(State.theta) + State.u * sin(State.theta);
        [Theta_Ref, AltState] = RunPID(Config.Target_Alt, Current_Alt, dt, C.Alt, AltState, ClimbRate);
        [Elev_Cmd, PitchState] = RunPID(Theta_Ref, State.theta, dt, C.Pitch, PitchState, State.q);
        [Throt_Cmd, SpeedState] = RunPID(Config.Target_Mach, Mach, dt, C.Speed, SpeedState);
        
        % Physics
        Controls.ElevatorDeflection = Elev_Cmd;
        Controls.ThrottleSetting    = Throt_Cmd;
        Controls.Gear               = 0;
        Log = FlightDynamics(State, AC, Env, Controls);
        
        Derv = Log.Derivatives;
        State.x_E = State.x_E + Derv.x_dot_E*dt;
        State.z_E = State.z_E + Derv.z_dot_E*dt;
        State.u   = State.u   + Derv.u_dot*dt;
        State.w   = State.w   + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt;
        State.q   = State.q   + Derv.q_dot*dt;
        
        % Store
        Hist_T(k)        = k*dt;
        Hist_Alt(k)      = Current_Alt;
        Hist_Pitch(k)    = rad2deg(State.theta);
        Hist_PitchCmd(k) = rad2deg(Theta_Ref);
        Hist_Mach(k)     = Mach;
        
        if State.z_E > 0, break; end
    end
    
    % Plotting
    figure('Name','Cascade Control Performance','Color','w');
    
    % 1. Altitude
    subplot(3,1,1); 
    plot(Hist_T, Hist_Alt, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); grid on;
    yline(Config.Target_Alt, '--r', 'Target (11,000 m)', 'LineWidth', 1.5);
    ylabel('Altitude (m)'); title('Outer Loop: Altitude Capture');
    
    % 2. Cascade Dynamics (Command vs Actual) - CRITICAL PLOT
    subplot(3,1,2); 
    plot(Hist_T, Hist_PitchCmd, 'r--', 'LineWidth', 1.5); hold on;
    plot(Hist_T, Hist_Pitch, 'b', 'LineWidth', 2); 
    legend('Theta Command (from Alt Loop)', 'Theta Actual (Aircraft)', 'Location', 'Best'); 
    grid on; ylabel('Pitch (deg)'); title('Inner Loop: Pitch Tracking');
    
    % 3. Speed
    subplot(3,1,3); 
    plot(Hist_T, Hist_Mach, 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880]); grid on;
    yline(Config.Target_Mach, '--r', 'Target (0.8 M)');
    ylabel('Mach'); title('Speed Maintenance');
    xlabel('Time (s)');
end
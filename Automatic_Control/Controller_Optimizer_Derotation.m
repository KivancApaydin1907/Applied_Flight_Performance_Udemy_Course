%% ========================================================================
%  PROJECT:       T-38 TALON FLIGHT DYNAMICS & CONTROL
%  MODULE:        DEROTATION PHASE OPTIMIZER (NOSE GEAR LOWERING)
%  ========================================================================
%  AUTHOR:        Kivanc Apaydin
%  DATE:          02/2026
%  PLATFORM:      MATLAB R2025b
%
%  DESCRIPTION:
%     This script utilizes the Nelder-Mead simplex algorithm (fminsearch) 
%     to tune the PID control gains for the aircraft derotation phase 
%     (transition from 2-point main gear contact to 3-point contact).
%
%  PERFORMANCE OBJECTIVES:
%     1. Soft Touchdown: Regulate nose pitch rate (q) to approx -0.03 rad/s.
%     2. Structural Safety: Strictly prevent impact rates exceeding -0.05 rad/s.
%     3. Transient Damping: Prevent initial elevator saturation (shock).
% ========================================================================
clearvars; clc; close all;

%% 1. SYSTEM INITIALIZATION & ARTIFACT LOADING
fprintf('==========================================================\n');
fprintf('>> T-38 DEROTATION OPTIMIZER INITIATED (LOGIC MATCH MODE)\n');
fprintf('==========================================================\n');

% --- Load Simulation Checkpoint (Handover from Flare Phase) ---
if ~isfile('Derotation_Checkpoint.mat')
    error('>> FATAL ERROR: Checkpoint file "Derotation_Checkpoint.mat" not found.');
end
load('Derotation_Checkpoint.mat', 'Checkpoint');
fprintf('>> System State Loaded. Touchdown Timestamp: %.2f s\n', Checkpoint.Time);

% --- TUNER CONFIGURATION ---
TunerConfig.Target_Theta    = 0;       % [rad] Target Pitch (Hard Zero / Ground)
TunerConfig.Target_Q        = -0.03;   % [rad/s] Ideal Descent Rate (Soft Contact)
TunerConfig.Structural_Lim  = -0.05;   % [rad/s] Maximum Allowable Impact Rate

% --- OPTIMIZATION VECTOR (INITIAL GUESS) ---
% Strategy:
% Kp (-0.2): Low proportional gain to handle large initial error (7 deg) 
%            without causing immediate actuator saturation (Shock).
% Ki (0.0) : Integral action not required for this transient phase.
% Kd (-3.0): Negative derivative gain provides active damping against acceleration.
InitialGains = [-0.2, 0.0, -3.0]; 

% --- SOLVER SETTINGS ---
Opts = optimset('Display', 'iter', ...
                'MaxIter', 600, ...
                'TolX', 1e-5, ...
                'TolFun', 1e-5);

%% 2. EXECUTE OPTIMIZATION ENGINE
fprintf('>> Optimization Engine Started. Solving for Soft Step Response...\n');
CostFunc = @(Gains) EvaluateDerotationPerformance(Gains, Checkpoint, TunerConfig);
[OptimalGains, MinCost] = fminsearch(CostFunc, InitialGains, Opts);

%% 3. FINAL REPORT & RESULTS
fprintf('\n==========================================================\n');
fprintf('OPTIMIZATION SUCCESSFUL.\n');
fprintf('Final Cost Function Value: %.4f\n', MinCost);
fprintf('----------------------------------------------------------\n');
fprintf('OPTIMIZED DEROTATION GAINS:\n');
fprintf('   Kp (Proportional): %8.5f  [Position Control]\n', OptimalGains(1));
fprintf('   Ki (Integral)    : %8.5f  [Locked]\n', 0.0);
fprintf('   Kd (Derivative)  : %8.5f  [Active Damping]\n', OptimalGains(3));
fprintf('==========================================================\n');

VisualizeResult(OptimalGains, Checkpoint, TunerConfig);

%% ========================================================================
%  LOCAL FUNCTION: COST FUNCTION EVALUATION
%  ========================================================================
function J = EvaluateDerotationPerformance(Gains, CP, Config)
    % 1. Controller Configuration
    C.Kp = Gains(1); 
    C.Ki = 0; 
    C.Kd = Gains(3);
    
    % Actuator Physical Limits (T-38 Stabilator)
    C.Max = deg2rad(15); 
    C.Min = -deg2rad(15); 
    
    % 2. Simulation Initialization
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = CP.dt;
    PIDState = struct('Integrator', 0, 'PrevError', 0);
    
    % [CRITICAL] Pre-calculate Derivatives (Simulating the t-1 state)
    % This is required to start the loop with the "Integration First" step.
    Controls = CP.LastControls;
    InitialLog = FlightDynamics(State, AC, Env, Controls);
    Derv = InitialLog.Derivatives;
    
    Step = 0; 
    MaxSteps = 800; 
    Total_Cost = 0;
    
    % 3. PHYSICS SIMULATION LOOP (Matches Main Code Architecture)
    % Order: Integration -> Kinematics -> Control -> Physics
    while State.theta > 0 && Step < MaxSteps
        Step = Step + 1;
        
        % A. NUMERICAL INTEGRATION (Euler Method)
        % Updates state based on forces calculated in the PREVIOUS step.
        State.u     = State.u     + Derv.u_dot * dt;
        State.w     = State.w     + Derv.w_dot * dt;
        State.x_E   = State.x_E   + Derv.x_dot_E * dt;
        State.z_E   = State.z_E   + Derv.z_dot_E * dt;
        State.q     = State.q     + Derv.q_dot * dt;
        State.theta = State.theta + Derv.theta_dot * dt;
        
        % Kinematics Override (Pivot around Main Landing Gear)
        % Enforces the geometric constraint that the MLG is fixed to the ground.
        V_ground = sqrt(State.u^2 + State.w^2);
        State.u  = V_ground * cos(State.theta);
        State.w  = V_ground * sin(State.theta);
        
        % Ground Collision Constraint
        if State.z_E > 0, State.z_E = 0; State.w = 0; end
        
        % B. ENVIRONMENT UPDATE
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        
        % C. CONTROL LAW EXECUTION
        % Calculates command based on the NEW position (t)
        [Delta_Cmd, PIDState] = RunPID(Config.Target_Theta, State.theta, dt, ...
                                       C, PIDState, State.q);
        
        Controls.ElevatorDeflection = Delta_Cmd;
        Controls.ThrottleSetting    = 0.0; % Idle
        Controls.SpeedBrake         = 1.0; % Full Drag
        Controls.Gear               = 1.0; % Down
        
        % D. FLIGHT DYNAMICS
        % Calculates forces/moments for the NEXT step (t+1)
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        % --- COST CALCULATION ---
        
        % 1. Actuator Saturation Penalty (SHOCK ABSORBER)
        % If the elevator deflects >12 deg within the first 0.5s (50 steps),
        % apply a massive penalty. This forces a "Soft Start".
        if Step < 50 && abs(Delta_Cmd) > deg2rad(12)
            Total_Cost = Total_Cost + 1e7; 
        end
        
        % 2. Velocity Tracking (Target: -0.03 rad/s)
        Rate_Error = abs(State.q - Config.Target_Q);
        Total_Cost = Total_Cost + (Rate_Error^2 * 5000);
        
        % 3. Safety Limit Violation (Hard Constraint)
        if State.q < Config.Structural_Lim
             Violation = abs(State.q - Config.Structural_Lim);
             Total_Cost = Total_Cost + (Violation * 1e9);
        end
    end
    
    % --- FINAL EVALUATION ---
    if State.theta > 0
        Total_Cost = Total_Cost + 1e9; % Penalty: Failed to land (Timeout)
    else
        % Touchdown Analysis
        if State.q < Config.Structural_Lim
            Total_Cost = Total_Cost + 1e9; % Penalty: Structural Damage (Crash)
        else
            % Bonus: Precision Landing
            Final_Error = abs(State.q - Config.Target_Q);
            Total_Cost = Total_Cost - (1 / (Final_Error + 1e-4));
        end
    end
    J = Total_Cost;
end

%% ========================================================================
%  LOCAL FUNCTION: PERFORMANCE VISUALIZATION
%  ========================================================================
function VisualizeResult(Gains, CP, Config)
    fprintf('>> Generating Engineering Plots...\n');
    
    % Reconstruct Controller
    C.Kp = Gains(1); C.Ki = 0; C.Kd = Gains(3);
    C.Max = deg2rad(15); C.Min = -deg2rad(15); 
    
    % Simulation Init
    State = CP.StateVector; AC = CP.AC; Env = CP.Env; dt = CP.dt;
    PIDState = struct('Integrator', 0, 'PrevError', 0);
    
    Controls = CP.LastControls;
    InitialLog = FlightDynamics(State, AC, Env, Controls);
    Derv = InitialLog.Derivatives;
    
    % Data Logging
    T_Hist = []; Theta_Hist = []; Q_Hist = []; Elev_Hist = [];
    Step = 0;
    
    % Verification Loop
    while State.theta > 0 && Step < 800
        Step = Step + 1;
        
        % Integration
        State.u = State.u + Derv.u_dot*dt; State.w = State.w + Derv.w_dot*dt;
        State.theta = State.theta + Derv.theta_dot*dt; State.q = State.q + Derv.q_dot*dt;
        
        % Kinematics
        V_ground = sqrt(State.u^2 + State.w^2);
        State.u = V_ground * cos(State.theta); State.w = V_ground * sin(State.theta);
        if State.z_E > 0, State.z_E = 0; end
        
        % Env
        [~, Env.a, ~, Env.rho] = atmosisa(-State.z_E);
        
        % Control
        [Delta_Cmd, PIDState] = RunPID(Config.Target_Theta, State.theta, dt, C, PIDState, State.q);
        Controls.ElevatorDeflection = Delta_Cmd;
        
        % Physics
        Log = FlightDynamics(State, AC, Env, Controls);
        Derv = Log.Derivatives;
        
        % Store Data
        T_Hist(Step) = Step*dt; 
        Theta_Hist(Step) = rad2deg(State.theta);
        Q_Hist(Step) = State.q; 
        Elev_Hist(Step) = rad2deg(Delta_Cmd);
    end
    
    % --- PLOTTING ---
    figure('Color','w', 'Name', 'Derotation Analysis (Main Logic Match)');
    
    % Subplot 1: Pitch Attitude
    subplot(3,1,1); 
    plot(T_Hist, Theta_Hist, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); 
    grid on; ylabel('Theta (deg)'); title('Nose Drop Trajectory');
    
    % Subplot 2: Pitch Rate
    subplot(3,1,2); 
    plot(T_Hist, Q_Hist, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); 
    grid on; ylabel('q (rad/s)'); 
    yline(Config.Structural_Lim, '--r', 'LineWidth', 1.5); 
    yline(Config.Target_Q, '--g', 'LineWidth', 1.5); 
    title('Pitch Rate vs. Limits');
    
    % Subplot 3: Control Effort
    subplot(3,1,3); 
    plot(T_Hist, Elev_Hist, 'LineWidth', 2, 'Color', [0.9290 0.6940 0.1250]); 
    grid on; ylabel('Elevator (deg)'); xlabel('Time (s)');
    title('Control Surface Deflection');
end


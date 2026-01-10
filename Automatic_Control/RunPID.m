function [Output, State] = RunPID(Target, Current, dt, Config, State, varargin)
% RUNPID  Generic PID Controller with Anti-Windup and Feed-Forward.
% -------------------------------------------------------------------------
% Purpose:
%   Calculates the control output based on Proportional-Integral-Derivative
%   logic. Supports "Derivative on Measurement" (to reduce kick) and 
%   "Conditional Integration" (to prevent windup).
%
% Inputs:
%   Target   - Desired setpoint (e.g., Target Altitude, Target Mach)
%   Current  - Actual process variable (e.g., Current Altitude)
%   dt       - Simulation time step [s]
%   Config   - Structure containing gains (Kp, Ki, Kd) and limits (Min, Max)
%   State    - Structure containing memory (Integrator, PrevError)
%   varargin - (Optional) Rate input for Derivative term (e.g., Pitch Rate q)
%
% Outputs:
%   Output   - Saturated control command
%   State    - Updated memory state for the next time step
% -------------------------------------------------------------------------

    % --- 1. ERROR CALCULATION ---
    Error = Target - Current;

    % --- 2. PROPORTIONAL TERM (P) ---
    P_Term = Config.Kp * Error;

    % --- 3. DERIVATIVE TERM (D) ---
    % Strategy: Use specific rate if provided (Derivative on Measurement),
    % otherwise calculate rate from error (Derivative on Error).
    if ~isempty(varargin) && ~isempty(varargin{1})
        % Mode: Derivative on Measurement (e.g., using Gyro 'q' for pitch damping)
        Rate_Input = varargin{1};
        D_Term     = Config.Kd * Rate_Input; 
    else
        % Mode: Derivative on Error (Standard)
        Error_Rate = (Error - State.PrevError) / dt;
        D_Term     = Config.Kd * Error_Rate;
    end

    % --- 4. FEED-FORWARD / BASE VALUE ---
    % Adds a trim value (e.g., Throttle 0.7) if defined in Config.
    Base_Input = 0.0;
    if isfield(Config, 'Base')
        Base_Input = Config.Base;
    end

    % --- 5. INTEGRAL LOGIC & ANTI-WINDUP (CONDITIONAL INTEGRATION) ---
    % Calculate a temporary output to check for saturation *before* integrating.
    Current_I   = Config.Ki * State.Integrator;
    Temp_Output = Base_Input + P_Term + Current_I + D_Term;

    % Logic: Accumulate Integral ONLY if:
    %   a) The system is NOT saturated.
    %   b) The system IS saturated, but the Error helps to desaturate it.
    Accumulate_Integral = false;

    if (Temp_Output >= Config.Min && Temp_Output <= Config.Max)
        Accumulate_Integral = true; % Region: Linear (Safe)
    elseif (Temp_Output > Config.Max && Error < 0)
        Accumulate_Integral = true; % Region: Max Saturated, but Error is negative (Recovery)
    elseif (Temp_Output < Config.Min && Error > 0)
        Accumulate_Integral = true; % Region: Min Saturated, but Error is positive (Recovery)
    end

    if Accumulate_Integral
        State.Integrator = State.Integrator + (Error * dt);
    end

    % --- 6. FINAL OUTPUT CALCULATION ---
    Final_I    = Config.Ki * State.Integrator;
    Raw_Output = Base_Input + P_Term + Final_I + D_Term;

    % --- 7. SATURATION (ACTUATOR LIMITS) ---
    Output = max(min(Raw_Output, Config.Max), Config.Min);

    % --- 8. STATE UPDATE ---
    State.PrevError = Error;
    
end
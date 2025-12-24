%% T-38 TALON (J85-GE-5) PERFORMANCE ENVELOPE GENERATOR
% ==============================================================================
%  SCRIPT INFO
%  -----------
%  Course:      Applied Flight Performance Analysis (Udemy)
%  Author:      Kivanc Apaydin
%  Date:        December 2025
%  Engine:      General Electric J85-GE-5 (Turbojet with Afterburner)
%  Method:      Simplified "Mass Addition" Method with Semi-Empirical Corrections
%  Validation:  Calibrated to match GasTurb 13 design point data (~17.1 kN @ SLS)
% ==============================================================================

clear; clc; close all; tic;

%% 1. SIMULATION SETUP & FLIGHT CONDITIONS
%  Define the operating range for the aircraft envelope.
% ------------------------------------------------------------------------------
h_vec = 0:500:16000;    % Altitude [m] (Sea Level to 16km)
M_vec = 0:0.05:1.6;     % Mach Number [-] (Static to Supersonic)

%% 2. ENGINE DESIGN PARAMETERS (INPUTS)
%  Baseline cycle parameters derived from J85-GE-5 specifications.
% ------------------------------------------------------------------------------
m0_sl_des = 19.96;      % [kg/s] Design Mass Flow at Sea Level
pi_c      = 7.0;        % [-]    Compressor Pressure Ratio
Tt4_max   = 1166.5;     % [K]    Max Turbine Inlet Temperature
hpr       = 43124000;   % [J/kg] Fuel Lower Heating Value (LHV)

% --- Afterburner Settings ---
AB_Active = 1;          % [1=ON, 0=OFF] Toggle Reheat
Tt7_AB    = 1800;       % [K]    Afterburner Temperature

% --- Component Efficiencies (Polytropic & Isentropic) ---
eta_d     = 0.98;       % [-] Diffuser/Intake Pressure Recovery (Subsonic base)
e_c       = 0.85;       % [-] Compressor Polytropic Efficiency
eta_b     = 0.99;       % [-] Combustion Efficiency
pi_b      = 0.97;       % [-] Combustor Pressure Ratio
e_t       = 0.88;       % [-] Turbine Polytropic Efficiency
eta_mech  = 0.999;      % [-] Mechanical Shaft Efficiency
pi_n      = 0.98;       % [-] Nozzle Duct Pressure Ratio

%% 3. CALIBRATION & REAL-WORLD CORRECTIONS
%  These parameters are fine-tuned to match GasTurb/Real Engine test data.
% ------------------------------------------------------------------------------
% [1] Gas Properties (Tuned for high Tt7)
cp_c    = 1004;         % [J/kgK] Specific Heat (Cold Section)
cp_h    = 1190;         % [J/kgK] Specific Heat (Hot Section - Avg for 1800K)
gamma_c = 1.40;         % [-]     Gamma (Cold)
gamma_h = 1.30;         % [-]     Gamma (Hot - Exhaust products)
R       = 287;          % [J/kgK] Gas Constant

% [2] Pressure Losses
% J85 flame holders create significant blockage. We assume ~8% total pressure loss.
pi_ab   = 0.92;         % [-] Afterburner Pressure Ratio (Loss Factor)

% [3] Nozzle Performance
% Real nozzles have friction. The Velocity Coefficient (Cv) corrects ideal velocity.
Cv      = 0.98;         % [-] Nozzle Velocity Coefficient

% [4] Cooling / Bleed Air
% "Wcl/W6 = 0.1" from GasTurb. 10% of air bypasses the core for liner cooling.
bleed_frac = 0.10;      % [-] Bleed Fraction
% Cooling air mixes poorly with the main jet. We assume 50% momentum transfer.
mixing_eff = 0.50;      % [-] Mixing Efficiency Factor

%% 4. INITIALIZATION
Thrust_Map = zeros(length(h_vec), length(M_vec));
SFC_Map    = zeros(length(h_vec), length(M_vec));

fprintf('>> Simulation Started... (Calculated for %d points)\n', numel(Thrust_Map));

%% 5. MAIN CALCULATION LOOP
for i = 1:length(h_vec)
    for j = 1:length(M_vec)
        
        % --- A. CURRENT FLIGHT STATE ---
        h = h_vec(i);   % Current Altitude
        M0 = M_vec(j);  % Current Mach
        
        [T0, a0, P0, ~] = atmosisa(h); % ISA Atmosphere
        V0 = M0 * a0;                  % True Airspeed
        
        % --- B. INTAKE (Station 0 -> 2) ---
        % Isentropic stagnation properties
        Pt0 = P0 * (1 + ((gamma_c - 1)/2)*M0^2)^(gamma_c/(gamma_c - 1));
        Tt0 = T0 * (1 + ((gamma_c - 1)/2)*M0^2);
        
        % Ram Recovery (Mil-Spec Approximation)
        if M0 <= 1
            pi_r = 1.0; 
        else
            pi_r = 1.0 - 0.075*(M0 - 1)^1.35; 
        end
        
        Pt2 = Pt0 * eta_d * pi_r; 
        Tt2 = Tt0; % Adiabatic
        
        % --- C. MASS FLOW SCALING ---
        % Corrected mass flow scaling: m = m_des*(delta / sqrt(theta))
        theta = Tt2/288.15; 
        delta = Pt2/101325;
        m0 = m0_sl_des*(delta/sqrt(theta)); 
        
        % --- D. COMPRESSOR (Station 2 -> 3) ---
        Pt3 = Pt2*pi_c; 
        tau_c = pi_c^((gamma_c - 1)/(gamma_c*e_c)); 
        Tt3 = Tt2 * tau_c;     
        W_dot_comp = m0*cp_c*(Tt3 - Tt2); % Work Required [Watts]
        
        % --- E. COMBUSTOR (Station 3 -> 4) ---
        Pt4 = Pt3*pi_b; 
        Tt4 = Tt4_max;    % Assume limit is reached
        
        % Split Flow: Separate Core Air vs. Cooling Air
        m_bleed = m0*bleed_frac; 
        m_core  = m0 - m_bleed;  
        
        % Energy Balance for Fuel Flow (Core only)
        m_dot_f = (m_core*cp_h*Tt4 - m_core*cp_c*Tt3) / ...
                  (hpr*eta_b - cp_h * Tt4);
                  
        if m_dot_f < 0, m_dot_f = 0; end
        
        % --- F. TURBINE (Station 4 -> 5) ---
        m_turb = m_core + m_dot_f; % Mass flow expanding through turbine
        
        % Work Balance: Turbine drives Compressor
        W_dot_turb = W_dot_comp/eta_mech;
        
        Tt5 = Tt4 - (W_dot_turb/(m_turb*cp_h));
        tau_t = Tt5 / Tt4;
        pi_t = tau_t^(gamma_h/(gamma_h - 1)*e_t);
        Pt5 = Pt4*pi_t;
        
        % --- G. AFTERBURNER (Station 5 -> 7) ---
        % Apply Pressure Loss (Flame holder drag)
        Pt7 = Pt5*pi_ab; 
        
        if AB_Active == 1
            Tt7 = Tt7_AB;
            % Fuel required to reheat from Tt5 to Tt7
            m_dot_f_ab = (m_turb * cp_h * (Tt7 - Tt5)) / ...
                         (hpr * 0.90 - cp_h * Tt7); % 0.90 is AB efficiency
            if m_dot_f_ab < 0, m_dot_f_ab = 0; end
        else
            Tt7 = Tt5; 
            m_dot_f_ab = 0;
        end
        
        % --- H. NOZZLE (Station 7 -> 9) ---
        Pt9 = Pt7*pi_n;      
        
        % Check for Choking
        P_crit_ratio = (1/((gamma_h + 1)/2))^(gamma_h/(gamma_h - 1));
        
        if (P0/Pt9) < P_crit_ratio
            % Choked Flow
            P9 = Pt9*P_crit_ratio;
            T9 = Tt7 / ((gamma_h + 1)/2); 
            M9 = 1;
        else
            % Unchoked Flow (Ideally Expanded)
            P9 = P0;
            M9 = sqrt(2/(gamma_h - 1) * ((Pt9/P0)^((gamma_h - 1)/gamma_h) - 1));
            T9 = Tt7/(1 + (gamma_h - 1)/2 * M9^2);
        end
        
        % Calculate Ideal Velocity
        V9_ideal = M9*sqrt(gamma_h*R*T9);
        
        % Apply Velocity Coefficient (Friction correction)
        V9 = V9_ideal*Cv; 
        
        % --- I. THRUST CALCULATION ---
        % 1. Recombine Mass Flows
        m_core_exit = m_turb + m_dot_f_ab;
        
        % 2. Calculate "Effective" Momentum Mass
        % (Cooling air contributes less due to mixing losses)
        m_effective_exit = m_core_exit + (m_bleed*mixing_eff);
        
        % 3. Calculate Physical Mass (for Pressure Term Area)
        m_physical_exit = m_core_exit + m_bleed;
        
        % 4. Nozzle Exit Area Calculation
        rho9 = P9/(R*T9);
        if rho9 > 0
             A9_calc = m_physical_exit/(rho9*V9);
        else
             A9_calc = 0;
        end
        
        % 5. Gross Thrust (Momentum + Pressure)
        Fg = m_effective_exit * V9 + (P9 - P0)*A9_calc;
        
        % 6. Net Thrust
        Thrust = Fg - (m0*V0);
        
        % --- J. SFC CALCULATION ---
        m_fuel_tot = m_dot_f + m_dot_f_ab;
        
        if Thrust > 0
            SFC = (m_fuel_tot*3600) / Thrust; % [kg/(N*h)]
        else
            SFC = NaN;
        end
        
        % Store Results
        Thrust_Map(i,j) = max(Thrust, 0);
        SFC_Map(i,j)    = SFC;
    end
end
toc;

%% 6. VISUALIZATION & VERIFICATION
SLS_Thrust = Thrust_Map(1,1)/1000;
fprintf('\n------------------------------------------------\n');
fprintf(' RESULTS VERIFICATION\n');
fprintf('------------------------------------------------\n');
fprintf(' Target Thrust (GasTurb): ~17.10 kN\n');
fprintf(' Calculated Thrust:       %.2f kN\n', SLS_Thrust);
fprintf(' Deviation:               %.2f %%\n', (SLS_Thrust - 17.1)/17.1 * 100);
fprintf('------------------------------------------------\n');

% --- 3D Envelope Plot ---
figure('Name', 'T-38 Performance Map', 'Color', 'w');
surf(M_vec, h_vec/1000, Thrust_Map/1000);

% Aesthetics
shading interp; 
colormap(jet); 
c = colorbar;
c.Label.String = 'Net Thrust [kN]';
c.Label.FontWeight = 'bold';

% Labels
xlabel('Mach Number [-]', 'FontWeight', 'bold');
ylabel('Altitude [km]', 'FontWeight', 'bold');
zlabel('Thrust [kN]', 'FontWeight', 'bold');
title(['T-38 Talon (J85-GE-5) Performance Envelope' newline ...
       'Model: Mass Addition with Calibrated Losses'], 'FontSize', 12);

% View
view(-35, 25); 
grid minor;
axis tight;
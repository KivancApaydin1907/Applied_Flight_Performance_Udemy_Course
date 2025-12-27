# ⚙️ Script: GasTurb to MATLAB Importer

This script (`GasTurb_to_Matlab.m`) is a utility designed to automate the data transfer process from **GasTurb 15** cycle analysis software to **MATLAB/Simulink** environments.

It solves the problem of converting non-linear, scattered engine cycle data (exported as Excel sheets) into structured 3D Lookup Tables required for Flight Dynamics Simulations.

## 📈 Visual Output (Verification)
When the script is executed, it generates the following 3D surfaces to verify the smoothness and physical consistency of the interpolated data:

![Thrust_Data](https://github.com/user-attachments/assets/65763dc1-63c0-48b0-bc9f-01fa4151c3fe)

*(Figure: Processed J85-GE-21 Thrust Envelopes for Military and Afterburner power settings)*

---

## 🧠 Code Logic & Algorithm

The script operates in a **Batch Processing** mode, handling both "Dry" and "Wet" datasets in a single run. Here is the step-by-step breakdown of the code:

### 1. Configuration & Setup
The script starts by defining the file paths for the input data.
```matlab
files.Dry = 'J85_Reheat_Off.xlsx';  % Military Power
files.Wet = 'J85_Reheat.xlsx';      % Maximum Afterburner
```
### 2. Robust Data Parsing
GasTurb exports data in "blocks" within a single Excel sheet (e.g., blocks for Altitude = 0, Altitude = 5000, etc.). The script does not rely on hardcoded row numbers. Instead, it searches for headers dynamically:

* Scans column headers to find keywords: Altitude, Mach Number, Net Thrust, SFC.

* Identifies the start and end of each data block automatically.

* This ensures the code doesn't break if you export different variables from GasTurb.

### 3. Scattered Interpolation
Raw data from engine cycle analysis is often "scattered" (irregular intervals). To make it usable for Simulink Lookup Tables, we map it to a regular grid.

Function Used: scatteredInterpolant

Method: Linear interpolation with 'Nearest' neighbor extrapolation (to prevent NaN errors at envelope boundaries).

```matlab
% Creating a meshgrid from unique Mach and Altitude vectors
[M_Grid, h_Grid] = meshgrid(M_vec_unique, h_vec_unique);

% Converting scattered points to a surface
F_Thrust = scatteredInterpolant(All_Mach', All_Alt', All_Thrust', 'linear', 'nearest');
Thrust_Map = F_Thrust(M_Grid, h_Grid);
```

### 4. Data Structuring
The final data is stored in a clean MATLAB Structure (struct) to keep the workspace organized.

```matlab
EngineData.Dry.Thrust  % 3D Matrix [Altitude x Mach]
EngineData.Wet.Thrust  % 3D Matrix [Altitude x Mach]
```

## 🛠️ How to Run
Place the script and the .xlsx files in the same directory.

Run the script.

Load the resulting J85_Engine_Performance.mat file into your Simulink model or Workspace.

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>


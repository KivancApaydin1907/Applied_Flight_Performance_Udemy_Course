# 📂 Aerodynamic Data Handling

This module is responsible for bridging the gap between raw CFD results (from OpenVSP) and the flight simulation core. It processes, restructures, and visualizes aerodynamic coefficients.

## 🚀 Overview

The script `Import_OpenVSP_Data.m` performs three main tasks:
1.  **Import:** Reads raw text data exported from OpenVSP parametric analysis.
2.  **Process:** Reshapes the linear data arrays into structured 2D Lookup Tables (Matrices) based on Mach and Angle of Attack ($\alpha$).
3.  **Store:** Saves the processed data into a hierarchical MATLAB Structure (`struct`) for efficient access during simulation.

## 📊 Workflow

### 1. Input Data (`.txt`)
The script reads `T38_OpenVSP_CFD_Results.txt`.
* **Source:** OpenVSP Aerodynamic Analysis.
* **Format:** Linear list of simulation cases.
* **Parameters:**
    * **Mach Range:** 0.2 to 0.8 (Step: 0.2)
    * **Alpha Range:** 0° to 18° (Step: 1°)

### 2. Reshaping Logic (Critical Step)
OpenVSP exports data in a single column (1D array). To use this in simulation interpolation (`interp2`), we convert it into a 2D Matrix.

* **Rows:** Angle of Attack ($\alpha$)
* **Columns:** Mach Number

```matlab
% Logic used in the script:
Lookup_CL = reshape(raw_CL, n_alpha, n_mach);

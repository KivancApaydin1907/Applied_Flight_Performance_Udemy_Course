# J85-GE-5 Turbojet Performance Analysis (T-38 Talon)

![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-orange) ![Status](https://img.shields.io/badge/Status-Validated-brightgreen) ![Course](https://img.shields.io/badge/Udemy-Flight%20Performance-blue)

## ✈️ Project Overview
This repository contains a physics-based performance simulation model for the **General Electric J85-GE-5** turbojet engine, which powers the Northrop T-38 Talon.

Developed as part of the **"Applied Flight Performance Analysis"** Udemy course, this project generates the full flight envelope (Thrust & SFC maps) by solving the thermodynamic cycle of the engine using **MATLAB**. The model is calibrated against **GasTurb 15** design point data.

<p align="center">
  <img width="1114" height="490" alt="Ekran görüntüsü 2025-12-23 192334" src="https://github.com/user-attachments/assets/4bc172d5-43a1-4bcc-b14e-57e303eeaf91" />
  <br>
  <em>Figure 1: Turbojet Station Numbering (Standard Notation)</em>
</p>

---

## 🔧 Technical Methodology
This code implements a **0-D Gas Turbine Cycle Analysis** based on the Brayton Cycle. Unlike simple ideal cycle analysis, this model incorporates **semi-empirical corrections** to reflect real-world engine behavior.

### 1. Thermodynamic Model
The simulation iterates through engine stations (0 to 9) calculating stagnation properties ($P_t, T_t$). Key physics features include:
* **Variable Gas Properties:** Specific heat ($C_p$) and Gamma ($\gamma$) are adjusted between the Cold Section (Compressor) and Hot Section (Turbine/AB) to account for temperature effects.
* **Ram Recovery:** Inlet pressure recovery is modeled as a function of Mach number (Mil-Spec E-5007D approximation).
* **Choked Flow Logic:** The Convergent-Divergent (Con-Di) nozzle logic automatically switches between choked and unchoked regimes based on the nozzle pressure ratio (NPR).

### 2. "Mass Addition" & Loss Modeling
To simulate the complex effects of turbine cooling and afterburner flows without full CFD, the model uses a "Mass Addition" approach with calibrated loss factors:

* **Afterburner Drag ($\pi_{ab}$):** Accounts for the total pressure loss (~8%) caused by flame holders and spray bars in the jet pipe.
* **Nozzle Velocity Coefficient ($C_v$):** Corrects the ideal exit velocity to account for friction losses in the nozzle liner ($C_v \approx 0.98$).
* **Cooling Air Mixing:** Bleed air (10%) bypasses the core and re-enters at the nozzle. A **Mixing Efficiency (50%)** factor is applied to penalize the momentum contribution of this low-energy cooling flow.

---

### Key Simulation Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Design Mass Flow** | 19.96 kg/s | Corrected flow at SLS |
| **OPR ($\pi_c$)** | 7.0 | Overall Pressure Ratio |
| **TIT ($T_{t4}$)** | 1166.5 K | Turbine Inlet Temperature |
| **Reheat Temp ($T_{t7}$)** | 1800 K | Max Afterburner Temperature |
| **Nozzle Type** | Con-Di | Variable geometry convergent-divergent |

---

## 📊 Validation (Model vs. GasTurb)
The MATLAB code was fine-tuned to match the design point data provided by GasTurb 13 software.

* **Target Thrust (GasTurb):** ~17.08 kN
* **Calculated Thrust (MATLAB):** ~17.63 kN
* **Deviation:** ~3.2%

> *The simplified model successfully captures the "Off-Design" behavior, including the effects of altitude on air density and Mach number on ram pressure rise.*

## ⚠️ Limitations
0-D Analysis: Flow is assumed to be one-dimensional; radial profiles are ignored.

Map Scaling: Component maps (Compressor/Turbine maps) are not used; constant component efficiencies are assumed for the scope of this course.

Chemical Kinetics: Combustion is modeled via energy balance (LHV), not chemical reaction rates.

---

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>


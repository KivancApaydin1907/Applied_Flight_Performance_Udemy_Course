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

## 🔧 Methodology & Physics
The simulation utilizes a **"Mass Addition" method** combined with semi-empirical corrections to account for real-world losses. Unlike ideal cycle analysis, this model includes:

* **Variable Gas Properties:** Specific heat ($C_p$) and Gamma ($\gamma$) change between cold (compressor) and hot (turbine/AB) sections.
* **Ram Recovery:** Mil-Spec inlet pressure recovery based on Mach number.
* **Afterburner Losses:** Accounting for flame holder drag ($\pi_{ab} \approx 0.92$).
* **Nozzle Friction:** Implementation of a Velocity Coefficient ($C_v$) to correct ideal exit velocity.
* **Cooling Air Mixing:** Simulates the momentum loss of bypass cooling air re-entering the exhaust stream.

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

---

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>

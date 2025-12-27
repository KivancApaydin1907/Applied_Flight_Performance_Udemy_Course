# Applied Flight Performance: Analysis with MATLAB & OpenVSP

![Project Status](https://img.shields.io/badge/Status-Active_Development-brightgreen)
![MATLAB](https://img.shields.io/badge/MATLAB-R2020b%2B-blue)
![OpenVSP](https://img.shields.io/badge/OpenVSP-3.30%2B-orange)
![Course](https://img.shields.io/badge/Udemy-Official_Course-red)

> **"Build your own Physics Engine. Design your own Mission."**

Welcome to the official repository for the **"Applied Flight Performance: Analysis with MATLAB & OpenVSP"** course. 

This project goes beyond standard textbook theory. We simulate a complete real-world engineering workflow: extracting aerodynamic data from **OpenVSP**, modeling the **J85-GE-5** turbojet engine from scratch, and building a custom **6-DOF (reduced to 3-DOF) Physics Engine** to simulate the entire mission profile of the **Northrop T-38 Talon**—from brake release to landing flare.

---

## 🚀 Project Overview & Capabilities

This repository houses a modular flight dynamics framework capable of simulating a **Full Mission Profile**. Unlike simple point-mass estimations, this project implements:

* **Custom Physics Engine:** A non-linear equation of motion solver written in MATLAB (using `ode45`).
* **Propulsion Modeling:** Dual-approach engine modeling using both **GasTurb** data and a custom **Parametric Cycle Analysis (PCA)** written in MATLAB ("White Box" model).
* **Automatic Flight Control:** Implementation of **PID Controllers** for pitch-hold, altitude-hold, and velocity-hold modes during Climb and Cruise.
* **Complex Maneuvers:** Logic for ground roll, rotation ($V_R$), lift-off, and the mathematical modeling of the **Landing Flare**.

## 📚 Curriculum & Technical Workflow

The codebase is structured to mirror the progression of the course modules:

### Phase 1: The Digital Twin (Data Generation)
Before simulation, we build the aircraft digitally.
* **Aerodynamics (OpenVSP):** Generating $C_L$, $C_D$, $C_m$ lookup tables and analyzing stability derivatives ($C_{m_{\delta}}$, $C_{L_{\delta}}$).
* **Propulsion (GasTurb & MATLAB):** * *Method A:* Batch processing/parsing raw Excel data from GasTurb 13.
    * *Method B:* Writing a thermodynamic cycle analysis script for the J85-GE-5 engine (Intake $\to$ Nozzle) to generate Thrust & SFC maps dynamically.

### Phase 2: The Physics Engine (Simulation)
We build the `main_mission.m` script to integrate the Equations of Motion over time.
* **Take-off:** Ground roll friction, drag, and rotation logic.
* **Climb:** Rate of Climb (RC) optimization and altitude capture.
* **Cruise:** Breguet range integration and fuel flow mass updates.
* **Descent & Landing:** Energy management and the exponential decay flare maneuver.

## 📂 Repository Structure

```text
├── 01_Aerodynamics_Database/      % OpenVSP exports and geometric data
│   ├── VSP_Geom/                  % .vsp3 files (T-38 Talon)
│   └── Aero_Data/                 % Processed .mat files (CL, CD, Cm vs Alpha/Mach)
│
├── 02_Propulsion_Model/           % Engine Performance Data
│   ├── GasTurb_Data/              % Raw .xlsx exports (Dry & Wet Thrust)
│   ├── MATLAB_PCA_Model/          % J85_Engine_Cycle.m (Thermodynamic script)
│   └── Thrust_Lookup_Tables.mat   % Final 3D matrices for simulation
│
├── 03_Flight_Dynamics_Sim/        % The Core Physics Engine
│   ├── Modules/
│   │   ├── GroundRoll.m
│   │   ├── Climb_PID.m            % PID Controller for Climb Phase
│   │   ├── Cruise_Dynamics.m
│   │   └── Landing_Flare.m        % Flare control logic
│   └── Main_Mission_Profile.m     % Master execution script
│
├── 04_Utils/                      % Helper functions
│   ├── grabit.m                   % Data digitization tool
│   └── unit_conversions.m
│
└── README.md
```

## 🛠️ Key Technologies
* MATLAB: The backbone of the simulation (Optimization Toolbox recommended for PID tuning).

* OpenVSP (NASA): Used for rapid aerodynamic analysis and stability derivative estimation.

* GasTurb 15: Used for generating baseline engine performance envelopes.

## 🚧 Development Status
[x] Aerodynamic Database: Complete (OpenVSP V3.30).

[x] Propulsion Model: Complete (Both GasTurb & MATLAB PCA methods).

[x] Take-off Module: Complete.

[x] Climb & Cruise Modules: Complete (PID Tuned).

[ ] Final Polish: Landing Flare fine-tuning and code cleanup. 

## 🤝 Contributing
This repository accompanies the Udemy course. While Pull Requests are welcome, the code is primarily educational. If you find a bug in the EOM derivation or the lookup table interpolation, please open an Issue!

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>

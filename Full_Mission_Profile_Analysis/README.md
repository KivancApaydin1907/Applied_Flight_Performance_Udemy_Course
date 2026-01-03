<div align="center">

# T-38 Talon: High-Fidelity Flight Dynamics & Autopilot Simulation

![Status](https://img.shields.io/badge/Status-Complete-success)
![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-blue)
![SimType](https://img.shields.io/badge/Simulation-3--DOF-orange)
![Course](https://img.shields.io/badge/Udemy-Official_Course-red)

> **"From Takeoff to Touchdown: A complete physics-based mission simulation framework."**

</div>

<p align="center">
  This repository contains the source code for the capstone project of the <strong>"Applied Flight Performance"</strong> engineering course. It simulates the full mission profile of a supersonic trainer aircraft (Northrop T-38 Talon), featuring a custom physics engine and a modular autopilot architecture built entirely from scratch in MATLAB.
</p>

---

## 🚀 Project Overview

This is not just a simple cruise performance calculator. It is a time-marching integration of non-linear Equations of Motion (EoM) that handles the aircraft's dynamics through **11 distinct flight phases**. The simulation manages complex transitions such as the rotation maneuver ($V_R$), aerodynamic braking during rollout, and a precision landing flare.

### Key Capabilities
* **Physics Engine:** Custom 3-DOF point-mass solver accounting for variable mass, atmosphere (ISA), and aerodynamic databases.
* **Universal Autopilot:** A modular `RunPID` architecture that handles **Cascade Control Loops** (e.g., Altitude $\to$ Pitch $\to$ Elevator).
* **Full Envelope Protection:** Alpha limiters and Gamma constraints during the approach phase to prevent stall.

## 🕹️ Mission Profile Breakdown

The simulation autonomously flies the aircraft through the following sequence, exactly as defined in `Main_Mission_Profile.m`:

| Phase | Description | Control Strategy |
| :--- | :--- | :--- |
| **01-03** | **Takeoff & Rotation** | Open Loop Throttle / PD Pitch Control for Rotation ($12^\circ$) |
| **04** | **Initial Climb** | Attitude Hold until Obstacle Clearance ($15m$) |
| **05** | **Climb** | **Constant Mach (0.6)** / Pitch holds Attitude |
| **06** | **Cruise** | **Altitude Hold (11km)** / Cascade Loop (Alt $\to$ Pitch) |
| **07** | **Descent** | **Constant Alpha Mode** ($8^\circ$) / Idle Thrust / Speedbrakes |
| **08** | **Approach** | **Gamma Constraint** ($-7^\circ$) with Alpha Limiter ($<12^\circ$) |
| **09** | **Flare** | **Precision Landing** / Sink Rate Control ($<1.2 m/s$) |
| **10** | **Derotation** | Controlled Nose Lowering (Active Suspension Logic) |
| **11** | **Braking** | Aerodynamic Braking (Stick Aft) + Wheel Brakes ($\mu = 0.5$) |

## 📂 Repository Structure

The codebase is organized to separate the physics engine from the mission logic:

```text
├── Performance_Analysis.m       % MASTER SCRIPT: Runs the 11-phase simulation loop
└── README.md                    % Documentation
```
<h2>🛠️ Technical Implementation Details</h2>

<h3>1. The Physics Solver</h3>
<p>The simulation integrates the state vector <code>[u, w, q, &theta;, x_E, z_E, m]</code> using a fixed-step Euler method (<code>dt = 0.01s</code> for critical phases).</p>

<h3>2. Cascade Control Architecture</h3>
<p>For the Cruise phase, a <strong>Cascade Control System</strong> is implemented to maintain altitude:</p>
<ul>
  <li><strong>Outer Loop:</strong> Reads Altitude Error &rarr; Outputs Target Pitch Angle.</li>
  <li><strong>Inner Loop:</strong> Reads Pitch Error &rarr; Outputs Elevator Deflection.</li>
</ul>

<h3>3. Landing Logic</h3>
<p>The landing phase (Phase 09) uses a specialized logic where the throttle is modulated to maintain a specific <strong>Sink Rate</strong> (Vertical Speed), while the elevator maintains a safe touchdown attitude, ensuring a "Butter" landing (< 1.0 m/s impact).</p>

<h2>📊 Visualization</h2>
<p>The script automatically generates a comprehensive <strong>Mission Analysis Dashboard</strong> upon completion, plotting:</p>
<ul>
  <li>Flight Path (Trajectory)</li>
  <li>Control Inputs (&delta;<sub>e</sub>, &delta;<sub>T</sub>)</li>
  <li>Kinematics (Velocity, Mach, RoC)</li>
  <li>Forces & Moments</li>
</ul>

<div align="center">
  
<img width="728" height="367" alt="image" src="https://github.com/user-attachments/assets/31b60460-cce8-4496-9b95-610407b93a1b" />

<p><em>Figure 1: Mission Performance Summary Table generated automatically at the end of the simulation.</em></p>

<br>

<img width="1836" height="851" alt="Mission Profile Analysis" src="https://github.com/user-attachments/assets/2337e16b-3e0d-4c52-bfb7-b311738bcd8d" />

<p><em>Figure 2: Comprehensive Mission Analysis Dashboard visualizing the complete 11-phase flight profile including kinematics, forces, and control inputs.</em></p>

</div>

<h2>🤝 Contributing</h2>
<p>This project is part of an educational curriculum. Suggestions for optimizing the ODE solver or refining the PID gains are welcome!</p>
<ol>
  <li>Fork the Project</li>
  <li>Create your Feature Branch (<code>git checkout -b feature/AmazingFeature</code>)</li>
  <li>Commit your Changes (<code>git commit -m 'Add some AmazingFeature'</code>)</li>
  <li>Push to the Branch (<code>git push origin feature/AmazingFeature</code>)</li>
  <li>Open a Pull Request</li>
</ol>

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>

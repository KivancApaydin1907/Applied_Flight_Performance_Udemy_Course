<div align="center">

# T-38 Talon: Full Mission Profile Simulation

![Status](https://img.shields.io/badge/Status-Mission_Accomplished-success)
![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-blue)
![SimType](https://img.shields.io/badge/Simulation-NonLinear_3DOF-orange)
![Architecture](https://img.shields.io/badge/Control-Gain_Scheduled-purple)

> **"The Executive Script: Integrating high-fidelity physics with phase-dependent control laws for a complete Gate-to-Gate sortie."**

</div>

<p align="center">
  This is the <strong>Master Executive Script</strong> of the project. It acts as the Flight Control Computer (FCC), integrating the Physics Engine (<code>FlightDynamics.m</code>) with the Autopilot Logic (<code>RunPID.m</code>). It autonomously flies the aircraft through 11 distinct flight phases, switching control architectures and gains dynamically in real-time.
</p>

---

## 🚀 Simulation Architecture

Unlike the isolated "Tuner" scripts, this simulation runs continuous, time-marching integration where the end-state of one phase becomes the initial condition of the next.

### Key Features
* **🧠 Gain Scheduling:** The autopilot is not static. It dynamically reconfigures PID gains and limits based on the flight regime (e.g., stiff pitch control during landing vs. soft control during cruise).
* **💾 Checkpoint Generation:** As the mission progresses, the script automatically saves state vectors (`Rotation_Checkpoint.mat`, `Flare_Checkpoint.mat`, etc.). These snapshots feed the isolated optimization tuners.
* **🛡️ State Machine Logic:** The code utilizes discrete `while` loops to represent flight phases, transitioning only when specific physical criteria are met (e.g., `Lift > Weight`, `Mach > 0.6`).

---

## 🕹️ Mission Profile Breakdown

The simulation executes the following sequence logic defined in `Full_Mission_Simulation.m`:

| Phase | Description | Control Architecture | Target |
| :--- | :--- | :--- | :--- |
| **01-02** | **Ground Roll** | Open Loop | Max Power / Stick Neutral |
| **03** | **Rotation** | **SISO PID** (Soft Pitch) | $\theta = 12^\circ$ |
| **04** | **Initial Climb** | Attitude Hold | Obstacle Clearance ($15m$) |
| **05** | **Climb** | **MIMO Control** (Coupled) | $Mach = 0.6$ / $\theta = 10^\circ$ |
| **06** | **Cruise** | **Cascade Control** | $h = 11,000m$ / $Mach = 0.8$ |
| **07** | **Descent** | **Energy Management** | $\alpha = 8^\circ$ (Constant AoA) |
| **08** | **Approach** | **Envelope Protection** | $\gamma = -7^\circ$ / Limit $\alpha < 12^\circ$ |
| **09** | **Flare** | **Precision Landing** | $\dot{h} = -1.2 m/s$ / $\theta = 6^\circ$ |
| **10** | **Derotation** | **Active Damping** | Soft Nose Drop ($\dot{q} > -3^\circ/s$) |
| **11** | **Rollout** | Kinematic Braking | Max Deceleration |

---

## 📂 Repository Structure

This folder contains the executive logic and the resulting flight data:

```text
Full_Mission_Profile_Analysis/
│
├── Full_Mission_Simulation.m   # THE BOSS CODE: Runs the complete 11-phase loop
└── README.md                   # Documentation
```

<h2>📊 Visualization</h2>
<p>The script automatically generates a comprehensive <strong>Mission Analysis Dashboard</strong> upon completion, plotting:</p>
<ul>
  <li>Flight Path (Trajectory)</li>
  <li>Control Inputs (&delta;<sub>e</sub>, &delta;<sub>T</sub>)</li>
  <li>Kinematics (Velocity, Mach, RoC)</li>
  <li>Forces & Moments</li>
</ul>

<div align="center">
  
<img width="738" height="366" alt="image" src="https://github.com/user-attachments/assets/3c8ffb99-5342-4de7-b8d2-0197a43afb09" />

<p><em>Figure 1: Mission Performance Summary Table generated automatically at the end of the simulation.</em></p>

<br>

<img width="1836" height="851" alt="Mission Profile Analysis" src="https://github.com/user-attachments/assets/81bda72d-483c-4776-93a2-564c5a17f55a" />

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

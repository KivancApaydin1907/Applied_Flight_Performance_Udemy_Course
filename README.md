# Applied Flight Performance & Control: Theory to Simulation

[![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-blue?logo=mathworks)](https://www.mathworks.com/products/matlab.html)
[![Aerospace](https://img.shields.io/badge/Domain-Aerospace_Engineering-orange)]()
[![Subject](https://img.shields.io/badge/Subject-Flight_Dynamics_%26_Control-purple)]()
[![CourseStatus](https://img.shields.io/badge/Udemy_Course-AVAILABLE_NOW-brightgreen)]() [![License](https://img.shields.io/badge/License-MIT-green)]()

> **"A complete, physics-based flight simulation framework bridging the gap between theoretical aerodynamics and practical mission analysis."**

<div align="center">

<h3>🚀 The Course is Live! 🚀</h3>
<p>
  This repository acts as the <b>official companion codebase</b> for the Udemy course. 
  <br>
  Start building your own flight simulator today:
</p>

<a href="https://www.udemy.com/course/applied-flight-performance-and-control/">
  <img src="https://img.shields.io/badge/ENROLL%20NOW-Udemy-eb4d4b?style=for-the-badge&logo=udemy" alt="Enroll on Udemy">
</a>

</div>

---

## ✈️ Project Overview

This repository houses the source code, datasets, and simulation scripts for the **"Applied Flight Performance & Control"** engineering course.

Using the **Northrop T-38 Talon** as a primary case study, this project demonstrates how to build a high-fidelity **3-DOF Flight Simulator** from scratch. Unlike standard textbook examples, this framework integrates real-world complexities:

* **Non-Linear Aerodynamics:** Calibrated CFD data (**OpenVSP**) validated against wind tunnel experiments.
* **Dynamic Propulsion:** J85-GE-5 turbojet performance maps varying with Altitude and Mach (Generated via **GasTurb**).
* **Adaptive Autopilot:** A gain-scheduled flight control system that handles 11 distinct mission phases.

---

## 📚 Course Modules & Repository Structure

This project is modular by design. Each folder corresponds to a specific learning module in the course curriculum:

| Module | Description | Key Tech |
| :--- | :--- | :--- |
| **[`Flight_Physics_Engine`](./Flight_Physics_Engine)** | **Core Solver.** Calculates Forces, Moments, and State Derivatives (Equations of Motion). | `MATLAB`, `ODE Solvers` |
| **[`Automatic_Control`](./Automatic_Control)** | **Autopilot Design.** Optimization scripts for tuning PID gains across different flight regimes (Rotation, Flare, etc.). | `fminsearch`, `PID Control` |
| **[`Full_Mission_Profile_Analysis`](./Full_Mission_Profile_Analysis)** | **Executive Script.** Integrates engine and autopilot to fly a complete "Gate-to-Gate" sortie. | `System Integration` |
| **[`Calibrate_OpenVSP_Data`](./Calibrate_OpenVSP_Data)** | **Data Science.** System ID tools to correct inviscid VSP data using experimental wind tunnel results. | `Curve Fitting` |
| **[`J-85-GE_Thrust_Data`](./J-85-GE_Thrust_Data)** | **Propulsion.** Digitized performance maps for the T-38's Turbojet engines. | `GasTurb`, `Interpolation` |
| **[`Data_Handling`](./Data_Handling)** | **Utilities.** Scripts for processing drag polars, atmospheric models, and unit conversions. | `Data Processing` |

---

## 🚀 Getting Started

To run the full mission simulation on your local machine:

1.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/KivancApaydin1907/Applied_Flight_Performance_Udemy_Course.git](https://github.com/KivancApaydin1907/Applied_Flight_Performance_Udemy_Course.git)
    ```

2.  **Setup MATLAB Path:**
    Open MATLAB and ensure all subfolders are added to your path (Right Click Folder -> *Add to Path* -> *Selected Folders and Subfolders*).

3.  **Run the Simulation:**
    Navigate to `Full_Mission_Profile_Analysis` and execute:
    ```matlab
    Full_Mission_Simulation
    ```

4.  **Analyze Results:**
    The script will output a mission summary to the command window and generate dashboard plots (Trajectory, Energy States, Control Inputs).

---

## 📊 Sample Output: Mission Dashboard

*The simulation automatically generates telemetry for trajectory, energy states, and control inputs.*

<img width="1920" height="1080" alt="udemy_2" src="https://github.com/user-attachments/assets/8077d14d-a1c2-41b5-aca6-b5e67c51a72f" />

*(Note: Visuals are generated directly from the course capstone project.)*

---

## 🛠️ Tech Stack

* **Language:** MATLAB R2025b (Backward compatible to R2020a)
* **Aerodynamics:** OpenVSP (Vortex Lattice Method)
* **Propulsion:** GasTurb (For engine cycle data generation)
* **Optimization:** Nelder-Mead Simplex Method (`fminsearch`)
* **Control Theory:** Classical PID, Cascade Control, Gain Scheduling

---

## 👨‍💻 Author & Instructor

**Kıvanç Apaydın**
*Aeronautical Engineer | Flight Physics*
* [LinkedIn Profile](https://www.linkedin.com/in/k%C4%B1van%C3%A7-apayd%C4%B1n-3a53a8259/)

---

_© 2026 Applied Flight Performance Course. All rights reserved._

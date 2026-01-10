<div align="center">

# Applied Flight Performance: Analysis with MATLAB & OpenVSP

![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-blue?logo=mathworks)
![Aerospace](https://img.shields.io/badge/Domain-Aerospace_Engineering-orange)
![Subject](https://img.shields.io/badge/Subject-Flight_Dynamics_&_Control-purple)
![CourseStatus](https://img.shields.io/badge/Udemy_Course-In_Development-yellow)
![License](https://img.shields.io/badge/License-MIT-green)

> **"A complete, physics-based flight simulation framework bridging the gap between theoretical aerodynamics and practical mission analysis."**

[**View Course Roadmap**](#-udemy-course--development-status) | [**Explore Modules**](#-repository-architecture) | [**Report Bug**](../../issues)

</div>

---

## ✈️ Project Overview

This repository houses the source code, datasets, and simulation scripts for the upcoming **"Applied Flight Performance"** engineering course.

Using the **Northrop T-38 Talon** as a primary case study, this project demonstrates how to build a high-fidelity **3-DOF Flight Simulator** from scratch. Unlike standard textbook examples, this framework integrates real-world complexities:
* **Non-Linear Aerodynamics:** Calibrated CFD data (OpenVSP) against wind tunnel experiments.
* **Dynamic Propulsion:** J85-GE-5 turbojet performance maps varying with Altitude and Mach.
* **Adaptive Autopilot:** A gain-scheduled flight control system that handles 11 distinct mission phases.

---

## 🚧 Udemy Course & Development Status

<div align="center">
  <h3>🎥 Applied Flight Performance: Master Class (Coming Soon)</h3>
  <p><em>This repository serves as the practical companion for an in-depth video course currently under production for Udemy.</em></p>
</div>

**Current Development Status:**
- [x] **Module 1:** Aerodynamic Data Generation (OpenVSP)
- [x] **Module 2:** Propulsion System Modeling (Thrust & SFC Maps)
- [x] **Module 3:** 3-DOF Physics Engine Implementation
- [x] **Module 4:** Autopilot Design & PID Tuning
- [ ] **Module 5:** Course Video Production & Editing (**In Progress**)

> **🔔 Stay Tuned:** The course will guide students line-by-line through the creation of this entire codebase.

---

## 📂 Repository Architecture

This project is modular by design. Each folder represents a specific engineering subsystem:

| Module | Description | Key Tech |
| :--- | :--- | :--- |
| **[`Flight_Physics_Engine`](./Flight_Physics_Engine)** | The core 3-DOF solver calculating Forces, Moments, and State Derivatives. | `Equations of Motion` |
| **[`Automatic_Control`](./Automatic_Control)** | Optimization scripts for tuning PID gains across different flight regimes (Rotation, Flare, etc.). | `fminsearch`, `PID` |
| **[`Full_Mission_Profile_Analysis`](./Full_Mission_Profile_Analysis)** | **The Executive Script.** Integrates engine and autopilot to fly a complete "Gate-to-Gate" sortie. | `Simulation`, `Integration` |
| **[`Calibrate_OpenVSP_Data`](./Calibrate_OpenVSP_Data)** | System ID tools to correct inviscid VSP data using experimental wind tunnel results. | `Data Fitting` |
| **[`J-85-GE_Thrust_Data`](./J-85-GE_Thrust_Data)** | Digitized performance maps for the T-38's Turbojet engines. | `Propulsion` |
| **[`Data_Handling`](./Data_Handling)** | Utilities for processing drag polars and atmospheric data. | `Data Science` |

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
    The script will output a mission summary to the command window and generate dashboard plots.

---

## 📊 Sample Output: Mission Dashboard

*The simulation automatically generates telemetry for trajectory, energy states, and control inputs.*

<div align="center">
  <img src="https://github.com/user-attachments/assets/81bda72d-483c-4776-93a2-564c5a17f55a" alt="Mission Profile Dashboard" width="90%">
</div>

---

## 🛠️ Tech Stack

* **Language:** MATLAB R2025b (Backward compatible to R2020a)
* **Aerodynamics:** OpenVSP (Vortex Lattice Method)
* **Optimization:** Nelder-Mead Simplex Method (`fminsearch`)
* **Control Theory:** Classical PID, Cascade Control, Gain Scheduling

---

## 👨‍💻 Author & Instructor

**Kıvanç Apaydın**
*Aerospace Engineer | Simulation Developer*

* [LinkedIn Profile](www.linkedin.com/in/kıvanç-apaydın-3a53a8259)
* [GitHub Profile](https://github.com/KivancApaydin1907)

---

<div align="center">
  <sub>&copy; 2026 Applied Flight Performance Course. All rights reserved.</sub>
</div>

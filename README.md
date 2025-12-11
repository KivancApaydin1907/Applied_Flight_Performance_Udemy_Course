# Aircraft Performance Analysis with MATLAB

Welcome to the official repository for the **"Aircraft Performance & Flight Dynamics Analysis"** course on Udemy.

This repository contains all the MATLAB source codes, data files, and simulation scripts used throughout the course. The projects are designed to bridge the gap between theoretical aerodynamics and practical flight simulation using the **Northrop T-38 Talon** as the primary case study.

## 📚 Course Overview

In this course, we build a modular flight performance analysis tool from scratch. The code base covers the following engineering pillars:

* **Standard Atmosphere:** Implementation of ISA models (Troposphere & Stratosphere).
* **Aerodynamics:** Importing and processing CFD data from **OpenVSP** (Lift/Drag Polars).
* **Propulsion:** Turbojet thrust modeling and specific fuel consumption (TSFC) mapping.
* **Equations of Motion (EOM):** Solving point-mass aircraft dynamics using MATLAB's ODE solvers (`ode45`).
* **Performance Metrics:** Calculating Rate of Climb, Turn Radius, Range, and Endurance.

## 📂 Repository Structure

The repository is organized into logical modules corresponding to the course sections:

```text
├── 01_Atmosphere       % ISA Calculator and density modeling
├── 02_Aerodynamics     % OpenVSP import scripts and Drag Polar build
├── 03_Propulsion       % Engine thrust database and interpolation
├── 04_Performance      % Core simulation scripts (Climb, Cruise, Turn)
├── data                % .txt (OpenVSP) and .mat (Processed) files
└── README.md
```

## 🛠️ Requirements
MATLAB (R2020b or later recommended).

Curve Fitting Toolbox (Optional, for some specific polynomial fits).

OpenVSP (Only if you intend to generate your own aerodynamic data).

## 🔗 Course Link
(not fully prepared yet)

## 🤝 Contributing & Support
If you encounter any bugs in the code or have suggestions for optimization, please open an Issue or submit a Pull Request. For specific course questions, please use the Q&A section on Udemy.

---

## 🚧 Project Status & Disclaimer

**This repository is currently under active development.**

Please note that the Udemy course associated with this repository is being recorded and prepared simultaneously. As a result:
* New scripts, data files, and modules are **uploaded incrementally** as they are finalized.
* Some folders might be empty or contain placeholder files temporarily.
* The code structure may undergo minor refactoring for optimization.

Feel free to **⭐ Star** or **👁️ Watch** the repository to get notified about the latest updates and module releases!

---

## 📝 License
This project is licensed under the MIT License - see the LICENSE file for details.

## 👨‍💻 Author

**Kıvanç Apaydın** – Aerospace Engineer  




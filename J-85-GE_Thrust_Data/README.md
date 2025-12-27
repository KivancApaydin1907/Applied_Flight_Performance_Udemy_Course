# 🚀 J85-GE Turbojet Propulsion Models

This directory contains the engine performance data generation modules for the T-38 Talon Flight Dynamics Simulation. 

To provide a comprehensive understanding of propulsion modeling, we apply two different approaches to obtain **Net Thrust ($F_n$)** and **Specific Fuel Consumption ($SFC$)**.

## 📂 Modules

### 1. [via_GasTurb](./via_GasTurb)
* **Method:** **High-Fidelity Cycle Analysis** (Software Export).
* **Description:** Imports complex cycle data exported from **GasTurb 15**. It uses raw Excel data to generate interpolated 3D Lookup Tables for the simulation.
* **Use Case:** Primary source for the 6-DOF simulation due to higher accuracy across the flight envelope.

### 2. [via_MATLAB](./via_MATLAB)
* **Method:** **Analytical Parametric Cycle Analysis**.
* **Description:** Calculates engine performance using fundamental thermodynamic equations (Brayton Cycle) and component efficiencies directly within MATLAB.
* **Use Case:** Educational purposes; demonstrates the math behind the engine cycle.

---
> 💡 **Navigation:** Please click on the folder names above to access the specific scripts, data files, and detailed technical documentation for each method.

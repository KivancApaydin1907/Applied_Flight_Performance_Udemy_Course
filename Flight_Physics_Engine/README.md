# T-38 Flight Physics Engine

**A high-fidelity, non-linear 3-DOF flight dynamics model encapsulating the Aerodynamics, Propulsion, and Equations of Motion for the Northrop T-38 Talon.**

### ⚙️ Core Architecture

This engine (`FlightDynamics.m`) serves as the "Digital Twin" of the aircraft. It transforms control inputs and environmental states into accelerations using a rigorous physics pipeline:

`Inputs (State, Controls) -> [ Aerodynamics | Propulsion | MassProps ] -> Forces & Moments -> Accelerations -> Derivatives`

---

### 🧠 Physics Modules

#### 1. Aerodynamics (Semi-Empirical)
* **Lift ($C_L$):** Uses a hybrid model combining linear stability derivatives with high-fidelity lookup tables ($f(\alpha, Mach)$) derived from OpenVSP and wind tunnel data.
* **Drag ($C_D$):** Implements a Drag Polar model:
    $$C_D = C_{D_0}(M) + K(M) \cdot C_L^2 + \Delta C_{D_{gear}} + \Delta C_{D_{speedbrake}}$$
* **Pitching Moment ($C_m$):**
    $$C_m = C_{m_0} + C_{m_\alpha}\alpha + C_{m_{\delta_e}}\delta_e$$

#### 2. Propulsion (J85-GE-5 Turbojet)
* **Dual-Mode Operation:** Simulates both **Dry (Military)** and **Wet (Afterburner)** power regimes.
* **Environmental Sensitivity:** Thrust and Specific Fuel Consumption (SFC) are dynamically interpolated based on Altitude ($h$) and Mach Number ($M$).
* **Fuel Logic:** Calculates real-time mass depletion ($\dot{m}_{fuel}$) based on throttle setting and SFC maps.

#### 3. Equations of Motion (3-DOF Longitudinal)
Solves the rigid-body dynamics in the **Body Frame**, including Coriolis effects and Ground Reactions.

* **Axial Acceleration ($\dot{u}$)**   
* **Normal Acceleration ($\dot{w}$)**
* **Pitch Acceleration ($\dot{q}$)**
 

---

### 📂 Input/Output Structure

**Inputs:**
* `State`: Current kinematic state vector ($u, w, q, \theta, x_E, z_E, mass$).
* `AC`: Aircraft database structure containing geometry, mass props, and aero tables.
* `Env`: Atmospheric properties ($\rho, a, g$).
* `Controls`: Pilot inputs ($\delta_e, \delta_{th}, \delta_{sb}, \delta_{gear}$).

**Outputs:**
* `Derivatives`: State rates ($\dot{u}, \dot{w}, \dot{q}, \dots$) for the ODE solver.
* `Forces/Moments`: Intermediate variables for telemetry and analysis.
* `Logs`: Computed variables like Mach, $\alpha$, $\gamma$, and Stall Speed ($V_{stall}$).

---

### 🛡️ Ground Handling Logic

The engine includes a dedicated ground interaction model to simulate:
* **Normal Force ($N$):** Reaction force from the runway prevents underground motion.
* **Friction ($F_f$):** $\mu \cdot N$ logic for rolling resistance and braking.
* **Transition:** Automatically detects lift-off when $L > W \cos(\theta)$.

---

### 🔧 Usage Example

```matlab
% 1. Define Initial Conditions
State.u = 150; % m/s
State.w = 0;
State.theta = 0;
...

% 2. Define Controls
Controls.ThrottleSetting = 1.0; % Max Power
Controls.ElevatorDeflection = deg2rad(-5);

% 3. Run Physics Step
Output = FlightDynamics(State, AC, Env, Controls);

% 4. Access Derivatives for Integration
u_dot = Output.Derivatives.u_dot;
```
<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>

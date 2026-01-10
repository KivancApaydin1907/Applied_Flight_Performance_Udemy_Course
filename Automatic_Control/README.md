# T-38 Full-Envelope Autopilot Tuner

**An automated control system optimization suite that tunes flight control laws for the T-38 Talon across the entire mission profile, from takeoff rotation to nose-gear derotation.**

### 🚀 Overview
Designing an autopilot for a supersonic trainer requires more than just PID guessing. It requires a robust tuning strategy that adapts to the changing physics of flight.

This project implements a **Physics-Based Optimization Framework** using MATLAB. Instead of manual tuning, the suite utilizes `fminsearch` (Nelder-Mead algorithm) to fly thousands of virtual sorties, minimizing custom cost functions for each flight phase. The result is a set of optimized gains that balance **tracking precision**, **actuator efficiency**, and **structural safety**.

---

### 🧠 The Control Engine: `RunPID.m`

At the heart of every control loop is a custom-built, robust PID function designed for non-linear aerospace applications.

**Key Features:**
* **🛡️ Anti-Windup (Conditional Integration):** Prevents the integrator from accumulating error when the actuator is saturated (hitting limits), ensuring instant recovery.
* **📉 Derivative on Measurement:** Calculates the D-term using the process variable rate (e.g., Gyro $q$) rather than error rate, preventing "Derivative Kick" during setpoint changes.
* **⏩ Feed-Forward Capability:** Supports base-input injection (Trim) for predictive control optimization.

---

### 📊 Optimization Strategy by Phase

The suite decomposes the flight envelope into 7 distinct phases, applying specific control architectures (MIMO, Cascade, Active Damping) for each.

#### 1. Takeoff & Climb
* **Rotation (`Controller_Optimizer_Rotation.m`):** Optimizes pitch-up authority while enforcing a **Tailstrike Barrier** (soft constraint).
* **Climb (`Controller_Optimizer_Climb.m`):** **MIMO Control**. Simultaneously tunes Pitch (Attitude) and Throttle (Airspeed) loops, accounting for the aerodynamic coupling between thrust and lift.

#### 2. Cruise & Descent
* **Cruise (`Controller_Optimizer_Cruise.m`):** **Cascade Control**. An outer loop manages Altitude ($h$) by commanding an inner Pitch ($\theta$) loop.
* **Descent (`Controller_Optimizer_Descent.m`):** **Energy Management**. Tunes the autopilot to hold a constant Angle of Attack ($\alpha$) for maximum drag descent, while penalizing "ballooning" (unintended climbs).

#### 3. Landing Sequence (The Critical Phase)
* **Approach (`Controller_Optimizer_Approach.m`):** **Envelope Protection**. Tracks Flight Path Angle ($\gamma$) while strictly enforcing an Alpha Limit ($12^\circ$) to prevent stall during steep approaches.
* **Flare (`Controller_Optimizer_Flare.m`):** Precision landing logic. Optimizes for a "Butter" touchdown ($w_{sink} \approx -1.2 m/s$) by modulating throttle and pitch in high-fidelity time steps ($50Hz$).
* **Derotation (`Controller_Optimizer_Derotation.m`):** **Active Damping**. Functions as a virtual shock absorber, using the elevator to gently lower the nose gear to the runway, minimizing structural impact loads.

---

### 🔧 Methodology

The optimization loop follows this logic:

1.  **Initialization:** Loads the aircraft state from the previous phase checkpoint (e.g., `Climb_Checkpoint.mat`).
2.  **Simulation:** Runs the non-linear 3-DOF flight dynamics engine.
3.  **Cost Function Evaluation ($J$):**
    $$J = w_1 \cdot |Error| + w_2 \cdot |Actuator_{sat}| + w_3 \cdot |Safety_{constraint}|$$
    * *Error:* Deviation from target (e.g., Altitude, Speed).
    * *Actuator Saturation:* Penalizes "Bang-Bang" control to ensure passenger comfort.
    * *Safety:* Heavy penalties for stall, hard landings, or tailstrikes.
4.  **Convergence:** The optimizer iterates gains ($K_p, K_i, K_d$) until the global minimum cost is found.

---

### 📂 Repository Structure

```text
Automatic_Control/
│
├── RunPID.m                          # Core PID Logic with Anti-Windup
├── Controller_Optimizer_Rotation.m   # Phase 1: Takeoff Rotation
├── Controller_Optimizer_Climb.m      # Phase 2: MIMO Climb Tuning
├── Controller_Optimizer_Cruise.m     # Phase 3: Cascade Altitude Hold
├── Controller_Optimizer_Descent.m    # Phase 4: Constant Alpha Descent
├── Controller_Optimizer_Approach.m   # Phase 5: Gamma Tracking & Protection
├── Controller_Optimizer_Flare.m      # Phase 6: Precision Touchdown
├── Controller_Optimizer_Derotation.m # Phase 7: Nose Gear Damping
└── README.md                         # Documentation
```

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>

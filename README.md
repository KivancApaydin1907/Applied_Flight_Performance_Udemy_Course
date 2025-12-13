<h1>Applied Aircraft Performance: Data-Driven Flight Simulation</h1>

<p>Welcome to the official repository for the <strong>"Applied Aircraft Performance"</strong> course on Udemy.</p>

<p>This repository hosts the MATLAB source codes, data datasets, and simulation scripts developed throughout the course. Unlike traditional theoretical courses, this project simulates a real-world engineering workflow: we extract data from technical references, generate aerodynamic/propulsive models using industry-standard tools, and build a custom physics engine to simulate the <strong>Northrop T-38 Talon</strong>.</p>

<h2>🚀 Project Scope</h2>

<p>The primary objective of this repository is to bridge the gap between <strong>Conceptual Design</strong> and <strong>Flight Mechanics</strong>. We move through three distinct phases:</p>

<ol>
  <li><strong>Data Acquisition:</strong> Extracting raw data from T-38 technical reports and generating new data using <strong>OpenVSP</strong> and <strong>GasTurb</strong>.</li>
  <li><strong>System Modeling:</strong> Deriving 6-DOF Equations of Motion (EOM) and reducing them to a solvable 3-DOF model.</li>
  <li><strong>Mission Simulation:</strong> Executing a physics-based simulation of the <strong>Take-off</strong> phase (Ground Roll &rarr; Rotation &rarr; Airborne).</li>
</ol>

<h2>📚 Course Modules & Repository Structure</h2>

<p>The code is organized to mirror the engineering design process:</p>

<pre>
├── 00_Appendix_DataTools   % 'Grabit' scripts and data smoothing algorithms
├── 01_Aerodynamics         % OpenVSP processed data (CL, CD, Cm, Drag Polars)
├── 02_Propulsion           % GE J85 Engine modeling (GasTurb data & interpolation)
├── 03_Dynamics_EOM         % 3-DOF Physics Engine & Integrator setup (ODE45)
├── 04_Simulation_Takeoff   % Ground roll, Rotation logic, and Climb transition
└── README.md
</pre>

<h2>🛠️ Tech Stack & Tools Used</h2>

<ul>
  <li><strong>MATLAB:</strong> Core simulation and logic (R2020b+ recommended).</li>
  <li><strong>OpenVSP:</strong> Used to generate aerodynamic coefficients (CL, CD, CM).</li>
  <li><strong>GasTurb:</strong> Used to generate thrust maps based on Altitude and Mach.</li>
  <li><strong>Grabit:</strong> Used to digitize legacy data from technical charts.</li>
</ul>

<h2>🔑 Key Features</h2>

<ul>
  <li><strong>Custom Physics Engine:</strong> We do not use Simulink blocks; we write the physics using Newton’s laws derived from the Body Frame to the Inertial Frame.</li>
  <li><strong>Real Data Integration:</strong> The simulation is fed by real interpolated lookup tables, not just constant values.</li>
  <li><strong>Modular Design:</strong> The EOM solver is written as a state-space representation, allowing for easy expansion into Cruise and Landing phases in the future.</li>
</ul>

<h2>🚧 Development Status</h2>

<p><strong>Active Development (Current Focus: Take-off Module)</strong></p>

<p>This repository is being updated synchronously with the course recording.</p>
<ul>
  <li>✅ <strong>Completed:</strong> Data Extraction, Aerodynamic Database.</li>
  <li>🔄 <strong>In Progress:</strong> EOM Implementation and Take-off Simulation.</li>
  <li>🔜 <strong>Future Roadmap:</strong> Climb, Cruise, Descent, and Landing modules.</li>
</ul>

<h2>🤝 Contributing</h2>
<p>Found a bug in the integration logic? Have a better drag polar fit? Please open an <strong>Issue</strong> or submit a <strong>Pull Request</strong>.</p>

<hr>

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>

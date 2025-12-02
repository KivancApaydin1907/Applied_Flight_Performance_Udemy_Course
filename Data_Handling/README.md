# Data Handling & Drag Polar Harmonization

This module focuses on processing raw, inconsistent aerodynamic data sources and converting them into a unified, simulation-ready format.

Specifically, it handles the **Drag-Due-to-Lift Factor ($K$)**, which changes with Mach number. The raw data comes from digitized charts where each $C_L$ line covers a different Mach range (e.g., some data stops at Mach 0.9, while others go to 1.2). This script harmonizes these discrepancies.

## 📂 Directory Structure

To maintain a clean workspace, raw digitized data files are stored in a subdirectory, while the processing logic resides in the root of this module.

```text
Data_Handling/
│
├── Inconsistent_Data/   % Contains raw .mat files (CL035, CL04, etc.)
│   ├── CL035.mat        % Data for CL <= 0.35
│   ├── CL04.mat         % Data for CL = 0.4
│   └── ...
│
├── Data_Handling.m      % Main processing script
└── README.md            % Documentation
```


## 🚀 Key Features

**Handling Inconsistent Ranges:** Raw datasets often have different starting and ending Mach numbers. This script maps them all to a common Mach_Common grid ($0 - 1.2$).

**Hybrid Extrapolation:** Uses a "Safe Interpolation" technique:
* Linear Interpolation: For values within the known data range.
* Nearest Neighbor Extrapolation: For Mach numbers outside the raw data limits (prevents wild oscillations common in polynomial extrapolation).

**Database Appending:** Instead of overwriting the existing T38_Aero_Data.mat, this script loads and appends the drag data to the existing structure, preserving the previously calculated Lift data.

## ⚙️ MethodologyThe core logic relies on normalizing the data arrays:

1) **Load:** Reads raw arrays from the Inconsistent_Data folder.
2) **Harmonize:** Applies fillmissing and interp1 to align all data to a standardized vector.
3) **Construct Matrix:** Builds a $(n_{CL} \times n_{Mach})$ matrix for 2D interpolation.
4) **Save:** Updates the master struct T38_Aerodynamics.Drag.

## 📊 Visualization

The script generates a comparative plot showing the raw digitized data points versus the harmonized grid, ensuring that the extrapolation logic is physical and sound. 

![drag_due_to_lift_factor](https://github.com/user-attachments/assets/0f0152b3-c706-48d9-a6ca-5921332cd90b)

## 🛠️ Usage

1) Ensure the Inconsistent_Data folder is in the same directory as the script.
2) Run Data_Handling.m.
3) The script will verify if T38_Aero_Data.mat exists:
  * If yes: It updates the file with Drag data.
  * If no: It creates a new file.

## 📄 Output Data

The updated .mat file will now include the Drag field:

```text
T38_Aerodynamics
├── Lift (Previously computed)
│   └── ...
└── Drag
    ├── K_Matrix      % (n_cl x n_mach) Harmonized Data
    ├── CL_Vector     % Vector for lookups
    ├── Mach_Vector   % Standardized Mach grid
    └── Description   % Processing notes
```

## ✍️ Author

**Kıvanç Apaydın** – Aerospace Engineer  

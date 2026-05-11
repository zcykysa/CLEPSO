# CLEPSO
三维牙齿正畸路径规划算法

---

```markdown
## 📂 Project Structure
```text
CLEPSO/
├── Data/                      # Sample 3D tooth model datasets (STL/PLY/VTK)
├── GroupFiles/                # Core algorithm modules
│   ├── GroupFdiMark.cpp/h     # Tooth FDI marking and identification
│   ├── GroupGridLine.cpp/h    # 3D grid visualization for spatial reference
│   ├── GroupMoveCalculation.cpp/h  # Tooth movement kinematics calculations
│   ├── GroupOcclusion.cpp/h   # OBB-based collision detection
│   └── GroupPathPlanningPSO.cpp/h  # CLEPSO path planning implementation
├── Common.h                   # Global constants, enums, and utility functions
├── GroupManager.cpp/h         # Module manager for coordinating algorithm components
├── MainWindow.cpp/h/ui/qrc    # Qt GUI implementation (main interface)
├── main.cpp                   # Application entry point
├── CMakeLists.txt             # CMake build configuration
└── README.md                  # Project documentation
```

---

## 🛠️ Environment Setup
| Category               | Tools/Libraries                                                                 |
|------------------------|---------------------------------------------------------------------------------|
| Core Language          | C++17                                                                           |
| Build System           | CMake 3.26+                                                                     |
| GUI Framework          | Qt 5.15.2 (MSVC2019_64)                                                         |
| 3D Visualization       | VTK 9.1.0                                                                       |
| Core Algorithm         | CLEPSO (Modified Particle Swarm Optimization)                                   |
| Collision Detection    | OBB (Oriented Bounding Box) + Separating Axis Theorem                            |
| Development Environment| Visual Studio 2019 (MSVC v142)                                                  |

---

## 📖 Usage Guide
### Step 1: Load Tooth Models
Use the Qt GUI to import 3D tooth mesh files (STL/PLY/VTK formats supported) from the `Data/` directory. The system will automatically parse tooth geometry and initialize coordinate systems.

### Step 2: Configure Parameters
- **Orthodontic Settings**: Set the number of treatment stages (20–50, per clinical standards) and physiological constraints (translation/rotation limits per stage).
- **Algorithm Parameters**: Adjust CLEPSO hyperparameters (population size, iteration count, inertia weights, etc.).

### Step 3: Run Path Planning
Click the "Start Planning" button to execute the CLEPSO algorithm. The system will compute collision-free, constraint-compliant tooth movement paths.

### Step 4: Visualize & Export Results
- View real-time 3D rendering of tooth movements in the VTK viewport.
- Export optimized path data (e.g., tooth poses per stage) for clinical appliance fabrication.
```

---


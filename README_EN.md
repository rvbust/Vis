English | [简体中文](./README.md)

<div align = "center">
<img src=./Data/Images/logo.png width="180"/>
</div>

# Vis: Asynchronous 3D Visualization Tool

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/std/the-standard)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows-green.svg)]()

Vis is an interactive asynchronous 3D visualization tool designed to make 3D vision and robotics development easier.

## ✨ Key Features

- 🎨 **Geometry Drawing** - Points, lines, surfaces, and various geometric primitives
- 📦 **3D Model Import** - Support for STL, DAE, 3DS and other common formats
- 🖱️ **Interactive Tools** - Multiple picking modes for object selection
- 🔧 **Gizmo** - Visual manipulator supporting translation, rotation, and scaling
- 🐍 **Python API** - Complete Python bindings for rapid development

---

## 📐 Architecture Design

### Overall Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Public API Layer                                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────────┐  ┌──────────────┐      │
│  │  View   │  │ Handle  │  │ Config  │  │ Geometry │  │ Interaction  │      │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬─────┘  └──────┬───────┘      │
└───────┼────────────┼────────────┼────────────┼───────────────┼──────────────┘
        │            │            │            │               │
        ▼            ▼            ▼            ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Command Layer                                   │
│  ┌──────────────────┐  ┌───────────────────┐  ┌─────────────────────┐       │
│  │  CommandQueue    │  │  CommandExecutor  │  │  CommandFactory     │       │
│  │  (Thread-safe)   │  │  (Async Execute)  │  │  (Type-safe)        │       │
│  └──────────────────┘  └───────────────────┘  └─────────────────────┘       │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Scene Layer                                     │
│  ┌───────────────┐  ┌──────────────────┐  ┌────────────────────────┐        │
│  │ SceneManager  │  │  NodeRegistry    │  │  TransformManager      │        │
│  │               │  │  (Handle→Node)   │  │  (Hierarchy)           │        │
│  └───────────────┘  └──────────────────┘  └────────────────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Geometry Factory Layer                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐     │
│  │PointFactory│  │ MeshFactory │  │ ShapeFactory│  │ ModelLoader     │     │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────┘     │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Interaction Layer                                    │
│  ┌─────────────────┐  ┌──────────────────┐  ┌────────────────────────┐      │
│  │ PickerManager   │  │  GizmoController │  │  CameraController      │      │
│  │ (Strategy)      │  │  (State Machine) │  │  (Manipulator)         │      │
│  └─────────────────┘  └──────────────────┘  └────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Rendering Backend (Abstract)                         │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        IRenderBackend                                 │   │
│  │  ┌─────────────────────┐         ┌─────────────────────────────────┐ │   │
│  │  │    OSGBackend       │         │   VulkanBackend (Future)        │ │   │
│  │  │  (OpenSceneGraph)   │         │                                 │ │   │
│  │  └─────────────────────┘         └─────────────────────────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Core Modules

| Module | Responsibility | Key Classes |
|--------|----------------|-------------|
| **Public API** | User-facing interfaces | `View`, `Handle`, `ViewConfig` |
| **Command Layer** | Thread-safe command execution | `CommandQueue`, `ICommand` |
| **Scene Layer** | Scene graph and object management | `SceneManager`, `NodeRegistry` |
| **Geometry Factory** | Create various geometries | `ShapeFactory`, `MeshFactory` |
| **Interaction Layer** | Handle user interactions | `PickerManager`, `GizmoController` |
| **Render Backend** | Abstract rendering interface | `IRenderBackend`, `OSGBackend` |

### Design Patterns

- **Command Pattern** - Encapsulate operations for async execution
- **Strategy Pattern** - Swappable picking strategies
- **State Machine** - Gizmo state management
- **Factory Pattern** - Geometry creation
- **Pimpl Pattern** - Hide implementation details

---

## 🚀 Installation

### Linux

```shell
# Install dependencies
sudo apt-get install build-essential python3-dev cmake git

# Install OpenSceneGraph
sudo apt install libopenscenegraph-3.4-dev
# If not available, build from source:
wget https://github.com/openscenegraph/OpenSceneGraph/archive/refs/tags/OpenSceneGraph-3.4.1.tar.gz

# Clone the repository
git clone https://github.com/rvbust/Vis.git

# Build and install
mkdir build && cd build
cmake ..
sudo make install -j

# Configure Python path
echo 'export PYTHONPATH="$PYTHONPATH:/opt/RVBUST/Vis/Python"' >> ~/.bashrc
source ~/.bashrc
```

### Windows

1. **Install OSG**

   Download OpenSceneGraph-3.4.1: [Click to Download](https://objexx.com/OpenSceneGraph/OpenSceneGraph-3.4.1-VC2017-64-Release.7z)

2. **Set Environment Variables**

   ```shell
   OSG_ROOT = C:\OpenSceneGraph-3.4.1-VC2017-64-Release
   OSG_BIN_DIR = %OSG_ROOT%\bin
   OSG_INCLUDE_DIR = %OSG_ROOT%\include
   OSG_LIB_DIR = %OSG_ROOT%\lib
   ```

   Add `OSG_BIN_DIR` to `PATH`

3. **Install PyVis**

   ```shell
   python3 Setup.py install
   ```

---

## 📖 Usage Guide

### Quick Start

```python
from RVBUST import Vis

# Create a view
v = Vis.View("My Visualization")

# Draw coordinate axes
axes = v.Axes([0, 0, 0], [0, 0, 0, 1], 1.0, 3.0)

# Draw a box
box = v.Box([0, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0])

# Go to home view
v.Home()
```

### Window Configuration

```python
# Shared scene - multiple windows show the same scene
v1 = Vis.View("View1", shared=True)
v2 = Vis.View("View2", shared=True)

# Independent scene - each window has its own scene
v1 = Vis.View("View1", shared=False)
v2 = Vis.View("View2", shared=False)

# Detailed configuration
cfg = Vis.ViewConfig()
cfg.name = "Custom View"
cfg.x, cfg.y = 100, 100
cfg.width, cfg.height = 1280, 720
cfg.bgcolor = [0.2, 0.2, 0.2, 1.0]
v = Vis.View(cfg, shared=False)
```

### Geometry Drawing

| Name | Description | Example |
|------|-------------|---------|
| **Axes** | Coordinate system | <img src=./Data/Images/Axes.png width="80"> |
| **Point** | Points/Point cloud | <img src=./Data/Images/Point.png width="80"> |
| **Line** | Line segments | <img src=./Data/Images/Line.png width="80"> |
| **Box** | Box/Cube | <img src=./Data/Images/Box.png width="80"> |
| **Sphere** | Sphere | <img src=./Data/Images/Sphere.png width="80"> |
| **Cone** | Cone | <img src=./Data/Images/Cone.png width="80"> |
| **Cylinder** | Cylinder | <img src=./Data/Images/Cylinder.png width="80"> |
| **Arrow** | Arrow | <img src=./Data/Images/Arrow.png width="80"> |
| **Mesh** | Triangle mesh | <img src=./Data/Images/Mesh.png width="80"> |
| **Plane** | Plane/Grid | <img src=./Data/Images/Plane.png width="80"> |

> **Units**: Length in meters (m), angles in radians (rad), quaternion format is (x, y, z, w)

### Model Loading

```python
from RVBUST import Vis

v = Vis.View("Robot Visualization")

# Load robot model
hs = v.Load(
    ["BaseLink.stl", "Link1.stl", "Link2.stl", "Link3.stl", "Link4.stl", "Link5.stl", "Link6.stl"],
    [[0, 0, 0], [0, 0, 0], [0.05, 0, 0.33], [0.05, 0, 0.66], [0.05, 0, 0.695], [0.385, 0, 0.695], [0.385, 0, 0.615]],
    [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0.707, 0, 0.707], [0, 0.707, 0, 0.707]]
)
```

<img src=./Data/Images/Robot.gif width="400">

### Interactive Tools

#### Picking Modes

| Mode | Function | Use Case |
|------|----------|----------|
| `IntersectorMode_Polytope` | Pick any object | General selection |
| `IntersectorMode_LineSegment` | Pick surface points | Get coordinates |
| `IntersectorMode_Point` | Pick from point cloud | Point cloud processing |
| `IntersectorMode_Line` | Pick line segments | Line selection |

```python
# Set picking mode
v.SetIntersectorMode(Vis.IntersectorMode_LineSegment)

# Get pick results
handle = v.Picked()
position_normal = v.PickedPlane()  # [x, y, z, nx, ny, nz]
```

<img src=./Data/Images/IntersectorModeLineSegment.gif width="500">

#### Gizmo Manipulator

```python
# Enable Gizmo (1=Move, 2=Rotate, 3=Scale, 4=Move+Rotate)
v.EnableGizmo(box, 4)
v.SetGizmoDisplayScale(1.5)

# Disable Gizmo
v.DisableGizmo()
```

<img src=./Data/Images/Gizmo.gif width="500">

### Complete Example

```python
from RVBUST import Vis

def main():
    # Create view
    cfg = Vis.ViewConfig()
    cfg.name = "Vis Demo"
    cfg.width, cfg.height = 800, 600
    v = Vis.View(cfg, shared=False)
    
    # Set log level
    Vis.SetLogLevel("debug")
    
    # Draw coordinate axes
    v.Axes([0, 0, 0], [0, 0, 0, 1], 2, 1)
    
    # Draw geometries
    box = v.Box([-2, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0, 0.5])
    sphere = v.Sphere([2, 0, 0], 0.5, [0, 1, 0])
    
    # Enable Gizmo
    v.EnableGizmo(box, 4)
    v.SetGizmoDisplayScale(2)
    
    # Go to home view
    v.Home()
    
    # Interactive shell
    from IPython import embed
    embed()

if __name__ == "__main__":
    main()
```

<img src=./Data/Images/move_box.gif width="700">

---

## 📁 Project Structure

```
Vis/
├── Include/Vis/          # Public headers
│   ├── Vis.h             # Main header
│   ├── Handle.h          # Handle system
│   ├── Types.h           # Basic types
│   └── View.h            # View class
├── Src/                  # Source code
│   ├── Core/             # Core components
│   ├── Commands/         # Command system
│   ├── Backend/          # Rendering backend
│   ├── Interaction/      # Interaction system
│   └── Utils/            # Utilities
├── Examples/             # Example code
│   ├── C++/
│   └── Python/
├── Data/                 # Resources
└── Externals/            # External dependencies
```

---

## 🤝 Contributing

Issues and Pull Requests are welcome!

## 📄 License

[MIT License](./LICENSE)

---

<div align="center">
Made with ❤️ by <a href="https://github.com/rvbust">RVBUST Inc.</a>
</div>

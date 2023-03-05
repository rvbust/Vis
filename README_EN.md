English | [简体中文](./README.md)

# Vis

> Interactive & Asynchronous 3D Visualization Tool - Make 3D Vision and Robotics Development Easier.  

## Introduction


## Installation

#### Linux
Before compiling Vis, please make sure that the necessary dependencies have been installed. You can install the dependencies of Vis by following the instructions：

```shell
sudo apt-get install build-essential python3-dev cmake git
```

Install libopenscenegraph [version >= 3.6.0]
```shell
sudo apt-get install libopenscenegraph-dev
```
If the `libopenscenegraph-dev` is not available on your platform, but `libopenscenegraph-3.4-dev` is, please use the `master` branch:

Clone the repo.
```shell
git clone https://github.com/rvbust/Vis.git
```
CMake is being used to build Vis: 
```cmake
mkdir build
cd build
cmake ..
sudo make install -j 
```
If you want to use the its python API, put following line into your .bashrc

```shell
export PYTHONPATH="$PYTHONPATH:/opt/RVBUST/Vis/Python"
```
#### Window

Before install Vis, please make sure OSG is installed.

##### Install OSG

1. Download windows binary file

   [Click to Download OpenSceneGraph-3.6.3](https://objexx.com/OpenSceneGraph/OpenSceneGraph-3.6.3-VC2017-64-Release.7z)
   
   Or you can compile OSG by yourself. 

2. Extract binary file and set the system path

    Extra 7z file and set below parameters to System Parameters.
    Add `OSG_ROOT` to `path`

   ```shell
    OSG_ROOT = C:\OpenSceneGraph-3.6.3-VC2017-64-Release
    OSG_BIN_DIR = %OSG_ROOT%\bin
    OSG_INCLUDE_DIR = %OSG_ROOT%\include
    OSG_LIB_DIR = %OSG_ROOT%\lib
   ```

3. Reboot your system. 

##### Install Vis

```shell
python3 Setup.py install
```

## Features

#### Set View Window Config

1. Sharing the same scene from different Views.

```python
# Sharing the scene means above two views are adding boxes in the same scene
from RVBUST import Vis 
v1 = Vis.View("View1", shared = True) #< shared means sharing the scene or not.
v2 = Vis.View("View2", shared = True)

v1.Box((0,0,0), (1,1,1))
v2.Box((2,2,2), (1,1,1))
```

2.  Not sharing the scene

```python
# Below two views are adding boxes into different scenes
v1 = Vis.View("View1", shared = False) #< shared means sharing the scene or not.
v2 = Vis.View("View2", shared = False) 

v1.Box((0,0,0), (1,1,1))
v2.Box((2,2,2), (1,1,1))
```

Inside Vis, there's only one shared scene. So if you turn on sharing all scene will be added into the same one.
The advantage is you could have multiple views for one scene.
If you create a view without sharing the scene, it will be the only view that could be used on the its scene.

#### Plot Geometrics Objects

| Name     | Description                                                                                                                                            | image                                            |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------ |
| Axes     | Plot an Axes with translations and quaternions and set asis len and axis size. The unit of axis length is meters, and the unit of dimension is pixels. | <img src=./Data/Images/Axes.png width="100">     |
| Point    | Plot point or points. Every 3 values is point position.The unit of size is pixels.                                                                     | <img src=./Data/Images/Point.png width="100">    |
| Line     | Plot line or lines. lines.size() = line_num \* 6. Every 6 values is two end points of a line.                                                          | <img src=./Data/Images/Line.png width="100">     |
| Box      | Plot a box with center position and extents.                                                                                                           | <img src=./Data/Images/Box.png width="100">      |
| Sphere   | Plot a sphere with center position and radius.                                                                                                         | <img src=./Data/Images/Sphere.png width="100">   |
| Cone     | Plot a cone with center, radius and hight.                                                                                                             | <img src=./Data/Images/Cone.png width="100">     |
| Cylinder | Plot a cylinder with center position, radius and height.                                                                                               | <img src=./Data/Images/Cylinder.png width="100"> |
| Arrow    | Plot an arrow with tail position, head position and radius.                                                                                            | <img src=./Data/Images/Arrow.png width="100">    |
| Mesh     | Plot a mesh with vertics and indices.                                                                                                                  | <img src=./Data/Images/Mesh.png width="100">     |
| Plane    | Plot a plane.                                                                                                                                          | <img src=./Data/Images/Plane.png width="100">    |

It should be noted that the units used in Vis are all SI units, that is, the length is meters (m), and the angle units are radians (rad). In addition, for orientation, Vis uses quaternion, in the form of (ox, oy, oz, ow), the real part is the last element.

#### Load Model
Vis supports common 3D model file formats, such as STL, DAE, 3DS, etc.
Vis does not currently support exporting model files.

```python
from RVBUST import Vis 
v = Vis.View("Test")
### You can import multiple files at once and set their positions and rotations.
hs = v.Load(["BaseLink.stl","Link1.stl","Link2.stl","Link3.stl","Link4.stl","Link5.stl","Link6.stl"],
[[0.0,0,0],[0, 0, 0],[0.05, 0, 0.33],[0.05, 0, 0.66],[0.05, 0, 0.695],[0.385, 0, 0.695],[0.385, 0, 0.615]],
[[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,1],[0, 0.707108, 0, 0.707105],[0, 0.707108, 0, 0.707105]])

```
<img src=./Data/Images/Robot.gif width="500">


##### Interactive
Vis provides a variety of interactive APIs.

1. Select
- IntersectorMode_Polytope
> In IntersectorMode_Polytope mode, you can click on an object in the scene and highlight the selected object.
```python
v.SetIntersectorMode(IntersectorMode_Polytope)
```
<img src=./Data/Images/IntersectorModePolytope.gif width="500">

- IntersectorMode_LineSegment
> In IntersectorMode_LineSegment mode, you can select points on the surface of the object by ctrl+left-click and draw the axes.
```python
v.SetIntersectorMode(IntersectorMode_LineSegment)
hs=v.GetPickedPointAxes()
```
<img src=./Data/Images/IntersectorModeLineSegment.gif width="500">

- IntersectorMode_Point
> In IntersectorMode_Point mode, you can select a point in a set of points.
```python
v.SetIntersectorMode(IntersectorMode_Point)
hs=v.GetPickedPointAxes()
```
<img src=./Data/Images/IntersectorModePoint.gif width="500">

- IntersectorMode_Line
> In IntersectorMode_Line mode, you can select the line segment drawn with View.Line(...).
```python
v.SetIntersectorMode(IntersectorMode_Line)
```
<img src=./Data/Images/IntersectorModeLine.gif width="500">

2. Gizmo
Gizmo is a tool that allow you to manipulate a geometry object(Rotate & translate at the moment).

```python
from RVBUST import Vis
v = Vis.View("Test")
box = v.Box([0,0,0],[0.5,0.5,0.5])
axes = v.Axes([0,0,0],[0,0,0,1],1,1)
v.EnableGizmo(box,4)
v.SetGizmoDisplayScale(0.6)
```

<img src=./Data/Images/Gizmo.gif width="500">
   
#### Miscellaneous

Vis can set different log levels to view the corresponding information. The log levels are info, debug, warn, error:

```python
# by default, the log level is error. If you want to change it, you could following function
# at any time:
Vis.SetLogLevel("debug")
```

#### Use Case

The following is a simple case of using Vis, a box of a given size is displayed in the Vis window, and the user can move the box in the window.

```
from RVBUST import Vis
from IPython import embed

# Show box in Vis
def ShowBox():
    transparent_box = v.Box([-2, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0, 0.5])
    normal_box = v.Box([0, 2, 0], [0.5, 0.5, 0.5], [1, 0, 0])

# Move box with Gizmo
def MoveBox():
    transparent_box = v.Box([-2, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0, 0.5])
    v.EnableGizmo(transparent_box, 4)
    v.SetGizmoDisplayScale(2)

def Basic():
    Vis.SetLogLevel("debug")
    axes1 = v.Axes([0, 0, 0], [0, 0, 0, 1], 2, 1)
    ShowBox()
    MoveBox()
    pass

if __name__ == "__main__":
    cfg = Vis.ViewConfig()
    cfg.x = 0
    cfg.y = 40
    cfg.width = 800
    cfg.height = 600
    cfg.name = "Basics"
    v = Vis.View(cfg, False)
    Basic()
    v.Home()
    from IPython.terminal import embed
    ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
```

<img src=./Data/Images/move_box.gif width="700">

## License

[MIT](/LICENSE)



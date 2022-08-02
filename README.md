

[English](./README_EN.md) | 简体中文

<div align = "center">
<img src=./Data/Images/logo.png width="180"/>
</div>



# Vis: Asynchronous 3D Visualization Tool

Vis 是一款交互式异步3D可视化工具，旨在让3D视觉和机器人应用开发更简单。

其核心功能包括：

- 图形绘制

- 3D模型文件导入

- 多种交互工具

- Gzimo

  

## 安装

- #### Linux


```shell
  # 安装必要的依赖项
  sudo apt-get install build-essential python3-dev cmake git
  
  # 安装libopenscenegraph
  sudo apt install libopenscenegraph-3.4-dev
  # 如果找不到以上版本，可以安装默认版本
  sudo apt install libopenscenegraph-dev
  
  # 下载Vis源码
  git clone https://github.com/rvbust/Vis.git
  # 编译
  mkdir build
  cd build
  cmake ..
  sudo make install -j
  
  # 如果想要使用Vis的Python API,需要在文件.bashrc中加入下列指令：
  export PYTHONPATH="$PYTHONPATH:/opt/RVBUST/Vis/Python"
```

- #### Windows


  安装Vis之前，请确定已经安装OSG。

   ##### **安装OSG**

  1. 下载OSG安装包

     OpenSceneGraph-3.4.1[点击下载](https://objexx.com/OpenSceneGraph/OpenSceneGraph-3.4.1-VC2017-64-Release.7z)

  2. 解压文件和设置路径

     解压7z文件和编辑系统环境变量
     Add `OSG_BIN_DIR` to `path`

      ```shell
      OSG_ROOT = C:\OpenSceneGraph-3.4.1-VC2017-64-Release
      OSG_BIN_DIR = %OSG_ROOT%\bin
      OSG_INCLUDE_DIR = %OSG_ROOT%\include
      OSG_LIB_DIR = %OSG_ROOT%\lib
      ```

  3. 重启系统

  #####   安装 PyVis

  ```shell
  python3 Setup.py install
  ```



## 功能说明

- #### 窗口设置


1. 在不同窗口共享相同场景.

```python
# 共享场景表示两个视图会显示相同的场景。
from RVBUST import Vis 
v1 = Vis.View("View1", shared = True) #< Shared表示是否共享场景.
v2 = Vis.View("View2", shared = True) 

v1.Box((0,0,0), (1,1,1))
v2.Box((2,2,2), (1,1,1))
```

2. 不共享场景

```python
# 不共享场景表示两个视图会显示各自的场景。
v1 = Vis.View("View1", shared = False) #< Shared表示是否共享场景.
v2 = Vis.View("View2", shared = False)

v1.Box((0,0,0), (1,1,1))
v2.Box((2,2,2), (1,1,1))
```

在 Vis 中，只有一个共享场景。 因此，如果您打开共享，所有场景都将添加到同一个视图中。这样做的优点是一个场景可以有多个视图。
如果你在不共享场景的情况下创建视图，它将是唯一可以在其场景中使用的视图。

- #### 图形绘制


| 名称     | 描述                                                                                                     | image                                            |
| -------- | -------------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| Axes     | 通过输入位置，旋转姿态来绘制坐标系，可以设置坐标系轴的长度和大小，其中长度单位为米，大小尺寸单位为像素。 | <img src=./Data/Images/Axes.png width="100">     |
| Point    | 输入一组点数据来绘制一个点或是一组点，每3个值来表达点的位置，大小尺寸单位为像素。                        | <img src=./Data/Images/Point.png width="100">    |
| Line     | 绘制一条线段或是一组线段，其中每6个值表达一条线段，分别为线段的起点和终点位置。                          | <img src=./Data/Images/Line.png width="100">     |
| Box      | 通过输入中心位置和尺寸来绘制一个盒子。                                                                   | <img src=./Data/Images/Box.png width="100">      |
| Sphere   | 通过输入中心位置和半径来绘制一个球。                                                                     | <img src=./Data/Images/Sphere.png width="100">   |
| Cone     | 通过输入形心位置，半径和高度来绘制一个圆锥。                                                             | <img src=./Data/Images/Cone.png width="100">     |
| Cylinder | 通过输入中心位置，半径和高度绘制一个圆柱。                                                               | <img src=./Data/Images/Cylinder.png width="100"> |
| Arrow    | 通过输入尾部位置，头部位置和半径绘制一个箭头.                                                            | <img src=./Data/Images/Arrow.png width="100">    |
| Mesh     | 通过输入3角面片顶点位置和3角面片索引来绘制一个Mesh.                                                      | <img src=./Data/Images/Mesh.png width="100">     |
| Plane    | 绘制一个平面。                                                                                           | <img src=./Data/Images/Plane.png width="100">    |

需要注意的是，Vis 中使用的单位都是国际标准单位制 SI，也即长度为米（m），角度单位为弧度（rad）；此外，对于姿态描述，Vis中采用的是四元数形式，其中四元数的实部在最后一位，也即 (ox, oy, oz, ow) 格式。

- #### 模型导入

Vis 支持常见 3D 模型文件格式，如STL，DAE，3DS等等。
Vis 暂时不支持导出模型文件。

  ```python
from RVBUST import Vis
v = Vis.View("Test")
hs = v.Load(["Models/Visual/BaseLink.stl","Models/Visual/Link1.stl","Models/Visual/Link2.stl",
"Models/Visual/Link3.stl","Models/Visual/Link4.stl","Models/Visual/Link5.stl","Models/Visual/Link6.stl"],
[[0.0,0,0],[0, 0, 0],[0.05, 0, 0.33],[0.05, 0, 0.66],[0.05, 0, 0.695],[0.385, 0, 0.695],[0.385, 0, 0.615]],
[[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,1],[0, 0.707108, 0, 0.707105],[0, 0.707108, 0, 0.707105]])

  ```
<img src=./Data/Images/Robot.gif width="400">

- #### 交互工具


Vis提供多种交互工具。

1. 选择工具

- IntersectorMode_Polytope

> 在 IntersectorMode_Polytope 模式，可以通过右键点击场景的物体，并高亮显示选中的物体。(能选择点和线段，但是不能获取到选中点的坐标)

```python
v.SetIntersectorMode(IntersectorMode_Polytope)
picked_object_handle = v.Picked()
```

<img src=./Data/Images/IntersectorModePolytope.gif width="500">

- IntersectorMode_LineSegment

> 在 IntersectorMode_LineSegment 模式下，可以通过右键点击场景的物体，并高亮显示选中的物体(不能选择点和线段, 可以获取到选中点的坐标)。还可以通过 Ctrl + 右键点击选择多个物体表面的点，并绘制坐标系。

```python
v.SetIntersectorMode(IntersectorMode_LineSegment)
picked_object_handle = v.Picked()
postion_normal = v.PickedPlane()
axes_handles = v.GetPickedPointAxes()
hs = v.MultiPicked()
```

<img src=./Data/Images/IntersectorModeLineSegment.gif width="500">

- IntersectorMode_Point

> 在 IntersectorMode_Point 模式下，可以通过右键点击在一组点中选择一个点，并在选中的点的位置绘制坐标系。还可以通过 Ctrl + 右键点击选择多个点，并绘制坐标系。

```python
v.SetIntersectorMode(IntersectorMode_Point)
picked_object_handle = v.Picked()
postion_normal = v.PickedPlane()
axes_handles = v.GetPickedPointAxes()
hs = v.MultiPicked()
```

<img src=./Data/Images/IntersectorModePoint.gif width="500">

- IntersectorMode_Line

> 在 IntersectorMode_Line 模式下，可以选中用 View.Line 绘制的线段和 View.Axes 绘制的坐标系，选中的线段会变成高亮。还可以通过 Ctrl + 右键点击选择多条线段。

```python
v.SetIntersectorMode(IntersectorMode_Line)
picked_object_handle = v.Picked()
hs = v.MultiPicked()
postion_normal = v.PickedPlane()
```

<img src=./Data/Images/IntersectorModeLine.gif width="500">

2. Gzimo

Gizmo 是一个操作场景中物体的工具，可以通过拖拉改变物体的位置和姿态。

```python
from RVBUST import Vis
v = Vis.View("Test")
box = v.Box([0,0,0],[0.5,0.5,0.5])
axes = v.Axes([0,0,0],[0,0,0,1],1,1)
v.EnableGizmo(box,4)
v.SetGizmoDisplayScale(0.6)
```

- #### 日志功能

Vis可以设置不同日志等级来查看对应的信息，日志等级有info，debug，warn，error:

```python
Vis.SetLogLevel("debug")
```

##  使用示例

以下是 Vis 使用的简单案例，在 Vis 窗口中展示给定尺寸的 box，用户可在窗口中对 box 进行移动。

```
from RVBUST import Vis
from IPython import embed

# 在Vis窗口中展示给定尺寸的box
def ShowBox():
    transparent_box = v.Box([-2, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0, 0.5])
    normal_box = v.Box([0, 2, 0], [0.5, 0.5, 0.5], [1, 0, 0])

# 移动box
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

